"""
Simple package for receiving and sending data over serial. Data is framed in packets:
|START|LEN|ID|... PAYLOAD BYTES ...|CRC|END|
Each packet has:
    id: 0-255 identifier byte
    payload: any data bytes
Packets are received and sent using two threads handling the serial port.
Callback targeting specific packet ids are supported. Packets that have a callback assigned are not placed in the
read queue!
"""

from queue import Queue, Full, Empty
from serial import Serial, SerialException
from threading import Thread
import struct
from time import sleep, time
from warnings import warn
import crcmod


def bytes2str(b):
    if not isinstance(b, (bytes, bytearray)):
        raise TypeError("b must be bytes or bytearray instance.")
    if isinstance(b, bytearray):
        b = bytes(b)
    return b.decode("UTF-8")


def str2bytes(s):
    if not isinstance(s, str):
        raise TypeError("s must be string instance.")
    return s.encode("UTF-8")


def bytes2int(b):
    if not isinstance(b, (bytes, bytearray)):
        raise TypeError("b must be bytes or bytearray instance.")
    if isinstance(b, bytearray):
        b = bytes(b)
    return int.from_bytes(b, byteorder="little")


def int2bytes(i):
    """
    Convert int to bytes.
    :param i: 32 bit integer inside range -2147483648, 2147483647
    :return: 4 bytes
    """
    if not isinstance(i, int):
        raise TypeError("i must be int")
    return struct.pack("i", i)


def bytes2float(b):
    if not isinstance(b, (bytes, bytearray)):
        raise TypeError("b must be bytes or bytearray instance.")
    if isinstance(b, bytearray):
        b = bytes(b)
    return struct.unpack("f", b)[0]


def float2bytes(f):
    """
    Convert float to bytes.
    :param f: float
    :return: 4 bytes
    """
    if isinstance(f, int):
        f = float(f)
    elif not isinstance(f, float):
        raise TypeError("f must be float")
    b = struct.pack("f", f)
    return b


class SimpleSerialException(Exception):
    pass


class FrameError(SimpleSerialException):
    """ Raised when received frame is invalid. """
    pass


class ReplyTimeout(SimpleSerialException):
    """ Raised when reply is not received in time. """
    pass


class CRCError(SimpleSerialException):
    """ Raised when CRC does not match. """
    pass


class SimpleSerial:
    """
    Create simple_serial_python object.
    """

    def __init__(self, port, baud, *args, start=0x02, end=0x03, esc=0x01, payload_max_len=8,
                 packet_timeout=1.0, queue_max_len=100, **kwargs):
        """
        :param port: serial port
        :param baud: baud rate
        :param start: start byte flag value
        :param end: end byte flag value
        :param esc: esc byte flag value
        :param payload_max_len: maximum payload length in bytes
        :param packet_timeout: time passed after start when packed is discarded
        :param queue_max_len: maximum receive, send, callback queue length
        :param args: args passed to Serial
        :param kwargs: kwargs passed to Serial
        """
        self.received_queue = Queue(maxsize=queue_max_len)
        self.send_queue = Queue(maxsize=queue_max_len)

        self.byte_count = 0
        self.current_id = None
        self.current_len = None
        self.current_payload = None
        self.esc_active = False

        self.serial = Serial(baudrate=baud, timeout=1, *args, *kwargs)
        self.serial.port = port
        self.serial_alive = False

        self.START = start
        self.END = end
        self.ESC = esc
        self.payload_max_len = payload_max_len
        self.packet_timeout = packet_timeout

        self.receive_thread: Thread = None
        self.send_thread: Thread = None
        self.callback_thread: Thread = None

        self.callbacks = {}
        self.callback_queue = Queue(maxsize=queue_max_len)

        self.packet_start_time = 0

        self.is_open = False

    def open(self):
        """
        Open serial communication, start receive/send/callback threads.
        """
        self.is_open = True
        self.serial.open()
        self.serial_alive = True

        self.receive_thread = Thread(target=self.receive_worker, daemon=True)
        self.send_thread = Thread(target=self.send_worker, daemon=True)
        self.callback_thread = Thread(target=self.callback_worker, daemon=True)
        self.receive_thread.start()
        self.send_thread.start()
        self.callback_thread.start()

    def close(self, wait_send=True):
        """
        Close serial communication, stop threads.
        :param wait_send: True to wait until all pending messages are sent.
        """
        if wait_send:
            while self.send_queue.qsize() > 0:
                sleep(1/1000)
        self.is_open = False
        self.receive_thread.join()
        self.send_thread.join()
        self.callback_thread.join()
        self.serial.close()
        self.serial_alive = False

    def send(self, id, payload, wait_reply=False, reply_has_payload=False, timeout=1.0, retry=0):
        """
        Convert payload to bytes from int/float/string and puts it to send queue.
        Ignores packet if send queue is full.
        :param id: packet id
        :param payload: bytes, int, float, str
        :param wait_reply: True to wait for reply
        :param reply_has_payload: True if reply has payload. If true, returns replied payload.
        :param timeout: time to wait for reply
        :param retry: number of retries if reply is not received. Total number of tries is 1 + retry
        """
        # Check type
        if isinstance(payload, bytes):
            pass
        elif isinstance(payload, int):
            payload = int2bytes(payload)
        elif isinstance(payload, float):
            payload = float2bytes(payload)
        elif isinstance(payload, str):
            payload = str2bytes(payload)
        else:
            raise TypeError("Argument is not bytes/int/float/string.")
        # Check length
        if len(payload) > self.payload_max_len:
            raise ValueError("Payload (len={}) must not be longer than payload_max_len={}."
                             .format(len(payload), self.payload_max_len))
        frame = self.frame(id, payload)

        if wait_reply:

            reply = {"occured": False,
                     "payload": None}

            def callback(payload):
                reply["occured"] = True
                reply["payload"] = payload

            prev_callback = self.callbacks.get("id", None)
            self.set_callback(id, callback)

        try:
            self.send_queue.put(frame, block=False)
        except Full:
            pass

        if wait_reply:
            for r in range(1, retry+2):
                start_time = time()
                break_for = False
                while True:
                    if reply["occured"]:
                        if reply_has_payload:
                            ret = reply["payload"]
                        else:
                            ret = None
                        break_for = True
                        break
                    if time() > start_time + timeout:
                        break
                    sleep(0.001)
                if break_for:
                    break
            else:
                raise ReplyTimeout("Reply not received after {}x {}s tries for packet id '{}'.".format(r, timeout, id))

            if not prev_callback:
                self.clear_callback(id)
            else:
                self.set_callback(id, prev_callback)
            return ret

    def read(self, block=True, timeout=None):
        """
        Return item from received packets queue.
        :param block: true to block until packet available
        :param timeout: if none, blocks indefinitely, otherwise Empty exception is raised if no packet available
                        after timeout seconds
        :return tuple: id, payload
        """
        return self.received_queue.get(block, timeout)

    def escape(self, data):
        """
        Escapes data with ESC byte.
        :param data: data to escape. type: bytes or bytearray
        :return: escaped bytearray
        """
        escaped = bytearray()
        for b in data:
            if b not in [self.START, self.END, self.ESC]:
                escaped.append(b)
            else:
                escaped.append(self.ESC)
                escaped.append(b)
        return escaped

    def unescape(self, data):
        """
        Remove escape bytes from data
        :param data: data to remove escapes from, type: bytes or bytearray
        :return: bytearray
        """
        unescaped = bytearray()
        esc = False
        for b in data:
            if not esc:
                if b != self.ESC:
                    unescaped.append(b)
                else:
                    esc = True
                    continue
            else:
                unescaped.append(b)
                esc = False
        return unescaped

    def frame(self, id, payload):
        """
        Frame payload data. Insert START, END and ESC bytes, length and id.
        :param id: packet id
        :param payload: data to frame, type bytes or bytearray
        :return: framed data
        """
        if not 0 <= id < 256:
            raise ValueError("id must be in range(0, 255)")
        payload = bytearray(payload)
        # Extend payload with calculated CRC
        payload.extend(self.calc_crc(payload))

        packet = bytearray()
        packet.append(self.START)  # start byte
        packet.append(0)  # length byte placeholder
        packet.append(id)  # id byte
        packet += self.escape(payload)  # escaped payload
        packet.append(self.END)  # end byte
        packet[1] = len(packet)  # set length byte to length of whole frame
        return packet

    def unframe(self, packet):
        """
        Un-frame data. Remove flag bytes
        Raises FrameError if frame is invalid.
        :param packet: framed data
        :return: dict with keys: length, id, payload
        """
        print(packet)
        if packet[0] != self.START:
            raise FrameError("START byte missing.")
        if packet[-1] != self.END:
            raise FrameError("END byte missing.")
        length = packet[1]
        if length != len(packet):
            raise FrameError("Length mismatch.")
        id = packet[2]
        payload = self.unescape(packet[3:-1])

        # Extract and verify CRC
        crc = payload.pop(-1)
        crc_calc = self.calc_crc(payload)
        if crc != crc_calc:
            raise CRCError()

        return {"len": length, "id": id, "payload": payload}

    @staticmethod
    def calc_crc(bts):
        """
        Calculate CRC-8 value of input bytes.
        :param bts: sequence of bytes, type bytes or bytearray
        :return: single byte CRC-8, type bytes
        """
        crc8 = crcmod.predefined.Crc("crc-8")
        crc8.update(bts)
        return crc8.digest()

    def read_packet(self, b):
        """
        Read packet byte by byte. Called when new byte is available.
        :param b: incoming byte
        """
        if len(b) < 1:
            return
        # if time() - self.packet_start_time > self.packet_timeout:
        #     self.byte_count = 0
        #     self.esc_active = False
        #     return
        b = b[0]
        # Wait for START
        if self.byte_count == 0:
            if b == self.START:
                self.current_len = None
                self.current_id = None
                self.current_payload = bytearray()
                self.byte_count += 1
                self.packet_start_time = time()
        # Length byte
        elif self.byte_count == 1:
            self.current_len = b
            self.byte_count += 1
        # Id byte
        elif self.byte_count == 2:
            self.current_id = b
            self.byte_count += 1
        # Payload bytes
        elif self.byte_count >= 3:
            if not self.esc_active:
                if b == self.ESC:
                    self.esc_active = True
                    return
                if b == self.END:
                    self.byte_count += 1
                    # End of frame, verify
                    crc_received = bytes([self.current_payload[-1]])
                    payload = self.current_payload[0:-1]
                    crc_calculated = self.calc_crc(payload)
                    if self.current_len == self.byte_count and crc_received == crc_calculated:
                        self.process_callback(self.current_id, payload)
                        self.received_queue.put((self.current_id, payload), block=False)
                    else:
                        pass
                    self.byte_count = 0
                    return
                else:
                    self.current_payload.append(b)
                    self.byte_count += 1
            else:
                self.current_payload.append(b)
                self.byte_count += 1
                self.esc_active = False
            if self.byte_count > self.current_len or time() - self.packet_start_time > self.packet_timeout:
                # Reset
                self.byte_count = 0
                self.esc_active = False

    def receive_worker(self):
        """
        Receive worker function. Reads serial port byte by byte.
        """
        while self.is_open:
            if self.serial_alive:
                try:
                    b = self.serial.read()
                except SerialException:
                    self.serial_alive = False
                    self.restart()
                else:
                    self.read_packet(b)

    def send_worker(self):
        """
        Send worker function. Takes frame from send queue and sends it over serial.
        """
        while self.is_open:
            if self.serial_alive:
                try:
                    frame = self.send_queue.get(block=False)
                except Empty:
                    pass
                else:
                    try:
                        self.serial.write(frame)
                    except SerialException:
                        self.serial_alive = False
                        self.restart()
            sleep(1/1e3)

    def restart(self):
        """
        Close and reopen the serial port.
        """
        print("Serial port closed unexpectedly. Trying to reopen...")
        while True:
            try:
                self.serial.close()
                self.serial.open()
            except SerialException:
                pass
            else:
                self.serial_alive = True
                print("Reopened serial port.")
                break
            sleep(1)

    def set_callback(self, id, callback):
        """
        Set a function to be called when a packet with certain id is received.
        Arguments passed to callback function: id, payload
        :param id: packet id
        :param callback: function to call
        """
        self.callbacks[id] = callback

    def clear_callback(self, id):
        """
        Remove callback function at certain message id.
        :param id: packet id
        """
        self.callbacks.pop(id, None)

    def process_callback(self, id, payload):
        """
        Called when new packet is received.
        Check if callback for packet id is set. If true, put the callback function in callback queue.
        :param id: packed id
        :param payload: packet payload
        """
        try:
            cb = self.callbacks[id]
        except KeyError:
            return
        else:
            try:
                self.callback_queue.put((cb, payload), block=False)
            except Full:
                pass

    def callback_worker(self):
        """
        Call functions from callback queue. Pass packet id, len, payload as arguments.
        """
        while self.is_open:
            try:
                callback, payload = self.callback_queue.get(block=False)
            except Empty:
                pass
            else:
                try:
                    callback(payload)
                except Exception as e:
                    warn("Exception occurred during callback for msg id '{}': {}".format(id, e))
            sleep(1/1000)


if __name__ == '__main__':
    ss = SimpleSerial("/dev/cu.usbserial-FTA02V6E", 115200)
    ss.set_callback(1, lambda pld: print(pld))
    ss.open()
    try:
        while True:
            ss.send(1, 12.345)
            sleep(1)
    finally:
        ss.close()
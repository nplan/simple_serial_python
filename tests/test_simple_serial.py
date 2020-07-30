import serial
import simple_serial
from math import isclose
from time import time, sleep


def test_escape():
    ser = serial.serial_for_url('loop://', baudrate=115200, timeout=1, do_not_open=True)
    ss = simple_serial.SimpleSerial(serial=ser)

    # data with no bytes identical to flags
    payload = bytearray([0, 0, 0, 0])
    escaped = ss.escape(payload)
    assert escaped == bytearray([0, 0, 0, 0])

    # data with bytes identical to flag bytes
    payload = bytearray([ss.START, ss.ESC, 0, ss.END])
    escaped = ss.escape(payload)
    assert escaped == bytearray([ss.ESC, ss.START, ss.ESC, ss.ESC, 0, ss.ESC, ss.END])

    payload = bytearray([ss.ESC, ss.ESC, ss.ESC, ss.ESC])
    escaped = ss.escape(payload)
    assert escaped == bytearray([ss.ESC, ss.ESC, ss.ESC, ss.ESC, ss.ESC, ss.ESC, ss.ESC, ss.ESC])

    payload = bytearray([ss.START, ss.START, ss.START, ss.START])
    escaped = ss.escape(payload)
    assert escaped == bytearray([ss.ESC, ss.START, ss.ESC, ss.START, ss.ESC, ss.START, ss.ESC, ss.START])
    
    payload = bytearray([ss.END, ss.END, ss.END, ss.END])
    escaped = ss.escape(payload)
    assert escaped == bytearray([ss.ESC, ss.END, ss.ESC, ss.END, ss.ESC, ss.END, ss.ESC, ss.END])


def test_unescape():
    ser = serial.serial_for_url('loop://', baudrate=115200, timeout=1, do_not_open=True)
    ss = simple_serial.SimpleSerial(serial=ser)

    # data with no bytes identical to flags
    payload = bytearray([0, 0, 0, 0])
    unescaped = ss.unescape(payload)
    assert unescaped == bytearray([0, 0, 0, 0])

    # data with bytes identical to flag bytes
    payload = bytearray([ss.ESC, ss.ESC, ss.ESC, ss.ESC])
    unescaped = ss.unescape(payload)
    assert unescaped == bytearray([ss.ESC, ss.ESC])

    payload = bytearray([ss.ESC, ss.START, ss.ESC, ss.ESC, 0, ss.ESC, ss.END])
    unescaped = ss.unescape(payload)
    assert unescaped == bytearray([ss.START, ss.ESC, 0, ss.END])


def test_escape_unescape():
    ser = serial.serial_for_url('loop://', baudrate=115200, timeout=1, do_not_open=True)
    ss = simple_serial.SimpleSerial(serial=ser)

    # data with no bytes identical to flags
    payload = bytearray([0, 0, 0, 0])
    escaped = ss.escape(payload)
    unescaped = ss.unescape(escaped)
    assert unescaped == payload

    # data with bytes identical to flag bytes
    payload = bytearray([ss.START, ss.ESC, 0, ss.END])
    escaped = ss.escape(payload)
    unescaped = ss.unescape(escaped)
    assert unescaped == payload

    # different flags
    for i in range(85):
        start = i
        end = 2 * i
        esc = 3 * i
        ss = simple_serial.SimpleSerial(serial=ser, start=start, end=end, esc=esc)
        payload = bytearray([ss.START, ss.ESC, 0, ss.END])
        escaped = ss.escape(payload)
        unescaped = ss.unescape(escaped)
        assert unescaped == payload


def test_crc():
    ser = serial.serial_for_url('loop://', baudrate=115200, timeout=1, do_not_open=True)
    ss = simple_serial.SimpleSerial(serial=ser)

    payload = bytearray([0, 0, 0, 0])
    assert ss.calc_crc(payload) == bytes([0])

    payload = bytearray([1, 2, 3, 4])
    assert ss.calc_crc(payload) == bytes([227])

    payload = bytearray([1, 2, 3, 4])
    payload_corrupted = bytearray([1, 2, 3, 4+1])
    assert ss.calc_crc(payload) != ss.calc_crc(ss.calc_crc(payload_corrupted))


def test_frame():
    ser = serial.serial_for_url('loop://', baudrate=115200, timeout=1, do_not_open=True)
    ss = simple_serial.SimpleSerial(serial=ser)

    # simple case
    id = 5
    payload = bytearray([5, 6, 7, 8])
    framed = ss.frame(id, payload)
    assert framed == bytearray([ss.START, 9, id, 5, 6, 7, 8, 96, ss.END])

    # flags in payload
    id = id
    payload = bytearray([ss.ESC, ss.START, ss.ESC, ss.END])
    framed = ss.frame(id, payload)
    correct = bytearray()
    correct.append(ss.START)
    correct.append(0)
    correct.append(id)
    correct += ss.escape(payload)
    correct.append(ss.calc_crc(payload)[0])
    correct.append(ss.END)
    correct[1] = len(correct)
    assert framed == correct


def test_frame_unframe():
    ser = serial.serial_for_url('loop://', baudrate=115200, timeout=1, do_not_open=True)
    ss = simple_serial.SimpleSerial(serial=ser)

    # simple case
    payload = bytearray([5, 6, 7, 8])
    framed = ss.frame(4, payload)
    unframed = ss.unframe(framed)
    assert unframed["len"] == len(framed) and unframed["id"] == 4 and unframed["payload"] == payload

    # flags in payload
    payload = bytearray([ss.START, ss.ESC, ss.END, 0, 123, 0, ss.ESC])
    framed = ss.frame(10, payload)
    unframed = ss.unframe(framed)
    assert unframed["len"] == len(framed) and unframed["id"] == 10 and unframed["payload"] == payload

    # every possible 1 byte payload
    for b1 in range(255):
        payload = bytearray([b1])
        framed = ss.frame(111, payload)
        unframed = ss.unframe(framed)
        assert unframed["len"] == len(framed) and unframed["id"] == 111 and unframed["payload"] == payload


def test_conversions():
    f = 1.234567
    b = simple_serial.float2bytes(f)
    f2 = simple_serial.bytes2float(b)
    assert isclose(f, f2, rel_tol=1e-6)

    i = 12345
    b = simple_serial.int2bytes(i)
    i2 = simple_serial.bytes2int(b)
    assert i == i2


def test_send_receive():
    ser = serial.serial_for_url('loop://', baudrate=115200, timeout=1, do_not_open=True)
    ss = simple_serial.SimpleSerial(serial=ser, payload_max_len=16)
    ss.open()

    # bytes
    id_send = 123
    payload_send = b"hello world"
    ss.send(id_send, payload_send)
    try:
        id_rec, payload_rec = ss.read(timeout=1)
        assert id_rec == id_send
        assert payload_rec == payload_send
    except:
        raise

    # float
    id_send = 123
    payload_send = 3.141592
    ss.send(id_send, payload_send)
    try:
        id_rec, payload_rec = ss.read(timeout=1)
        assert id_rec == id_send
        assert isclose(simple_serial.bytes2float(payload_rec), payload_send, rel_tol=1e-6)
    except:
        raise

    # int
    id_send = 123
    payload_send = 42069
    ss.send(id_send, payload_send)
    try:
        id_rec, payload_rec = ss.read(timeout=1)
        assert id_rec == id_send
        assert simple_serial.bytes2int(payload_rec) == payload_send
    except:
        raise

    ss.close()


def test_callback():
    ser = serial.serial_for_url('loop://', baudrate=115200, timeout=1, do_not_open=True)
    ss = simple_serial.SimpleSerial(serial=ser, payload_max_len=16)

    global callback_called
    callback_called = False

    def callback(pld):
        global callback_called
        callback_called = True

    ss.set_callback(123, callback)
    ss.open()

    ss.send(123, b"hello")
    sleep(1)
    assert callback_called


# def test_retry():
#     ser = serial.serial_for_url('loop://', baudrate=115200, timeout=1, do_not_open=True)
#     ss = simple_serial.SimpleSerial(serial=ser, payload_max_len=16)
#
#     global num_rec
#     num_rec = 0
#
#     def callback(pld):
#         global num_rec
#         num_rec += 1
#         print("clbk")
#
#     ss.set_callback(123, callback)
#     ss.open()
#
#     num_resend = 5
#     timeout = 0.1
#     ss.send(123, b"hello", expect_reply=True, resend=num_resend, reply_timeout=timeout)
#     sleep(2*2*num_resend*timeout)
#     ss.close()
#     print(num_rec)
#
#     assert num_resend == num_rec


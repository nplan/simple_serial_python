"""
Sends a float over simple serial and prints reply which is expected to be a multiple of sent value.
"""

from time import sleep
import logging
import argparse

import simple_serial


if __name__ == '__main__':
    logging.basicConfig(level=logging.ERROR)

    parser = argparse.ArgumentParser(description='Send a float and print reply that should be a multiple.')
    parser.add_argument('port', type=str, help="serial port")
    parser.add_argument('baud', type=int, nargs="?", default=115200, help="baudrate")
    args = parser.parse_args()

    ss = simple_serial.SimpleSerial(args.port, args.baud, payload_max_len=8)
    ss.open()
    sleep(2)

    i = 1
    while True:
        ss.send(123, float(i))

        id, pyld = ss.read()
        f = simple_serial.bytes2float(pyld)
        print("Sent: {:.2f}, Received: {:.2f}".format(i, f))

        i += 1
        sleep(1)

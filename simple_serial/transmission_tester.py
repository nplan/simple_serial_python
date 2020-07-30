"""
A module for testing simple_serial on external devices. External device must be running provided test program that
replies to this testing sequence.
"""

from time import sleep
import logging
import argparse

import simple_serial

test_suite_version = b"v0.1"

if __name__ == '__main__':
    logging.basicConfig(level=logging.ERROR)

    parser = argparse.ArgumentParser(description='Test simple_serial connection to external device.')
    parser.add_argument('port', type=str, help="serial port")
    parser.add_argument('baud', type=int, nargs="?", default=115200, help="baudrate")
    args = parser.parse_args()

    with simple_serial.SimpleSerial(args.port, args.baud, payload_max_len=8) as ss:
        sleep(2)

        print("Starting tests on '{}' ...".format(args.port))

        # Verify connection and test suite version

        try:
            r = ss.send(1, b"?", expect_reply=True, block_until_reply=True, reply_timeout=1.0)
        except simple_serial.ReplyTimeout:
            raise RuntimeError("External device not responding.")
        else:
            if r != test_suite_version:
                raise RuntimeError("Test suite version on external device ({}) "
                                   "does not match this version ({})".format(r.decode("ascii"),
                                                                             test_suite_version.decode("ascii")))

        tests = [
            {"test_nr": 1, "id": 11, "payload": bytes([0]), "expected_reply": b"ok"},
            {"test_nr": 2, "id": 12, "payload": b"hello", "expected_reply": b"ok"},
            {"test_nr": 3, "id": 13, "payload": 12345, "expected_reply": b"ok"},
            {"test_nr": 4, "id": 14, "payload": -4321, "expected_reply": b"ok"},
            {"test_nr": 5, "id": 15, "payload": 4.321, "expected_reply": b"ok"},
            {"test_nr": 6, "id": 16, "payload": -123.4, "expected_reply": b"ok"},
            {"test_nr": 7, "id": 17, "payload": bytes([ss.START, ss.ESC, ss.END]), "expected_reply": b"ok"},
            {"test_nr": 8, "id": 18, "payload": b"hello", "expected_reply": b"world"},
            {"test_nr": 9, "id": 19, "payload": 4567, "expected_reply": simple_serial.int2bytes(2*4567)},
            {"test_nr": 10, "id": 20, "payload": 12.34, "expected_reply": simple_serial.float2bytes(2*12.34)}
        ]

        for test in tests:
            try:
                r = ss.send(test["id"], test["payload"], expect_reply=True, block_until_reply=True, reply_timeout=1.0)
            except simple_serial.ReplyTimeout:
                print("Test {}: TIMEOUT".format(test["test_nr"]))
            else:
                if r == test["expected_reply"]:
                    print("Test {}: OK".format(test["test_nr"]))
                else:
                    print("Test {}: FAIL".format(test["test_nr"]))
            sleep(0.1)

        sleep(1)
        print("Testing completed.")

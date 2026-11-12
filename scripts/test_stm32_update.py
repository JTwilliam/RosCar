#!/usr/bin/env python3
"""Offline safety/encoding checks; never opens a real serial device."""
import struct
import unittest

import stm32_update as update
from learm_bridge.protocol import ProtocolDecoder


class FakePort:
    def __init__(self, reply=None):
        self.writes, self.buffer = [], bytearray()
        self.reply = reply or (lambda frame: (0xF0, struct.pack('<HB', frame.sequence, 0)))

    @property
    def in_waiting(self):
        return len(self.buffer)

    def write(self, data):
        frame = ProtocolDecoder().feed(data)[0]
        self.writes.append(frame)
        response = self.reply(frame)
        if response:
            self.buffer.extend(update.encode_request(response[0], 100, response[1]))
        return len(data)

    def flush(self):
        pass

    def read(self, size):
        data = bytes(self.buffer[:size])
        del self.buffer[:size]
        return data


class UpdateTests(unittest.TestCase):
    def test_image_vectors_and_partition(self):
        image = struct.pack('<II', 0x20005000, 0x08004009) + bytes(24)
        self.assertEqual(update.validate_image(image), (0x20005000, 0x08004009))
        invalid = [b'', image + bytes(0x1BC00),
                   struct.pack('<II', 0x20005001, 0x08004009) + bytes(24),
                   struct.pack('<II', 0x20005000, 0x08000001) + bytes(24),
                   struct.pack('<II', 0x20005000, 0x08004008) + bytes(24),
                   struct.pack('<II', 0x20005000, 0x08004101) + bytes(24)]
        for value in invalid:
            with self.subTest(image=value[:8]):
                with self.assertRaises(ValueError):
                    update.validate_image(value)

    def test_stop_ack_before_boot_and_same_stop_token(self):
        port = FakePort()
        update.request_rtos_boot(port, timeout=.02, stop_sequence=65535)
        self.assertEqual([(f.message_type, f.sequence, f.payload) for f in port.writes],
                         [(0x12, 65535, b''), (0x13, 0, b'\xff\xff')])

    def test_stop_nack_must_not_send_boot(self):
        port = FakePort(lambda f: (0xF1, struct.pack('<HB', f.sequence, 6)))
        with self.assertRaisesRegex(RuntimeError, 'NACK 6'):
            update.request_rtos_boot(port, timeout=.02, stop_sequence=10)
        self.assertEqual(len(port.writes), 1)

    def test_unrelated_ack_must_not_authorize_boot(self):
        port = FakePort(lambda f: (0xF0, struct.pack('<HB', f.sequence + 1, 0)))
        with self.assertRaises(TimeoutError):
            update.request_rtos_boot(port, timeout=.01, stop_sequence=10)
        self.assertEqual(len(port.writes), 1)

    def test_boot_rejection_is_not_retried_or_bypassed(self):
        def reply(frame):
            code = 0 if frame.message_type == 0x12 else 4
            return (0xF0 if code == 0 else 0xF1, struct.pack('<HB', frame.sequence, code))
        port = FakePort(reply)
        with self.assertRaisesRegex(RuntimeError, 'NACK 4'):
            update.request_rtos_boot(port, timeout=.02, stop_sequence=10)
        self.assertEqual(len(port.writes), 2)


if __name__ == '__main__':
    unittest.main()

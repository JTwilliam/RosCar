import math
import struct

import pytest

from icm20948_ros2.sensor import GRAVITY, ICM20948, decode_sample


class FakeBus:
    def __init__(self, identity=0xEA):
        self.bank = 0
        self.registers = {(0, 0): identity, (0, 0x1A): 1}
        self.writes = []
        self.closed = False
        self.data = list(struct.pack('>7h', 0, 0, 8192, 0, 0, 655, 0))

    def write_byte_data(self, address, register, value):
        self.writes.append((self.bank, register, value))
        if register == 0x7F:
            self.bank = value >> 4
        else:
            self.registers[self.bank, register] = value

    def read_byte_data(self, address, register):
        return self.registers.get((self.bank, register), 0)

    def read_i2c_block_data(self, address, register, length):
        assert self.bank == 0 and register == 0x2D and length == 14
        return self.data

    def close(self):
        self.closed = True


def test_signed_units_and_temperature():
    sample = decode_sample(struct.pack('>7h', -8192, 4096, 8192, -655, 655, 0, 3339))
    assert sample.acceleration == pytest.approx((-GRAVITY, GRAVITY / 2, GRAVITY))
    assert sample.angular_velocity == pytest.approx((-math.radians(10), math.radians(10), 0))
    assert sample.temperature == pytest.approx(31.0009, abs=0.001)


def test_initialization_bank_scale_and_fresh_data():
    bus = FakeBus()
    sensor = ICM20948(bus=bus, sleep=lambda _: None)
    sensor.initialize()
    assert sensor.hardware_rate >= 100
    assert bus.bank == 0
    assert bus.registers[2, 0x01] == 0x1B
    assert bus.registers[2, 0x14] == 0x2B
    assert sensor.read().acceleration[2] == pytest.approx(GRAVITY)
    bus.registers[0, 0x1A] = 0
    assert sensor.read() is None
    sensor.close()
    assert bus.closed


def test_wrong_device_is_rejected_before_reset():
    bus = FakeBus(identity=0x68)
    with pytest.raises(RuntimeError, match='WHO_AM_I'):
        ICM20948(bus=bus, sleep=lambda _: None).initialize()
    assert (0, 0x06, 0x80) not in bus.writes


def test_corrupt_and_saturated_reads_are_rejected():
    with pytest.raises(ValueError, match='14'):
        decode_sample([0] * 13)
    with pytest.raises(ValueError, match='saturated'):
        decode_sample(struct.pack('>7h', 32767, 0, 0, 0, 0, 0, 0))

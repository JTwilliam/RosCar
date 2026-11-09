"""Linux SMBus port of the register operations in ICM20948_WE.

See LICENSE for attribution. Vectors use the accelerometer/gyro sensor axes.
"""

from dataclasses import dataclass
import math
import struct
import time


GRAVITY = 9.80665


@dataclass(frozen=True)
class Sample:
    acceleration: tuple
    angular_velocity: tuple
    temperature: float


def decode_sample(data):
    """Decode a coherent bank-0 accel/gyro/temp burst; +/-4g, +/-500 deg/s."""
    if len(data) != 14:
        raise ValueError(f'Expected 14 sensor bytes, got {len(data)}')
    values = struct.unpack('>7h', bytes(data))
    if any(abs(x) >= 32760 for x in values[:6]):
        raise ValueError('Accelerometer or gyro saturated; sample discarded')
    return Sample(
        tuple(x * GRAVITY / 8192.0 for x in values[:3]),
        tuple(math.radians(x / 65.5) for x in values[3:6]),
        values[6] / 333.87 + 21.0,
    )


class ICM20948:
    def __init__(self, bus_number=1, address=0x68, sample_rate=100.0,
                 bus=None, sleep=time.sleep, accel_dlpf=5, gyro_dlpf=3):
        if address not in (0x68, 0x69):
            raise ValueError('ICM-20948 address must be 0x68 (104) or 0x69 (105)')
        if not 10.0 <= sample_rate <= 225.0:
            raise ValueError('sample_rate must be between 10 and 225 Hz')
        if accel_dlpf not in range(8) or gyro_dlpf not in range(8):
            raise ValueError('DLPF indices must be in 0..7')
        self.accel_config = (accel_dlpf << 3) | 0x03
        self.gyro_config = (gyro_dlpf << 3) | 0x03
        self.address = address
        self.sample_rate = sample_rate
        self._sleep = sleep
        if bus is None:
            try:
                from smbus import SMBus
            except ImportError:
                from smbus2 import SMBus
            bus = SMBus(bus_number)
        self.bus = bus

    def _bank(self, number):
        self.bus.write_byte_data(self.address, 0x7F, number << 4)

    def _write(self, register, value):
        self.bus.write_byte_data(self.address, register, value)

    def initialize(self):
        self._bank(0)
        identity = self.bus.read_byte_data(self.address, 0x00)
        if identity != 0xEA:
            raise RuntimeError(f'WHO_AM_I=0x{identity:02x}, expected ICM-20948 0xea')
        self._write(0x06, 0x80)  # DEVICE_RESET
        self._sleep(0.1)
        self._bank(0)
        self._write(0x06, 0x01)  # Wake; auto-select best clock
        self._write(0x07, 0x00)  # Enable all accelerometer/gyro axes
        self._write(0x05, 0x00)  # Continuous, not low-power cycling
        self._write(0x03, 0x00)  # Disable FIFO, DMP and auxiliary I2C master
        self._write(0x0F, 0x00)  # No bypass; magnetometer intentionally unused
        self._write(0x11, 0x01)  # RAW_DATA_0_RDY_EN
        self._bank(2)
        divider = max(0, int(1125.0 / self.sample_rate) - 1)
        self.hardware_rate = 1125.0 / (divider + 1)
        self._write(0x00, divider)  # GYRO_SMPLRT_DIV
        self._write(0x01, self.gyro_config)  # +/-500 dps; default ~51 Hz DLPF
        self._write(0x10, divider >> 8)
        self._write(0x11, divider & 0xFF)
        self._write(0x14, self.accel_config)  # +/-4g; default ~11.5 Hz DLPF
        self._write(0x09, 0x01)  # ODR_ALIGN_EN
        if (self.bus.read_byte_data(self.address, 0x01) != self.gyro_config or
                self.bus.read_byte_data(self.address, 0x14) != self.accel_config):
            raise RuntimeError('IMU configuration readback failed')
        self._bank(0)
        self._sleep(0.2)

    def read(self):
        # Reading the status clears data-ready. Do not republish a stale sample.
        if not self.bus.read_byte_data(self.address, 0x1A) & 0x01:
            return None
        data = self.bus.read_i2c_block_data(self.address, 0x2D, 14)
        return decode_sample(data)

    def close(self):
        self.bus.close()

"""Six-axis attitude filtering inspired by the supplied bsp_imu.c.

Quaternion convention: (x,y,z,w), rotation from sensor body into a Z-up world.
World yaw is relative to startup and is not an absolute ENU compass heading.
"""

import math
import statistics

from .sensor import GRAVITY


def norm(vector):
    return math.sqrt(sum(x * x for x in vector))


def euler_quaternion(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


class StationaryCalibration:
    """Reject moving windows rather than silently calibrating out a rotation.

Slow perfectly steady rotation cannot be distinguished from bias with this
sensor alone. The operator must keep the robot stationary during calibration.
"""
    def __init__(self, count, gyro_std_limit=0.015, accel_std_limit=0.15,
                 gyro_mean_limit=0.10):
        if count < 2:
            raise ValueError('Calibration requires at least two samples')
        self.count = count
        self.gyro_std_limit = gyro_std_limit
        self.accel_std_limit = accel_std_limit
        self.gyro_mean_limit = gyro_mean_limit
        self.samples = []

    def add(self, acceleration, angular_velocity):
        self.samples.append((acceleration, angular_velocity))
        if len(self.samples) < self.count:
            return None
        acc_axes = list(zip(*(x[0] for x in self.samples)))
        gyr_axes = list(zip(*(x[1] for x in self.samples)))
        mean_acc = tuple(statistics.mean(x) for x in acc_axes)
        bias = tuple(statistics.mean(x) for x in gyr_axes)
        stationary = (
            abs(norm(mean_acc) - GRAVITY) < 0.15 * GRAVITY and
            norm(bias) < self.gyro_mean_limit and
            max(statistics.pstdev(x) for x in gyr_axes) < self.gyro_std_limit and
            max(statistics.pstdev(x) for x in acc_axes) < self.accel_std_limit
        )
        self.samples.clear()
        if not stationary:
            raise ValueError('Motion or abnormal acceleration during calibration')
        return bias


class Mahony6D:
    def __init__(self, gain=1.0, accel_tolerance=0.15, max_dt=0.2,
                 integral_gain=0.0001, integral_limit=2.0, reference_dt=0.02):
        self.gain = gain
        self.accel_tolerance = accel_tolerance
        self.max_dt = max_dt
        self.integral_gain = integral_gain
        self.integral_limit = integral_limit
        self.reference_dt = reference_dt
        self.integral = [0.0, 0.0]
        self.q = None

    def update(self, acceleration, angular_velocity, dt):
        magnitude = norm(acceleration)
        gravity_usable = abs(magnitude - GRAVITY) < self.accel_tolerance * GRAVITY
        if self.q is None:
            if not gravity_usable:
                return None
            ax, ay, az = acceleration
            self.q = euler_quaternion(
                math.atan2(ay, az), math.atan2(-ax, math.hypot(ay, az)), 0.0)
            return self.q
        # Do not integrate across missing data; the intervening motion is unknown.
        if not 0 < dt <= self.max_dt:
            self.integral = [0.0, 0.0]
            return None
        x, y, z, w = self.q
        gx, gy, gz = angular_velocity
        if gravity_usable:
            ax, ay, az = (v / magnitude for v in acceleration)
            vx = 2.0 * (x * z - w * y)
            vy = 2.0 * (y * z + w * x)
            vz = 1.0 - 2.0 * (x * x + y * y)
            error = (ay * vz - az * vy, az * vx - ax * vz)
            for axis in range(2):
                # Reference code accumulates per sample. Scale to its nominal
                # 20ms period so gain/limit retain their meaning at 100 Hz.
                value = self.integral[axis] + error[axis] * dt / self.reference_dt
                self.integral[axis] = max(-self.integral_limit, min(self.integral_limit, value))
            gx += self.gain * error[0] + self.integral_gain * self.integral[0]
            gy += self.gain * error[1] + self.integral_gain * self.integral[1]
            # As in bsp_imu.c, gravity feedback does not modify body Z gyro.
            # Absolute world yaw remains unobservable without another reference.
        # First-order body-rate quaternion integration, normalize every sample.
        updated = (
            x + 0.5 * dt * (w * gx + y * gz - z * gy),
            y + 0.5 * dt * (w * gy + z * gx - x * gz),
            z + 0.5 * dt * (w * gz + x * gy - y * gx),
            w - 0.5 * dt * (x * gx + y * gy + z * gz),
        )
        length = norm(updated)
        self.q = tuple(v / length for v in updated)
        return self.q


class MeasurementFilter:
    """Time-adjusted EMA, with optional pre-filter Z deadband from bsp_imu.c."""
    def __init__(self, gyro_alpha=0.8, accel_alpha=0.7,
                 reference_dt=0.02, gyro_z_deadband=0.0):
        if not all(math.isfinite(a) and 0 <= a < 1 for a in (gyro_alpha, accel_alpha)):
            raise ValueError('LPF alpha values must be finite and in [0, 1)')
        if not math.isfinite(reference_dt) or reference_dt <= 0:
            raise ValueError('LPF reference_dt must be finite and positive')
        if not math.isfinite(gyro_z_deadband) or gyro_z_deadband < 0:
            raise ValueError('gyro_z_deadband must be finite and nonnegative')
        self.gyro_alpha = gyro_alpha
        self.accel_alpha = accel_alpha
        self.reference_dt = reference_dt
        self.deadband = gyro_z_deadband
        self.acceleration = None
        self.gyro = None

    def update(self, acceleration, gyro, dt):
        gyro = tuple(gyro)
        if abs(gyro[2]) < self.deadband:
            gyro = (gyro[0], gyro[1], 0.0)
        if self.acceleration is None or not 0 < dt <= 0.2:
            self.acceleration, self.gyro = tuple(acceleration), gyro
        else:
            aa = self.accel_alpha ** (dt / self.reference_dt)
            ga = self.gyro_alpha ** (dt / self.reference_dt)
            self.acceleration = tuple(aa * p + (1 - aa) * v
                                      for p, v in zip(self.acceleration, acceleration))
            self.gyro = tuple(ga * p + (1 - ga) * v for p, v in zip(self.gyro, gyro))
        return self.acceleration, self.gyro

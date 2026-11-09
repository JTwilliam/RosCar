"""ROS 2 ICM-20948 node; never publish fabricated data after a bus failure."""

import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, Temperature

from .fusion import Mahony6D, MeasurementFilter, StationaryCalibration, norm
from .sensor import ICM20948


def diagonal(variance):
    return [variance, 0.0, 0.0, 0.0, variance, 0.0, 0.0, 0.0, variance]


def vector_parameter(node, name):
    values = tuple(float(x) for x in node.get_parameter(name).value)
    if len(values) != 3 or not all(math.isfinite(x) for x in values):
        raise ValueError(f'{name} must contain three finite SI-unit values')
    return values


class ImuNode(Node):
    def __init__(self):
        super().__init__('icm20948')
        self.sensor = None
        defaults = {
            'bus': 1, 'address': 104, 'frame_id': 'imu_link',
            'sample_rate': 100.0, 'calibration_seconds': 3.0,
            'calibration_timeout': 30.0, 'calibrate_on_start': True,
            'gyro_bias': [0.0, 0.0, 0.0],
            'accel_bias': [0.0, 0.0, 0.0],
            'gyro_stddev': 0.01, 'accel_stddev': 0.15,
            'tilt_stddev': 0.05, 'fusion_gain': 0.8,
            'fusion_integral_gain': 0.0001, 'fusion_integral_limit': 2.0,
            'gyro_lpf_alpha': 0.8, 'accel_lpf_alpha': 0.7,
            'lpf_reference_dt': 0.02, 'gyro_z_deadband': 0.0,
            'accel_dlpf': 5, 'gyro_dlpf': 3,
            'max_consecutive_errors': 20,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.rate = float(self.get_parameter('sample_rate').value)
        if not math.isfinite(self.rate) or not 10 <= self.rate <= 225:
            raise ValueError('sample_rate must be between 10 and 225 Hz')
        self.frame = self.get_parameter('frame_id').value
        if not self.frame or self.frame.startswith('/'):
            raise ValueError('frame_id must be nonempty without a leading slash')
        self.bias = vector_parameter(self, 'gyro_bias')
        self.accel_bias = vector_parameter(self, 'accel_bias')
        self.gyro_variance = self._positive('gyro_stddev') ** 2
        self.accel_variance = self._positive('accel_stddev') ** 2
        self.tilt_variance = self._positive('tilt_stddev') ** 2
        reference_dt = self._positive('lpf_reference_dt')
        self.filter = Mahony6D(
            gain=self._positive('fusion_gain'),
            integral_gain=self._nonnegative('fusion_integral_gain'),
            integral_limit=self._positive('fusion_integral_limit'),
            reference_dt=reference_dt)
        self.measurement_filter = MeasurementFilter(
            gyro_alpha=float(self.get_parameter('gyro_lpf_alpha').value),
            accel_alpha=float(self.get_parameter('accel_lpf_alpha').value),
            reference_dt=reference_dt,
            gyro_z_deadband=self._nonnegative('gyro_z_deadband'))
        seconds = self._positive('calibration_seconds')
        self.calibration_timeout = self._positive('calibration_timeout')
        if self.calibration_timeout <= seconds:
            raise ValueError('calibration_timeout must exceed calibration_seconds')
        self.error_limit = int(self.get_parameter('max_consecutive_errors').value)
        if self.error_limit < 1:
            raise ValueError('max_consecutive_errors must be positive')
        self.calibration = None
        if self.get_parameter('calibrate_on_start').value:
            self.calibration = StationaryCalibration(max(2, round(seconds * self.rate)))
        self.raw_pub = self.create_publisher(Imu, 'imu/data_raw', qos_profile_sensor_data)
        self.fused_pub = self.create_publisher(Imu, 'imu/data', qos_profile_sensor_data)
        self.temp_pub = self.create_publisher(Temperature, 'imu/temperature', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.samples = 0
        self.total_errors = 0
        self.consecutive_errors = 0
        self.last_sample = None
        self.last_step = None
        self.last_accel_norm = float('nan')
        self.last_error = ''
        self.previous_diag_count = 0
        try:
            self.sensor = ICM20948(
                bus_number=int(self.get_parameter('bus').value),
                address=int(self.get_parameter('address').value), sample_rate=self.rate,
                accel_dlpf=int(self.get_parameter('accel_dlpf').value),
                gyro_dlpf=int(self.get_parameter('gyro_dlpf').value))
            self.sensor.initialize()
        except Exception:
            if self.sensor is not None:
                self.sensor.close()
            self.destroy_node()
            raise
        self.started = time.monotonic()
        self.previous_diag_time = self.started
        self.get_logger().info(
            f'ICM-20948 connected on /dev/i2c-{self.get_parameter("bus").value}, '
            f'address=0x{self.get_parameter("address").value:02x}; '
            f'poll={self.rate:.1f} Hz, sensor ODR={self.sensor.hardware_rate:.1f} Hz')
        if self.calibration:
            self.get_logger().warn(
                f'Keep the IMU stationary for at least {seconds:.1f}s. '
                'IMU messages start only after gyro calibration succeeds.')
        self.get_logger().info(
            'Six-axis fusion: gravity retained, startup-relative yaw (may drift), no TF published.')
        self.timer = self.create_timer(1.0 / self.rate, self._tick)
        self.diag_timer = self.create_timer(1.0, self._diagnostics)

    def _positive(self, name):
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f'{name} must be finite and positive')
        return value

    def _nonnegative(self, name):
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value < 0:
            raise ValueError(f'{name} must be finite and nonnegative')
        return value

    def _tick(self):
        now = time.monotonic()
        if self.calibration and now - self.started > self.calibration_timeout:
            raise RuntimeError('Gyro calibration timed out; keep robot stationary and restart')
        if now - (self.last_sample or self.started) > 2.0:
            raise RuntimeError('No fresh ICM-20948 samples for 2 seconds; check bus and restart')
        try:
            sample = self.sensor.read()
        except (OSError, ValueError) as exc:
            self.total_errors += 1
            self.consecutive_errors += 1
            self.last_error = str(exc)
            if self.consecutive_errors == 1:
                self.get_logger().error(f'IMU read failed: {exc}')
            if self.consecutive_errors >= self.error_limit:
                raise RuntimeError('Repeated IMU read failures; shutting down') from exc
            return
        if sample is None:
            return
        self.last_sample = now
        self.consecutive_errors = 0
        self.last_error = ''
        acceleration = tuple(a - b for a, b in zip(sample.acceleration, self.accel_bias))
        self.last_accel_norm = norm(acceleration)
        if self.calibration:
            try:
                bias = self.calibration.add(acceleration, sample.angular_velocity)
            except ValueError as exc:
                self.get_logger().warn(f'{exc}; restarting stationary calibration window')
                return
            if bias is None:
                return
            self.bias = bias
            self.calibration = None
            self.get_logger().info(
                f'Calibration complete. Gyro bias [rad/s]: {self.bias}. Publishing IMU.')
        gyro = tuple(a - b for a, b in zip(sample.angular_velocity, self.bias))
        dt = 1.0 / self.rate if self.last_step is None else now - self.last_step
        self.last_step = now
        stamp = self.get_clock().now().to_msg()
        raw = Imu()
        raw.header.stamp = stamp
        raw.header.frame_id = self.frame
        raw.orientation_covariance[0] = -1.0  # REP-145: orientation unavailable
        raw.angular_velocity.x, raw.angular_velocity.y, raw.angular_velocity.z = gyro
        raw.linear_acceleration.x, raw.linear_acceleration.y, raw.linear_acceleration.z = acceleration
        raw.angular_velocity_covariance = diagonal(self.gyro_variance)
        raw.linear_acceleration_covariance = diagonal(self.accel_variance)
        self.raw_pub.publish(raw)
        filtered_acc, filtered_gyro = self.measurement_filter.update(acceleration, gyro, dt)
        q = self.filter.update(filtered_acc, filtered_gyro, dt)
        if q is not None:
            fused = Imu()
            fused.header = raw.header
            (fused.angular_velocity.x, fused.angular_velocity.y,
             fused.angular_velocity.z) = filtered_gyro
            (fused.linear_acceleration.x, fused.linear_acceleration.y,
             fused.linear_acceleration.z) = filtered_acc
            fused.angular_velocity_covariance = raw.angular_velocity_covariance
            fused.linear_acceleration_covariance = raw.linear_acceleration_covariance
            fused.orientation.x, fused.orientation.y, fused.orientation.z, fused.orientation.w = q
            fused.orientation_covariance = [
                self.tilt_variance, 0.0, 0.0, 0.0, self.tilt_variance, 0.0,
                0.0, 0.0, 1.0e6,  # No absolute yaw observation; do not fuse as heading
            ]
            self.fused_pub.publish(fused)
        else:
            self.get_logger().warn(
                'Attitude output skipped: timing gap or unsuitable initialization acceleration',
                throttle_duration_sec=5.0)
        # Temperature does not need to consume 100 Hz of network bandwidth.
        if self.samples % max(1, round(self.rate)) == 0:
            temp = Temperature()
            temp.header = raw.header
            temp.temperature = sample.temperature
            temp.variance = 0.0  # Unknown
            self.temp_pub.publish(temp)
        self.samples += 1

    def _diagnostics(self):
        now = time.monotonic()
        status = DiagnosticStatus()
        status.name = 'icm20948/acquisition'
        status.hardware_id = f'i2c-{self.get_parameter("bus").value}:0x{self.sensor.address:02x}'
        status.level = DiagnosticStatus.OK
        status.message = 'Streaming; yaw is relative and unobservable from gravity'
        if self.calibration:
            status.level, status.message = DiagnosticStatus.WARN, 'Calibrating: keep stationary'
        if self.consecutive_errors or now - (self.last_sample or self.started) > 0.5:
            status.level, status.message = DiagnosticStatus.ERROR, 'Sensor data missing or read failure'
        rate = (self.samples - self.previous_diag_count) / (now - self.previous_diag_time)
        status.values = [KeyValue(key=k, value=str(v)) for k, v in {
            'published_raw_hz': round(rate, 2), 'published_raw_samples': self.samples,
            'read_errors': self.total_errors, 'last_error': self.last_error,
            'acceleration_norm_m_s2': self.last_accel_norm,
            'gyro_bias_rad_s': self.bias,
        }.items()]
        output = DiagnosticArray()
        output.header.stamp = self.get_clock().now().to_msg()
        output.status = [status]
        self.diag_pub.publish(output)
        self.previous_diag_count, self.previous_diag_time = self.samples, now

    def close(self):
        if self.sensor is not None:
            self.sensor.close()
            self.sensor = None


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ImuNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        rclpy.logging.get_logger('icm20948').fatal(str(exc))
        raise
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

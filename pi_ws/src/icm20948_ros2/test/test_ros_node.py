"""Exercise real ROS publications with a simulated register-independent sensor."""

import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from icm20948_ros2 import node as driver
from icm20948_ros2.sensor import GRAVITY, Sample


class SimulatedSensor:
    closed = False

    def __init__(self, **kwargs):
        self.address = kwargs['address']
        self.hardware_rate = 102.27
        self.sample = Sample((0, 0, GRAVITY), (.01, -.02, .03), 25.0)

    def initialize(self):
        pass

    def read(self):
        return self.sample

    def close(self):
        self.closed = True


def test_calibrated_raw_and_fused_messages(monkeypatch):
    monkeypatch.setattr(driver, 'ICM20948', SimulatedSensor)
    rclpy.init(args=['--ros-args', '-p', 'calibration_seconds:=0.1'])
    imu = None
    listener = None
    executor = SingleThreadedExecutor()
    raw, fused = [], []
    try:
        imu = driver.ImuNode()
        listener = rclpy.create_node('imu_test_listener')
        listener.create_subscription(Imu, 'imu/data_raw', raw.append, qos_profile_sensor_data)
        listener.create_subscription(Imu, 'imu/data', fused.append, qos_profile_sensor_data)
        executor.add_node(imu)
        executor.add_node(listener)
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline and len(fused) < 20:
            executor.spin_once(timeout_sec=.02)
        assert len(raw) >= 20 and len(fused) >= 20
        assert imu.bias == pytest.approx((.01, -.02, .03))
        assert raw[-1].orientation_covariance[0] == -1
        assert raw[-1].angular_velocity.z == pytest.approx(0, abs=1e-12)
        assert fused[-1].header.frame_id == 'imu_link'
        assert fused[-1].orientation.w == pytest.approx(1)
        assert fused[-1].orientation_covariance[8] == 1e6
        assert fused[-1].linear_acceleration.z == pytest.approx(GRAVITY)
        # A step must appear in raw measurements before the low-pass output.
        imu.sensor.sample = Sample((0, 0, GRAVITY), (.01, -.02, .13), 25.0)
        deadline = time.monotonic() + 1
        while time.monotonic() < deadline and fused[-1].angular_velocity.z <= 0:
            executor.spin_once(timeout_sec=.01)
        assert raw[-1].angular_velocity.z == pytest.approx(.1)
        assert 0 < fused[-1].angular_velocity.z < .1
        # A transient bus error must not increment the published-sample count.
        count = imu.samples
        monkeypatch.setattr(imu.sensor, 'read', lambda: (_ for _ in ()).throw(OSError('test bus error')))
        imu._tick()
        assert imu.samples == count and imu.total_errors == 1
    finally:
        executor.shutdown()
        if imu is not None:
            sensor = imu.sensor
            imu.close()
            assert sensor.closed
            imu.destroy_node()
        if listener is not None:
            listener.destroy_node()
        rclpy.shutdown()


def test_viewer_receives_imu_and_publishes_markers():
    from icm20948_ros2.viewer import ImuViewer
    from visualization_msgs.msg import MarkerArray

    rclpy.init()
    viewer = ImuViewer()
    tester = rclpy.create_node('imu_visualization_test')
    executor = SingleThreadedExecutor()
    results = []
    publisher = tester.create_publisher(Imu, '/imu/data', qos_profile_sensor_data)
    tester.create_subscription(MarkerArray, '/imu/markers', results.append, 10)
    message = Imu()
    message.header.frame_id = 'imu_link'
    message.orientation.z = 2 ** -.5
    message.orientation.w = 2 ** -.5
    timer = tester.create_timer(.01, lambda: publisher.publish(message))
    executor.add_node(viewer)
    executor.add_node(tester)
    try:
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline and not results:
            executor.spin_once(timeout_sec=.02)
        assert results and len(results[-1].markers) == 4
        assert results[-1].markers[0].header.frame_id == 'imu_view'
        assert results[-1].markers[0].pose.orientation.z == pytest.approx(2 ** -.5)
    finally:
        timer.cancel()
        executor.shutdown()
        viewer.destroy_node()
        tester.destroy_node()
        rclpy.shutdown()

"""Check the bridge's real ROS outputs with simulated serial frames."""

import math
import struct
import time

import pytest
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from tf2_msgs.msg import TFMessage

from learm_bridge.protocol import Frame, MSG_CHASSIS_STATE
from learm_bridge.serial_bridge_node import LeArmSerialBridge


class SimulatedSerial:
    is_open = True
    in_waiting = 1

    def read(self, size):
        return b"test"

    def close(self):
        self.is_open = False


def chassis_frame(timestamp_ms, encoders):
    return Frame(
        version=1, message_type=MSG_CHASSIS_STATE, flags=0, sequence=0,
        timestamp_ms=timestamp_ms,
        payload=struct.pack("<I4i4bHHBBH", timestamp_ms, *encoders,
                            0, 0, 0, 0, 12000, 0, 0, 0, 0),
    )


def test_batched_encoder_samples_keep_firmware_dt_and_disable_raw_tf(monkeypatch):
    rclpy.init(args=["--ros-args", "-p", "publish_tf:=false"])
    bridge = LeArmSerialBridge()
    bridge._poll_timer.cancel()
    bridge._diagnostic_timer.cancel()
    bridge._serial = SimulatedSerial()
    listener = rclpy.create_node("learm_ekf_input_test")
    executor = SingleThreadedExecutor()
    odometry, transforms = [], []
    listener.create_subscription(Odometry, "/odom", odometry.append, 10)
    listener.create_subscription(TFMessage, "/tf", transforms.append, 10)
    executor.add_node(bridge)
    executor.add_node(listener)
    # Forward, left and counterclockwise updates in one serial read.
    frames = [chassis_frame(1000, (0, 0, 0, 0)),
              chassis_frame(1100, (100, -100, -100, 100)),
              chassis_frame(1200, (200, 0, 0, 200)),
              chassis_frame(1300, (300, 100, -100, 100))]
    monkeypatch.setattr(bridge._decoder, "feed", lambda data: frames)
    try:
        deadline = time.monotonic() + 3.0
        while bridge._odom_publisher.get_subscription_count() == 0:
            executor.spin_once(timeout_sec=0.02)
            assert time.monotonic() < deadline, "ROS discovery timed out"
        bridge._poll_serial()
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and len(odometry) < 4:
            executor.spin_once(timeout_sec=0.02)
        assert len(odometry) == 4
        distance = 100 * bridge._odometry.meters_per_tick
        assert odometry[1].twist.twist.linear.x == pytest.approx(distance / 0.1)
        assert odometry[2].twist.twist.linear.y == pytest.approx(distance / 0.1)
        expected_wz = 4 * distance / (2 * (0.190 + 0.205)) / 0.1
        assert odometry[3].twist.twist.angular.z == pytest.approx(expected_wz)
        for msg in odometry:
            assert msg.header.frame_id == "odom"
            assert msg.child_frame_id == "base_link"
            assert msg.header.stamp.sec > 0
            assert msg.twist.covariance[0] == pytest.approx(0.0025)
            assert msg.twist.covariance[7] == pytest.approx(0.01)
            assert msg.twist.covariance[35] == pytest.approx(0.04)
            assert all(math.isfinite(value) for value in msg.twist.covariance)
        assert not transforms
        # Firmware restart rebaselines the encoders rather than inventing speed.
        frames[:] = [chassis_frame(10, (0, 0, 0, 0))]
        bridge._poll_serial()
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and len(odometry) < 5:
            executor.spin_once(timeout_sec=0.02)
        assert len(odometry) == 5
        assert odometry[-1].twist.twist.linear.x == 0.0
        assert odometry[-1].twist.twist.linear.y == 0.0
        assert odometry[-1].twist.twist.angular.z == 0.0
    finally:
        executor.shutdown()
        bridge.destroy_node()
        listener.destroy_node()
        rclpy.shutdown()

"""ROS 2 node that owns the STM32 serial port and publishes telemetry."""

from __future__ import annotations

import math
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from tf2_ros import TransformBroadcaster

try:
    import serial
except ImportError:  # pragma: no cover - reported clearly on the target host
    serial = None

from .diagnostic_mapping import snapshot_to_values
from .odometry import MecanumOdometry
from .protocol import MSG_CHASSIS_STATE, ProtocolDecoder, decode_chassis_state
from .telemetry import ChassisTelemetryTracker


DEFAULT_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"


class LeArmSerialBridge(Node):
    def __init__(self) -> None:
        super().__init__("learm_serial_bridge")

        self.declare_parameter("port", DEFAULT_PORT)
        self.declare_parameter("baud", 115200)
        self.declare_parameter("read_chunk_size", 512)
        self.declare_parameter("diagnostics_period_s", 0.2)
        self.declare_parameter("reconnect_period_s", 2.0)
        self.declare_parameter("wheel_radius_m", 0.048)
        self.declare_parameter("wheel_base_m", 0.190)
        self.declare_parameter("track_width_m", 0.205)
        self.declare_parameter("encoder_counts_per_revolution", 5740.25304043815)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", True)
        # Variances of [vx, vy, vz, wx, wy, wz], not standard deviations.
        # Initial estimates only: mecanum lateral slip is usually larger.
        self.declare_parameter(
            "twist_covariance_diagonal", [0.0025, 0.01, 1e6, 1e6, 1e6, 0.04]
        )

        self._port = str(self.get_parameter("port").value)
        self._baud = int(self.get_parameter("baud").value)
        self._read_chunk_size = max(1, int(self.get_parameter("read_chunk_size").value))
        self._reconnect_period_s = max(
            0.2, float(self.get_parameter("reconnect_period_s").value)
        )
        diagnostics_period_s = max(
            0.1, float(self.get_parameter("diagnostics_period_s").value)
        )

        self._decoder = ProtocolDecoder()
        self._tracker = ChassisTelemetryTracker()
        self._serial = None
        self._next_reconnect_at = 0.0
        self._last_serial_error = ""
        self._last_state = None
        self._last_firmware_timestamp_ms: int | None = None
        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._publish_tf = bool(self.get_parameter("publish_tf").value)
        variances = list(self.get_parameter("twist_covariance_diagonal").value)
        if len(variances) != 6 or any(
            not math.isfinite(value) or value <= 0.0 for value in variances
        ):
            raise ValueError("twist_covariance_diagonal needs six finite positive variances")
        self._twist_covariance = [0.0] * 36
        for index, variance in enumerate(variances):
            self._twist_covariance[index * 7] = variance
        self._odometry = MecanumOdometry(
            wheel_radius_m=float(self.get_parameter("wheel_radius_m").value),
            wheel_base_m=float(self.get_parameter("wheel_base_m").value),
            track_width_m=float(self.get_parameter("track_width_m").value),
            encoder_counts_per_revolution=float(
                self.get_parameter("encoder_counts_per_revolution").value
            ),
        )

        self._encoder_publisher = self.create_publisher(
            Int32MultiArray, "/learm/wheel_encoders", 10
        )
        self._diagnostic_publisher = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10
        )
        self._odom_publisher = self.create_publisher(Odometry, "/odom", 10)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._poll_timer = self.create_timer(0.005, self._poll_serial)
        self._diagnostic_timer = self.create_timer(
            diagnostics_period_s, self._publish_diagnostics
        )

        if serial is None:
            self._last_serial_error = "python3-serial is not installed"
            self.get_logger().error(self._last_serial_error)
        else:
            self.get_logger().info(
                f"LeArm serial bridge configured for {self._port} at {self._baud} 8N1"
            )

    def _ensure_serial_open(self, now: float) -> bool:
        if self._serial is not None and self._serial.is_open:
            return True
        if serial is None or now < self._next_reconnect_at:
            return False

        try:
            self._serial = serial.Serial(
                port=self._port,
                baudrate=self._baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0,
                write_timeout=0,
            )
            self._last_serial_error = ""
            self.get_logger().info(f"Opened serial port {self._port}")
            return True
        except (OSError, serial.SerialException) as error:
            message = str(error)
            if message != self._last_serial_error:
                self.get_logger().error(f"Cannot open {self._port}: {message}")
                self._last_serial_error = message
            self._serial = None
            self._next_reconnect_at = now + self._reconnect_period_s
            return False

    def _close_serial_after_error(self, error: Exception) -> None:
        message = str(error)
        if message != self._last_serial_error:
            self.get_logger().error(f"Serial read failed: {message}")
            self._last_serial_error = message
        if self._serial is not None:
            try:
                self._serial.close()
            except (OSError, serial.SerialException):
                pass
        self._serial = None
        self._next_reconnect_at = time.monotonic() + self._reconnect_period_s

    def _poll_serial(self) -> None:
        now = time.monotonic()
        if not self._ensure_serial_open(now):
            return

        try:
            waiting = self._serial.in_waiting
            if waiting <= 0:
                return
            data = self._serial.read(min(waiting, self._read_chunk_size))
        except (OSError, serial.SerialException) as error:
            self._close_serial_after_error(error)
            return

        for frame in self._decoder.feed(data):
            if frame.message_type != MSG_CHASSIS_STATE:
                continue
            try:
                state = decode_chassis_state(frame)
            except ValueError:
                self._decoder.stats.format_errors += 1
                self._decoder.stats.frames_dropped += 1
                continue

            received_at = time.monotonic()
            # STM32 单调时钟回退表示下位机重启，只重建编码器基线，避免计数清零
            # 被误判成一次巨大的底盘位移。
            if (
                self._last_firmware_timestamp_ms is not None
                and state.timestamp_ms < self._last_firmware_timestamp_ms
            ):
                self._odometry.reset(state.encoders, state.timestamp_ms * 0.001)
            self._last_firmware_timestamp_ms = state.timestamp_ms
            self._last_state = state
            self._tracker.update(state, received_at)
            # Several firmware samples can arrive in one USB read. Their receive
            # times are almost identical; use the STM32 sampling clock for dt.
            # ROS message stamps still use the host's ROS clock below.
            self._publish_odometry(state.encoders, state.timestamp_ms * 0.001)

            message = Int32MultiArray()
            message.data = list(state.encoders)
            self._encoder_publisher.publish(message)

    def _publish_odometry(
        self,
        encoders: tuple[int, int, int, int],
        sample_time_s: float,
    ) -> None:
        """将 STM32 累计编码器值转为标准 ROS 2 里程计并发布 TF。"""

        estimate = self._odometry.update(encoders, sample_time_s)
        stamp = self.get_clock().now().to_msg()

        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = self._odom_frame
        message.child_frame_id = self._base_frame
        message.pose.pose.position.x = estimate.x_m
        message.pose.pose.position.y = estimate.y_m
        message.pose.pose.orientation.z = math.sin(estimate.yaw_rad * 0.5)
        message.pose.pose.orientation.w = math.cos(estimate.yaw_rad * 0.5)
        message.twist.twist.linear.x = estimate.vx_m_s
        message.twist.twist.linear.y = estimate.vy_m_s
        message.twist.twist.angular.z = estimate.wz_rad_s
        message.twist.covariance = self._twist_covariance
        self._odom_publisher.publish(message)

        if self._publish_tf:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self._odom_frame
            transform.child_frame_id = self._base_frame
            transform.transform.translation.x = estimate.x_m
            transform.transform.translation.y = estimate.y_m
            transform.transform.rotation.z = message.pose.pose.orientation.z
            transform.transform.rotation.w = message.pose.pose.orientation.w
            self._tf_broadcaster.sendTransform(transform)

    def _publish_diagnostics(self) -> None:
        now = time.monotonic()
        snapshot = self._tracker.snapshot(
            now=now,
            decoder_stats=self._decoder.stats,
            serial_port=self._port,
            baud_rate=self._baud,
        )

        status = DiagnosticStatus()
        status.name = "learm_serial_bridge"
        status.hardware_id = "stm32f103rbt6"
        if self._serial is None or not self._serial.is_open:
            status.level = DiagnosticStatus.ERROR
            status.message = self._last_serial_error or "serial port is closed"
        elif snapshot.frame_age_s == float("inf") or snapshot.frame_age_s > 0.5:
            status.level = DiagnosticStatus.ERROR
            status.message = "no recent CHASSIS_STATE frame"
        elif snapshot.fault_flags != 0 or snapshot.safety_state == 7:
            status.level = DiagnosticStatus.ERROR
            status.message = "STM32 reports a controller fault"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "receiving chassis telemetry"

        values = snapshot_to_values(snapshot)
        values.update(
            {
                "bytes_received": str(self._decoder.stats.bytes_received),
                "cobs_errors": str(self._decoder.stats.cobs_errors),
                "format_errors": str(self._decoder.stats.format_errors),
                "overflow_errors": str(self._decoder.stats.overflow_errors),
                "duplicate_frames": str(self._decoder.stats.duplicate_frames),
            }
        )
        if self._last_state is not None:
            values["battery_mv"] = str(self._last_state.battery_mv)
            values["motor_commands"] = ",".join(
                str(value) for value in self._last_state.motor_commands
            )
        status.values = [KeyValue(key=key, value=value) for key, value in values.items()]

        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self._diagnostic_publisher.publish(message)

    def destroy_node(self) -> bool:
        if self._serial is not None:
            try:
                self._serial.close()
            except (OSError, serial.SerialException):
                pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LeArmSerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

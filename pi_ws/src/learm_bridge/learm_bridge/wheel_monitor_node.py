"""Terminal dashboard subscribing to the decoded ROS 2 telemetry."""

from __future__ import annotations

from collections import deque
from dataclasses import replace
import math
import sys
import time

from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

from .dashboard import DashboardSnapshot, render_dashboard
from .diagnostic_mapping import apply_diagnostic_values


class WheelMonitor(Node):
    def __init__(self) -> None:
        super().__init__("wheel_monitor")
        self._snapshot = DashboardSnapshot()
        self._last_encoders: tuple[int, int, int, int] | None = None
        self._last_encoder_at: float | None = None
        self._receive_times: deque[float] = deque()
        self._last_odom_xy: tuple[float, float] | None = None
        self._odom_path_m = 0.0

        self.create_subscription(
            Int32MultiArray,
            "/learm/wheel_encoders",
            self._encoder_callback,
            10,
        )
        self.create_subscription(
            DiagnosticArray,
            "/diagnostics",
            self._diagnostic_callback,
            10,
        )
        self.create_subscription(Odometry, "/odom", self._odometry_callback, 10)
        self.create_timer(0.1, self._render)

    def _encoder_callback(self, message: Int32MultiArray) -> None:
        if len(message.data) != 4:
            return

        now = time.monotonic()
        encoders = tuple(int(value) for value in message.data)
        deltas = (0, 0, 0, 0)
        if self._last_encoders is not None:
            deltas = tuple(
                current - previous
                for current, previous in zip(encoders, self._last_encoders)
            )
        self._last_encoders = encoders
        self._last_encoder_at = now

        self._receive_times.append(now)
        cutoff = now - 2.0
        while len(self._receive_times) > 1 and self._receive_times[0] < cutoff:
            self._receive_times.popleft()
        rate_hz = 0.0
        if len(self._receive_times) > 1:
            duration = self._receive_times[-1] - self._receive_times[0]
            if duration > 0.0:
                rate_hz = (len(self._receive_times) - 1) / duration

        self._snapshot = replace(
            self._snapshot,
            encoders=encoders,
            deltas=deltas,
            receive_rate_hz=rate_hz,
            frame_age_s=0.0,
        )

    def _odometry_callback(self, message: Odometry) -> None:
        """显示里程计发布者的位置，并累加相邻位置间的平移距离。"""

        position = message.pose.pose.position
        quaternion = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
            1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
        )
        current_xy = (position.x, position.y)
        if self._last_odom_xy is not None:
            dx = current_xy[0] - self._last_odom_xy[0]
            dy = current_xy[1] - self._last_odom_xy[1]
            self._odom_path_m += math.hypot(dx, dy)
        self._last_odom_xy = current_xy
        self._snapshot = replace(
            self._snapshot,
            odom_x_m=position.x,
            odom_y_m=position.y,
            odom_yaw_rad=yaw,
            odom_path_m=self._odom_path_m,
            odom_received=True,
        )

    def _diagnostic_callback(self, message: DiagnosticArray) -> None:
        for status in message.status:
            if status.name != "learm_serial_bridge":
                continue
            values = {item.key: item.value for item in status.values}
            self._snapshot = apply_diagnostic_values(self._snapshot, values)
            break

    def _render(self) -> None:
        snapshot = self._snapshot
        if self._last_encoder_at is not None:
            snapshot = replace(
                snapshot,
                frame_age_s=max(0.0, time.monotonic() - self._last_encoder_at),
            )
        sys.stdout.write("\x1b[2J\x1b[H")
        sys.stdout.write(render_dashboard(snapshot))
        sys.stdout.write("\n")
        sys.stdout.flush()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WheelMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

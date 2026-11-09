"""麦克纳姆底盘编码器里程计数学模块。

编码器顺序必须与 STM32 协议和 Armrobot 原工程一致：
``[front_right, rear_right, front_left, rear_left]``。
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence


@dataclass(frozen=True)
class OdometryEstimate:
    """一次编码器更新得到的二维位姿和车体速度。"""

    x_m: float = 0.0
    y_m: float = 0.0
    yaw_rad: float = 0.0
    vx_m_s: float = 0.0
    vy_m_s: float = 0.0
    wz_rad_s: float = 0.0


class MecanumOdometry:
    """根据四个轮子的累计计数积分底盘里程计。"""

    def __init__(
        self,
        *,
        wheel_radius_m: float = 0.048,
        wheel_base_m: float = 0.190,
        track_width_m: float = 0.205,
        encoder_counts_per_revolution: float = 5740.25304043815,
    ) -> None:
        for name, value in (
            ("wheel_radius_m", wheel_radius_m),
            ("wheel_base_m", wheel_base_m),
            ("track_width_m", track_width_m),
            ("encoder_counts_per_revolution", encoder_counts_per_revolution),
        ):
            if value <= 0.0 or not math.isfinite(value):
                raise ValueError(f"{name} must be a finite positive number")

        self._meters_per_tick = (
            2.0 * math.pi * wheel_radius_m / encoder_counts_per_revolution
        )
        self._wheel_base_m = wheel_base_m
        self._track_width_m = track_width_m
        self._last_counts: tuple[int, int, int, int] | None = None
        self._last_time_s: float | None = None
        self._estimate = OdometryEstimate()

    @property
    def meters_per_tick(self) -> float:
        """返回当前参数对应的每 tick 行驶距离。"""

        return self._meters_per_tick

    def reset(
        self,
        encoders: Sequence[int] | None = None,
        timestamp_s: float | None = None,
    ) -> OdometryEstimate:
        """清零位姿，并可选地建立新的编码器/时间基线。"""

        self._estimate = OdometryEstimate()
        self._last_counts = None if encoders is None else self._validate_counts(encoders)
        self._last_time_s = timestamp_s
        return self._estimate

    def update(
        self,
        encoders: Sequence[int],
        timestamp_s: float,
    ) -> OdometryEstimate:
        """消费一组累计计数并返回最新里程计。

        第一帧只建基线，不会把设备上电前的累计计数误认为移动距离。
        """

        current = self._validate_counts(encoders)
        if not math.isfinite(timestamp_s):
            raise ValueError("timestamp_s must be finite")

        if self._last_counts is None or self._last_time_s is None:
            self._last_counts = current
            self._last_time_s = timestamp_s
            return self._estimate

        previous = self._last_counts
        previous_time = self._last_time_s
        self._last_counts = current
        self._last_time_s = timestamp_s
        dt = timestamp_s - previous_time
        if dt <= 0.0:
            return self._estimate

        deltas = tuple(
            current[index] - previous[index] for index in range(4)
        )
        return self._apply_delta(deltas, dt)

    def _apply_delta(
        self,
        deltas: tuple[int, int, int, int],
        dt: float,
    ) -> OdometryEstimate:
        """按 Armrobot 已验证的麦轮符号约定积分一次增量。"""

        # 原工程使用 forward=(FR-RR-FL+RL)/4，横向为四轮平均值。
        forward_ticks = (deltas[0] - deltas[1] - deltas[2] + deltas[3]) * 0.25
        left_ticks = sum(deltas) * 0.25
        rotation_ticks = deltas[0] + deltas[1] - deltas[2] - deltas[3]

        forward_m = forward_ticks * self._meters_per_tick
        left_m = left_ticks * self._meters_per_tick
        yaw_delta = (
            rotation_ticks * self._meters_per_tick
            / (2.0 * (self._wheel_base_m + self._track_width_m))
        )

        old_yaw = self._estimate.yaw_rad
        self._estimate = OdometryEstimate(
            x_m=self._estimate.x_m + forward_m * math.cos(old_yaw) - left_m * math.sin(old_yaw),
            y_m=self._estimate.y_m + forward_m * math.sin(old_yaw) + left_m * math.cos(old_yaw),
            yaw_rad=_wrap_angle(old_yaw + yaw_delta),
            vx_m_s=forward_m / dt,
            vy_m_s=left_m / dt,
            wz_rad_s=yaw_delta / dt,
        )
        return self._estimate

    @staticmethod
    def _validate_counts(encoders: Sequence[int]) -> tuple[int, int, int, int]:
        if len(encoders) != 4:
            raise ValueError("exactly four encoder counts are required")
        return tuple(int(value) for value in encoders)  # type: ignore[return-value]


def _wrap_angle(angle_rad: float) -> float:
    """将航向角限制在 [-pi, pi)。"""

    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi

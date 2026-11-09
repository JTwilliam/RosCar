"""Terminal dashboard model and renderer kept independent from ROS 2."""

from __future__ import annotations

from dataclasses import dataclass


SAFETY_STATE_NAMES = {
    0: "STARTUP",
    1: "NORMAL",
    2: "STOPPING",
    3: "SAFE_IDLE",
    4: "ESTOP_LATCHED",
    5: "ESTOP_READY",
    6: "BOOT_PENDING",
    7: "FAULT",
}


@dataclass(frozen=True)
class DashboardSnapshot:
    encoders: tuple[int, int, int, int] = (0, 0, 0, 0)
    deltas: tuple[int, int, int, int] = (0, 0, 0, 0)
    firmware_timestamp_ms: int = 0
    sample_sequence: int = 0
    receive_rate_hz: float = 0.0
    frame_age_s: float = float("inf")
    fault_flags: int = 0
    safety_state: int = 0
    frames_valid: int = 0
    frames_dropped: int = 0
    crc_errors: int = 0
    sequence_drops: int = 0
    serial_port: str = "-"
    baud_rate: int = 0
    odom_x_m: float = 0.0
    odom_y_m: float = 0.0
    odom_yaw_rad: float = 0.0
    odom_path_m: float = 0.0
    odom_received: bool = False


def render_dashboard(snapshot: DashboardSnapshot) -> str:
    safety_name = SAFETY_STATE_NAMES.get(
        snapshot.safety_state, f"UNKNOWN({snapshot.safety_state})"
    )
    age = "no data" if snapshot.frame_age_s == float("inf") else f"{snapshot.frame_age_s:.3f} s"
    displacement_m = (snapshot.odom_x_m**2 + snapshot.odom_y_m**2) ** 0.5

    lines = [
        "LeArm STM32 / ROS 2 wheel monitor",
        "=" * 64,
        f"Serial : {snapshot.serial_port}",
        f"Link   : {snapshot.baud_rate} 8N1 | {snapshot.receive_rate_hz:5.1f} Hz | age {age}",
        "-" * 64,
        "Wheel       Encoder count          Delta",
    ]
    for index, (encoder, delta) in enumerate(zip(snapshot.encoders, snapshot.deltas)):
        lines.append(f"Wheel {index}  {encoder:18d}  {delta:+13d}")

    lines.extend(
        [
            "-" * 64,
            f"STM32 timestamp : {snapshot.firmware_timestamp_ms:10d} ms",
            f"Sample sequence : {snapshot.sample_sequence:10d}",
            f"Safety          : {safety_name}",
            f"Fault flags     : 0x{snapshot.fault_flags:04X}",
            "-" * 64,
            "Odometry (from /odom)",
            (
                f"Pose x/y/yaw   : {snapshot.odom_x_m:+.3f} / "
                f"{snapshot.odom_y_m:+.3f} m / {snapshot.odom_yaw_rad:+.3f} rad"
                if snapshot.odom_received
                else "Pose x/y/yaw   : waiting for /odom"
            ),
            f"Displacement   : {displacement_m:.3f} m",
            f"Travel distance: {snapshot.odom_path_m:.3f} m",
            "-" * 64,
            f"Valid frames    : {snapshot.frames_valid}",
            f"Dropped frames  : {snapshot.frames_dropped}",
            f"CRC errors      : {snapshot.crc_errors}",
            f"Sequence drops  : {snapshot.sequence_drops}",
            "",
            "Press Ctrl+C to exit the monitor.",
        ]
    )
    return "\n".join(lines)

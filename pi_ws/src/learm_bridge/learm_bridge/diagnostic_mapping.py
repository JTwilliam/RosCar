"""String mapping used by diagnostic_msgs and the terminal monitor."""

from __future__ import annotations

from dataclasses import replace
from typing import Mapping

from .dashboard import DashboardSnapshot


def snapshot_to_values(snapshot: DashboardSnapshot) -> dict[str, str]:
    return {
        "firmware_timestamp_ms": str(snapshot.firmware_timestamp_ms),
        "sample_sequence": str(snapshot.sample_sequence),
        "receive_rate_hz": f"{snapshot.receive_rate_hz:.3f}",
        "frame_age_s": f"{snapshot.frame_age_s:.6f}",
        "fault_flags": f"0x{snapshot.fault_flags:04X}",
        "safety_state": str(snapshot.safety_state),
        "frames_valid": str(snapshot.frames_valid),
        "frames_dropped": str(snapshot.frames_dropped),
        "crc_errors": str(snapshot.crc_errors),
        "sequence_drops": str(snapshot.sequence_drops),
        "serial_port": snapshot.serial_port,
        "baud_rate": str(snapshot.baud_rate),
    }


def _parse_int(values: Mapping[str, str], key: str, previous: int) -> int:
    try:
        return int(values[key], 0)
    except (KeyError, TypeError, ValueError):
        return previous


def _parse_float(values: Mapping[str, str], key: str, previous: float) -> float:
    try:
        return float(values[key])
    except (KeyError, TypeError, ValueError):
        return previous


def apply_diagnostic_values(
    previous: DashboardSnapshot, values: Mapping[str, str]
) -> DashboardSnapshot:
    return replace(
        previous,
        firmware_timestamp_ms=_parse_int(
            values, "firmware_timestamp_ms", previous.firmware_timestamp_ms
        ),
        sample_sequence=_parse_int(values, "sample_sequence", previous.sample_sequence),
        receive_rate_hz=_parse_float(
            values, "receive_rate_hz", previous.receive_rate_hz
        ),
        frame_age_s=_parse_float(values, "frame_age_s", previous.frame_age_s),
        fault_flags=_parse_int(values, "fault_flags", previous.fault_flags),
        safety_state=_parse_int(values, "safety_state", previous.safety_state),
        frames_valid=_parse_int(values, "frames_valid", previous.frames_valid),
        frames_dropped=_parse_int(
            values, "frames_dropped", previous.frames_dropped
        ),
        crc_errors=_parse_int(values, "crc_errors", previous.crc_errors),
        sequence_drops=_parse_int(
            values, "sequence_drops", previous.sequence_drops
        ),
        serial_port=values.get("serial_port", previous.serial_port),
        baud_rate=_parse_int(values, "baud_rate", previous.baud_rate),
    )

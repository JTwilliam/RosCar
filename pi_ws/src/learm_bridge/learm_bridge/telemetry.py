"""State tracking shared by the ROS bridge and terminal monitor."""

from __future__ import annotations

from collections import deque

from .dashboard import DashboardSnapshot
from .protocol import ChassisState, DecoderStats


class ChassisTelemetryTracker:
    def __init__(self, window_seconds: float = 2.0) -> None:
        if window_seconds <= 0.0:
            raise ValueError("window_seconds must be positive")
        self._window_seconds = window_seconds
        self._receive_times: deque[float] = deque()
        self._state: ChassisState | None = None
        self._deltas = (0, 0, 0, 0)
        self._last_received_at: float | None = None

    def update(self, state: ChassisState, received_at: float) -> None:
        if self._state is not None:
            self._deltas = tuple(
                current - previous
                for current, previous in zip(state.encoders, self._state.encoders)
            )
        self._state = state
        self._last_received_at = received_at
        self._receive_times.append(received_at)

        cutoff = received_at - self._window_seconds
        while len(self._receive_times) > 1 and self._receive_times[0] < cutoff:
            self._receive_times.popleft()

    def snapshot(
        self,
        *,
        now: float,
        decoder_stats: DecoderStats,
        serial_port: str,
        baud_rate: int,
    ) -> DashboardSnapshot:
        state = self._state
        if state is None or self._last_received_at is None:
            return DashboardSnapshot(
                frames_valid=decoder_stats.frames_valid,
                frames_dropped=decoder_stats.frames_dropped,
                crc_errors=decoder_stats.crc_errors,
                sequence_drops=decoder_stats.sequence_drops,
                serial_port=serial_port,
                baud_rate=baud_rate,
            )

        rate_hz = 0.0
        if len(self._receive_times) > 1:
            duration = self._receive_times[-1] - self._receive_times[0]
            if duration > 0.0:
                rate_hz = (len(self._receive_times) - 1) / duration

        return DashboardSnapshot(
            encoders=state.encoders,
            deltas=self._deltas,
            firmware_timestamp_ms=state.timestamp_ms,
            sample_sequence=state.sequence,
            receive_rate_hz=rate_hz,
            frame_age_s=max(0.0, now - self._last_received_at),
            fault_flags=state.fault_flags,
            safety_state=state.safety_state,
            frames_valid=decoder_stats.frames_valid,
            frames_dropped=decoder_stats.frames_dropped,
            crc_errors=decoder_stats.crc_errors,
            sequence_drops=decoder_stats.sequence_drops,
            serial_port=serial_port,
            baud_rate=baud_rate,
        )

from learm_bridge.protocol import ChassisState, DecoderStats
from learm_bridge.telemetry import ChassisTelemetryTracker


def make_state(timestamp_ms, encoders, sequence):
    return ChassisState(
        timestamp_ms=timestamp_ms,
        encoders=encoders,
        motor_commands=(0, 0, 0, 0),
        battery_mv=7400,
        fault_flags=0,
        safety_state=3,
        sequence=sequence,
    )


def test_tracker_calculates_wheel_deltas_and_receive_rate():
    tracker = ChassisTelemetryTracker(window_seconds=1.0)
    tracker.update(make_state(1000, (10, 20, 30, 40), 1), received_at=2.00)
    tracker.update(make_state(1020, (12, 17, 35, 40), 2), received_at=2.02)
    tracker.update(make_state(1040, (15, 15, 42, 41), 3), received_at=2.04)

    snapshot = tracker.snapshot(
        now=2.05,
        decoder_stats=DecoderStats(frames_valid=3, crc_errors=1),
        serial_port="/dev/ttyUSB0",
        baud_rate=115200,
    )

    assert snapshot.encoders == (15, 15, 42, 41)
    assert snapshot.deltas == (3, -2, 7, 1)
    assert 49.9 < snapshot.receive_rate_hz < 50.1
    assert 0.009 < snapshot.frame_age_s < 0.011
    assert snapshot.firmware_timestamp_ms == 1040
    assert snapshot.sample_sequence == 3
    assert snapshot.crc_errors == 1


def test_empty_tracker_reports_no_data():
    tracker = ChassisTelemetryTracker()

    snapshot = tracker.snapshot(
        now=8.0,
        decoder_stats=DecoderStats(),
        serial_port="/dev/ttyUSB0",
        baud_rate=115200,
    )

    assert snapshot.frame_age_s == float("inf")
    assert snapshot.receive_rate_hz == 0.0
    assert snapshot.encoders == (0, 0, 0, 0)

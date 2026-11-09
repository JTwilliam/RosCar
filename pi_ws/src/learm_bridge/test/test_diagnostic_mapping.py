from learm_bridge.dashboard import DashboardSnapshot
from learm_bridge.diagnostic_mapping import apply_diagnostic_values, snapshot_to_values


def test_diagnostic_values_round_trip_monitor_fields():
    source = DashboardSnapshot(
        firmware_timestamp_ms=4040,
        sample_sequence=21,
        receive_rate_hz=50.0,
        frame_age_s=0.02,
        fault_flags=0x10,
        safety_state=7,
        frames_valid=200,
        frames_dropped=4,
        crc_errors=2,
        sequence_drops=1,
        serial_port="/dev/ttyUSB0",
        baud_rate=115200,
    )

    values = snapshot_to_values(source)
    restored = apply_diagnostic_values(DashboardSnapshot(), values)

    assert restored.firmware_timestamp_ms == 4040
    assert restored.sample_sequence == 21
    assert restored.receive_rate_hz == 50.0
    assert restored.frame_age_s == 0.02
    assert restored.fault_flags == 0x10
    assert restored.safety_state == 7
    assert restored.frames_valid == 200
    assert restored.frames_dropped == 4
    assert restored.crc_errors == 2
    assert restored.sequence_drops == 1
    assert restored.serial_port == "/dev/ttyUSB0"
    assert restored.baud_rate == 115200


def test_invalid_diagnostic_value_keeps_previous_field():
    previous = DashboardSnapshot(receive_rate_hz=49.5, fault_flags=3)

    restored = apply_diagnostic_values(
        previous,
        {"receive_rate_hz": "not-a-number", "fault_flags": "0xZZZZ"},
    )

    assert restored.receive_rate_hz == 49.5
    assert restored.fault_flags == 3

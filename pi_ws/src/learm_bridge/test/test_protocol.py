from learm_bridge.protocol import (
    MSG_CHASSIS_STATE,
    ProtocolDecoder,
    decode_chassis_state,
)


# Captured from the STM32F103RBT6 at 115200 8N1. The leading 0xff seen
# immediately after reset is intentionally excluded from this valid frame.
CAPTURED_CHASSIS_FRAME = bytes.fromhex(
    "03 01 81 02 0c 01 02 20 03 01 04 01 03 01 04 01 "
    "01 01 01 01 01 01 01 01 01 01 01 01 01 01 01 01 "
    "01 01 01 01 01 01 02 10 02 07 01 01 05 6f f5 3d 3b 00"
)


def test_decodes_captured_chassis_frame():
    decoder = ProtocolDecoder()

    frames = decoder.feed(CAPTURED_CHASSIS_FRAME)

    assert len(frames) == 1
    frame = frames[0]
    assert frame.message_type == MSG_CHASSIS_STATE
    assert frame.timestamp_ms == 1025
    state = decode_chassis_state(frame)
    assert state.timestamp_ms == 1025
    assert state.encoders == (0, 0, 0, 0)
    assert state.motor_commands == (0, 0, 0, 0)
    assert state.battery_mv == 0
    assert state.fault_flags == 0x0010
    assert state.safety_state == 7


def test_recovers_after_reset_noise_candidate():
    decoder = ProtocolDecoder()

    frames = decoder.feed(b"\xff" + CAPTURED_CHASSIS_FRAME + CAPTURED_CHASSIS_FRAME)

    assert len(frames) == 1
    assert frames[0].timestamp_ms == 1025
    assert decoder.stats.frames_dropped == 1


def test_rejects_crc_corruption_and_counts_it():
    decoder = ProtocolDecoder()
    corrupted = bytearray(CAPTURED_CHASSIS_FRAME)
    corrupted[-3] ^= 0x01

    frames = decoder.feed(corrupted)

    assert frames == []
    assert decoder.stats.crc_errors == 1
    assert decoder.stats.frames_dropped == 1


def test_accepts_fragmented_serial_reads():
    decoder = ProtocolDecoder()

    first = decoder.feed(CAPTURED_CHASSIS_FRAME[:17])
    second = decoder.feed(CAPTURED_CHASSIS_FRAME[17:])

    assert first == []
    assert len(second) == 1
    assert decoder.stats.frames_valid == 1


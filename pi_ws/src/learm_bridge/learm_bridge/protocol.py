"""LeArm STM32 serial framing and telemetry decoding.

The transport is COBS framed with a trailing 0x00 delimiter. Decoded frames
use the little-endian layout documented in robot_protocol_v1.md.
"""

from __future__ import annotations

from dataclasses import dataclass
import struct
from typing import Iterable


PROTOCOL_VERSION = 1
HEADER_LENGTH = 12
CRC_LENGTH = 4
MAX_PAYLOAD_LENGTH = 128
MAX_DECODED_LENGTH = 256
MAX_ENCODED_LENGTH = 258

MSG_HELLO_STATE = 0x80
MSG_CHASSIS_STATE = 0x81
MSG_ARM_STATE = 0x82
MSG_HEALTH_STATE = 0x83


@dataclass(frozen=True)
class Frame:
    version: int
    message_type: int
    flags: int
    sequence: int
    timestamp_ms: int
    payload: bytes


@dataclass(frozen=True)
class ChassisState:
    timestamp_ms: int
    encoders: tuple[int, int, int, int]
    motor_commands: tuple[int, int, int, int]
    battery_mv: int
    fault_flags: int
    safety_state: int
    sequence: int


@dataclass
class DecoderStats:
    bytes_received: int = 0
    frames_valid: int = 0
    frames_dropped: int = 0
    cobs_errors: int = 0
    crc_errors: int = 0
    format_errors: int = 0
    overflow_errors: int = 0
    sequence_drops: int = 0
    duplicate_frames: int = 0


class _CobsError(ValueError):
    pass


class _CrcError(ValueError):
    pass


class _FormatError(ValueError):
    pass


def crc32c(data: bytes | bytearray | memoryview) -> int:
    """Return CRC32C Castagnoli using the STM32 reflected implementation."""
    crc = 0xFFFFFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0x82F63B78
            else:
                crc >>= 1
    return crc ^ 0xFFFFFFFF


def cobs_decode(encoded: bytes | bytearray | memoryview) -> bytes:
    """Decode one COBS candidate without its trailing delimiter."""
    if not encoded:
        raise _CobsError("empty COBS candidate")

    output = bytearray()
    index = 0
    length = len(encoded)
    while index < length:
        code = encoded[index]
        if code == 0:
            raise _CobsError("zero byte inside COBS candidate")
        index += 1
        copy_length = code - 1
        end = index + copy_length
        if end > length:
            raise _CobsError("COBS code exceeds candidate length")
        output.extend(encoded[index:end])
        index = end
        if code != 0xFF and index < length:
            output.append(0)
        if len(output) > MAX_DECODED_LENGTH:
            raise _CobsError("decoded frame exceeds limit")
    return bytes(output)


def decode_frame(encoded_candidate: bytes | bytearray | memoryview) -> Frame:
    decoded = cobs_decode(encoded_candidate)
    if len(decoded) < HEADER_LENGTH + CRC_LENGTH:
        raise _FormatError("decoded frame is too short")

    version, message_type, flags, header_length, sequence, payload_length, timestamp_ms = (
        struct.unpack_from("<BBBBHHI", decoded, 0)
    )
    if version != PROTOCOL_VERSION:
        raise _FormatError(f"unsupported protocol version {version}")
    if header_length != HEADER_LENGTH:
        raise _FormatError(f"invalid header length {header_length}")
    if payload_length > MAX_PAYLOAD_LENGTH:
        raise _FormatError("payload exceeds limit")

    expected_length = HEADER_LENGTH + payload_length + CRC_LENGTH
    if len(decoded) != expected_length:
        raise _FormatError("decoded frame length does not match payload length")

    received_crc = struct.unpack_from("<I", decoded, HEADER_LENGTH + payload_length)[0]
    expected_crc = crc32c(decoded[: HEADER_LENGTH + payload_length])
    if received_crc != expected_crc:
        raise _CrcError(
            f"CRC32C mismatch: received 0x{received_crc:08x}, "
            f"expected 0x{expected_crc:08x}"
        )

    return Frame(
        version=version,
        message_type=message_type,
        flags=flags,
        sequence=sequence,
        timestamp_ms=timestamp_ms,
        payload=decoded[HEADER_LENGTH : HEADER_LENGTH + payload_length],
    )


def decode_chassis_state(frame: Frame) -> ChassisState:
    if frame.message_type != MSG_CHASSIS_STATE:
        raise ValueError(f"expected CHASSIS_STATE, got 0x{frame.message_type:02x}")
    if len(frame.payload) != 32:
        raise ValueError(f"CHASSIS_STATE payload must be 32 bytes, got {len(frame.payload)}")

    values = struct.unpack("<I4i4bHHBBH", frame.payload)
    return ChassisState(
        timestamp_ms=values[0],
        encoders=tuple(values[1:5]),
        motor_commands=tuple(values[5:9]),
        battery_mv=values[9],
        fault_flags=values[10],
        safety_state=values[11],
        sequence=values[13],
    )


class ProtocolDecoder:
    """Incremental byte-stream decoder with delimiter resynchronization."""

    def __init__(self) -> None:
        self.stats = DecoderStats()
        self._candidate = bytearray()
        self._discard_until_delimiter = False
        self._last_sequence: int | None = None
        self._last_timestamp_ms: int | None = None

    def feed(self, data: bytes | bytearray | memoryview | Iterable[int]) -> list[Frame]:
        frames: list[Frame] = []
        for value in data:
            byte = int(value) & 0xFF
            self.stats.bytes_received += 1

            if self._discard_until_delimiter:
                if byte == 0:
                    self._discard_until_delimiter = False
                    self.stats.frames_dropped += 1
                continue

            if byte != 0:
                if len(self._candidate) >= MAX_ENCODED_LENGTH:
                    self._candidate.clear()
                    self._discard_until_delimiter = True
                    self.stats.overflow_errors += 1
                else:
                    self._candidate.append(byte)
                continue

            if not self._candidate:
                continue

            candidate = bytes(self._candidate)
            self._candidate.clear()
            try:
                frame = decode_frame(candidate)
            except _CobsError:
                self.stats.cobs_errors += 1
                self.stats.frames_dropped += 1
                continue
            except _CrcError:
                self.stats.crc_errors += 1
                self.stats.frames_dropped += 1
                continue
            except _FormatError:
                self.stats.format_errors += 1
                self.stats.frames_dropped += 1
                continue

            self._track_sequence(frame)
            self.stats.frames_valid += 1
            frames.append(frame)
        return frames

    def _track_sequence(self, frame: Frame) -> None:
        if self._last_sequence is None:
            self._last_sequence = frame.sequence
            self._last_timestamp_ms = frame.timestamp_ms
            return

        # A decreasing STM32 tick normally means the device restarted. Do not
        # turn that restart into a huge modulo sequence-loss count.
        if self._last_timestamp_ms is not None and frame.timestamp_ms < self._last_timestamp_ms:
            self._last_sequence = frame.sequence
            self._last_timestamp_ms = frame.timestamp_ms
            return

        difference = (frame.sequence - self._last_sequence) & 0xFFFF
        if difference == 0:
            self.stats.duplicate_frames += 1
        elif difference > 1:
            self.stats.sequence_drops += difference - 1
        self._last_sequence = frame.sequence
        self._last_timestamp_ms = frame.timestamp_ms


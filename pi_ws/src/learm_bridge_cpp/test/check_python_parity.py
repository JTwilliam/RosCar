"""Compare the C++ replay against the unchanged Python reference, no ROS needed."""
import csv
import io
import math
from pathlib import Path
import random
import struct
import subprocess
import sys
import tempfile

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE.parent / 'learm_bridge'))
from learm_bridge.protocol import ProtocolDecoder, decode_chassis_state, crc32c
from learm_bridge.odometry import MecanumOdometry


def cobs(raw):
    result = bytearray([0])
    code_index, code = 0, 1
    for value in raw:
        if value == 0:
            result[code_index] = code
            code_index = len(result)
            result.append(0)
            code = 1
        else:
            result.append(value)
            code += 1
            if code == 255:
                result[code_index] = code
                code_index = len(result)
                result.append(0)
                code = 1
    result[code_index] = code
    return bytes(result) + b'\x00'


def frame(sequence, timestamp, counts):
    payload = struct.pack('<I4i4bHHBBH', timestamp, *counts,
                          -100, -1, 0, 100, 12000, 16, 7, 0, sequence)
    raw = struct.pack('<BBBBHHI', 1, 0x81, 0, 12, sequence, len(payload), timestamp) + payload
    return cobs(raw + struct.pack('<I', crc32c(raw)))


def main():
    binary = Path(sys.argv[1]).resolve()
    rng = random.Random(20261109)
    stream = bytearray(b'\xff\x00')
    counts = [0] * 4
    timestamp = 1000
    for i in range(160):
        if i == 80:  # STM32 reset: preserve the Python reset semantics.
            timestamp, counts = 10, [0] * 4
        elif i % 17 != 0:  # Also include repeated firmware timestamps.
            timestamp += 20
        counts = [n + rng.randint(-200, 200) for n in counts]
        packet = frame((65500 + i * 2) & 65535, timestamp, counts)
        if i == 40:
            damaged = bytearray(packet)
            damaged[-3] ^= 1
            stream.extend(damaged)
        if i == 60:
            stream.extend(b'\xff' * 300 + b'\x00')
        stream.extend(packet)
        if i == 30:
            stream.extend(packet)

    decoder, odom = ProtocolDecoder(), MecanumOdometry()
    expected = []
    last_time = None
    # Deliberately different read sizes from the C++ file replay.
    for offset in range(0, len(stream), 13):
        for decoded in decoder.feed(stream[offset:offset + 13]):
            state = decode_chassis_state(decoded)
            if last_time is not None and state.timestamp_ms < last_time:
                odom.reset(state.encoders, state.timestamp_ms * .001)
            last_time = state.timestamp_ms
            estimate = odom.update(state.encoders, state.timestamp_ms * .001)
            expected.append([state.timestamp_ms, *state.encoders,
                             estimate.x_m, estimate.y_m, estimate.yaw_rad,
                             estimate.vx_m_s, estimate.vy_m_s, estimate.wz_rad_s])
    # Keep temporary data on the project drive, not the host's default /tmp.
    scratch = PACKAGE / 'build' / 'parity'
    scratch.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=scratch) as folder:
        capture = Path(folder) / 'uart.bin'
        capture.write_bytes(stream)
        result = subprocess.run([str(binary), str(capture)], capture_output=True,
                                text=True, check=True)
    actual = list(csv.reader(io.StringIO(result.stdout)))[1:]
    assert len(actual) == len(expected), (len(actual), len(expected))
    for row_index, (got, want) in enumerate(zip(actual, expected)):
        assert len(got) == len(want)
        for column, (value, reference) in enumerate(zip(got, want)):
            assert math.isclose(float(value), reference, rel_tol=1e-10, abs_tol=1e-11), (
                row_index, column, value, reference)
    reported = dict(item.split('=', 1) for item in result.stderr.split())
    for name, label in {'frames_valid': 'valid', 'frames_dropped': 'dropped',
                        'crc_errors': 'crc', 'cobs_errors': 'cobs',
                        'overflow_errors': 'overflow', 'format_errors': 'format',
                        'sequence_drops': 'sequence_drops', 'duplicate_frames': 'duplicates'}.items():
        assert int(reported[label]) == getattr(decoder.stats, name), (name, result.stderr)
    print(f'PASS: {len(expected)} C++/Python telemetry and odometry rows; corruption/reset/overflow')


if __name__ == '__main__':
    main()

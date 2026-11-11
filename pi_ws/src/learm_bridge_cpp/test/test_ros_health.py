"""PTY health stream through actual ROS bridge, independent of odometry."""
import importlib.util
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import tempfile
import time
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

spec = importlib.util.spec_from_file_location('fixtures', Path(__file__).with_name('check_python_parity.py'))
fixtures = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fixtures)


def packet(stamp, active=0, history=0, safety=1, legacy=False, task_age=3):
    p = bytearray(44 if legacy else 128)
    struct.pack_into('<II', p, 0, stamp, 4096)
    struct.pack_into('<5H', p, 8, 100, 100, 120, 120, 100)
    struct.pack_into('<HBB', p, 40, history, safety, 0)
    if not legacy:
        struct.pack_into('<BBHHHI', p, 44, 1, 31, active, history, history, 100)
        struct.pack_into('<5H', p, 56, 1, task_age, 2, 5, 0)
        struct.pack_into('<HHHBBBBHHIII', p, 66, 10, 0, 35, 1, 1, 0, 0, 8, 0, 90, stamp, 0)
        p[92:108] = b'test-fw'.ljust(16, b'\0')
    raw = struct.pack('<BBBBHHI', 1, 0x83, 0, 12, stamp & 65535, len(p), stamp) + p
    return fixtures.cobs(raw + struct.pack('<I', fixtures.crc32c(raw)))


def main():
    binary = str(Path(sys.argv[1]).resolve())
    firmware_fixture = Path(sys.argv[2]).read_bytes() if len(sys.argv) > 2 else None
    rclpy.init()
    node = rclpy.create_node('health_integration_test')
    latest = {}
    def receive(msg):
        for status in msg.status:
            if status.name.startswith('learm/health/'):
                latest[status.name.rsplit('/', 1)[-1]] = status
    subscription = node.create_subscription(DiagnosticArray, '/diagnostics', receive, 100)
    master, slave = os.openpty()
    path = Path(__file__).resolve().parents[1] / 'build/health_test'
    path.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=path) as folder:
        with open(Path(folder) / 'node.log', 'w+') as log:
            proc = subprocess.Popen([binary, '--ros-args', '-p', f'port:={os.ttyname(slave)}',
                '-p', 'publish_tf:=false', '-p', 'diagnostics_period_s:=0.1',
                '-p', 'health_stale_after_s:=0.3', '-p', 'health_offline_after_s:=0.8'],
                stdout=log, stderr=log)
            def wait(predicate, timeout=5):
                until = time.monotonic() + timeout
                while time.monotonic() < until:
                    rclpy.spin_once(node, timeout_sec=.02)
                    if predicate(): return
                    assert proc.poll() is None, proc.returncode
                raise AssertionError(f'health wait timed out: {latest}')
            def vals(name):
                return {v.key: v.value for v in latest[name].values}
            try:
                wait(lambda: 'link' in latest)
                assert latest['link'].level == DiagnosticStatus.STALE
                # Allow subscription discovery before sending one fresh snapshot.
                time.sleep(.2)
                os.write(master, packet(1000))
                wait(lambda: latest['link'].message == 'ONLINE')
                assert set(('link','chassis','arm','communication','safety','resources','tasks')) <= latest.keys()
                assert latest['arm'].level != DiagnosticStatus.ERROR, 'idle arm was called faulty'
                assert vals('link')['firmware_id'] == 'test-fw'
                # A stream of identical snapshots must expire even though bytes arrive.
                for _ in range(6):
                    os.write(master, packet(1000))
                    rclpy.spin_once(node, timeout_sec=.08)
                    time.sleep(.06)
                wait(lambda: latest['link'].message == 'STALE', 1)
                wait(lambda: latest['link'].message == 'OFFLINE', 2)
                # Recovery preserves historical evidence without keeping current error red.
                os.write(master, packet(2000, active=16, history=16))
                wait(lambda: latest['chassis'].level == DiagnosticStatus.ERROR)
                os.write(master, packet(2100, active=0, history=16))
                wait(lambda: latest['link'].message == 'ONLINE' and latest['chassis'].level == DiagnosticStatus.OK)
                assert vals('link')['historical_faults'] != 'NONE'
                os.write(master, packet(2200, history=24, safety=4))
                wait(lambda: latest['safety'].level == DiagnosticStatus.WARN)
                assert latest['resources'].level == DiagnosticStatus.OK
                # Invalid schema must not replace a valid snapshot.
                malformed = bytearray(128)
                malformed[44] = 99
                raw = struct.pack('<BBBBHHI', 1, 0x83, 0, 12, 90, 128, 2250) + malformed
                os.write(master, fixtures.cobs(raw + struct.pack('<I', fixtures.crc32c(raw))))
                os.write(master, packet(2300, legacy=True))
                wait(lambda: vals('link').get('firmware_id') in ('UNKNOWN', 'unknown'))
                assert latest['chassis'].level != DiagnosticStatus.OK
                if firmware_fixture is not None:
                    assert len(firmware_fixture) == 128
                    raw = struct.pack('<BBBBHHI', 1, 0x83, 0, 12, 99, 128, 0x12345678) + firmware_fixture
                    os.write(master, fixtures.cobs(raw + struct.pack('<I', fixtures.crc32c(raw))))
                    wait(lambda: vals('link').get('firmware_id') == 'freertos-health1')
                    assert 'MOTOR' in vals('link')['active_faults']
                    assert vals('resources')['minimum_heap_bytes'] == '2048'
                    assert vals('tasks')['heartbeat_age_ms'] == '10,11,12,13,unknown'
                    print('PASS: firmware C encoder fixture decoded through PTY and ROS')
                proc.send_signal(signal.SIGINT); proc.wait(timeout=5)
                assert proc.returncode == 0
                print('PASS: health legacy/extended, frozen snapshots, stale/offline, recovery, history, idle/estop')
            except Exception:
                log.flush(); log.seek(0); print(log.read(), file=sys.stderr); raise
            finally:
                if proc.poll() is None:
                    proc.send_signal(signal.SIGINT)
                    try: proc.wait(timeout=5)
                    except subprocess.TimeoutExpired: proc.kill(); proc.wait(timeout=5)
                os.close(master); os.close(slave)
                node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()

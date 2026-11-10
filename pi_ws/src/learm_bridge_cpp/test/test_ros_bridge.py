"""Actual C++ process + Linux PTY + ROS subscriptions, without hardware."""
import importlib.util
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32MultiArray
from tf2_msgs.msg import TFMessage

SPEC = importlib.util.spec_from_file_location('fixtures', Path(__file__).with_name('check_python_parity.py'))
FIXTURES = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(FIXTURES)


def main():
    binary = str(Path(sys.argv[1]).resolve())
    scratch = Path(__file__).resolve().parents[1] / 'build/ros_test'
    scratch.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    listener = rclpy.create_node('cpp_bridge_integration_test')
    odom, counts, diagnostics, transforms = [], [], [], []
    subscriptions = [
        listener.create_subscription(Odometry, '/odom', odom.append, 100),
        listener.create_subscription(Int32MultiArray, '/learm/wheel_encoders', counts.append, 100),
        listener.create_subscription(DiagnosticArray, '/diagnostics', diagnostics.append, 100),
        listener.create_subscription(TFMessage, '/tf', transforms.append, 100),
    ]
    processes, descriptors = [], []

    def wait_for(predicate, seconds=5):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(listener, timeout_sec=.02)
            if predicate():
                return
            for proc in processes:
                assert proc.poll() is None, f'bridge exited: {proc.returncode}'
        raise AssertionError(f'timed out: diagnostics={diagnostics[-1:]!r}, publishers={listener.count_publishers("/diagnostics")}, odom={len(odom)}')

    def values(message):
        return {kv.key: kv.value for kv in message.status[0].values}

    def stop(proc):
        proc.send_signal(signal.SIGINT)
        proc.wait(timeout=5)
        processes.remove(proc)
        assert proc.returncode == 0, proc.returncode

    with tempfile.TemporaryDirectory(dir=scratch) as folder:
        path = Path(folder)
        log = open(path / 'bridge.log', 'w+')
        link = path / 'serial'

        def spawn(tf):
            proc = subprocess.Popen([binary, '--ros-args', '-p', f'port:={link}',
                                     '-p', f'publish_tf:={str(tf).lower()}',
                                     '-p', 'diagnostics_period_s:=0.1',
                                     '-p', 'reconnect_period_s:=0.2'], stdout=log, stderr=log)
            processes.append(proc)
            return proc

        def pty_link():
            master, slave = os.openpty()
            descriptors.extend([master, slave])
            if link.is_symlink():
                link.unlink()
            link.symlink_to(os.ttyname(slave))
            return master, slave

        try:
            # Missing port: node must stay alive and report ERROR, then reconnect.
            proc = spawn(False)
            wait_for(lambda: diagnostics and diagnostics[-1].status[0].level == DiagnosticStatus.ERROR)
            master, slave = pty_link()
            wait_for(lambda: listener.count_publishers('/odom') > 0)
            # Diagnostics after opening becomes 'no recent frame'.
            wait_for(lambda: diagnostics[-1].status[0].message == 'no recent CHASSIS_STATE frame')
            time.sleep(.1)  # Let DDS discover the encoder subscription too.
            packets = [(1000, (0, 0, 0, 0)), (1100, (100, -100, -100, 100)),
                       (1200, (200, 0, 0, 200)), (1300, (300, 100, -100, 100))]
            # Build healthy telemetry with independently selected fault/safety fields.
            def healthy(sequence, stamp, enc, fault=0, safety=0):
                import struct
                payload = struct.pack('<I4i4bHHBBH', stamp, *enc, -1, 0, 1, 2,
                                      12000, fault, safety, 0, sequence)
                raw = struct.pack('<BBBBHHI', 1, 0x81, 0, 12, sequence, 32, stamp) + payload
                return FIXTURES.cobs(raw + struct.pack('<I', FIXTURES.crc32c(raw)))
            batch = b''.join(healthy(i, stamp, enc) for i, (stamp, enc) in enumerate(packets))
            os.write(master, batch[:17])
            os.write(master, batch[17:])
            wait_for(lambda: len(odom) >= 4 and len(counts) >= 4)
            tick = 2 * math.pi * .048 / 5740.25304043815
            assert math.isclose(odom[1].twist.twist.linear.x, 1000 * tick, rel_tol=1e-9)
            assert math.isclose(odom[2].twist.twist.linear.y, 1000 * tick, rel_tol=1e-9)
            assert math.isclose(odom[3].twist.twist.angular.z, 400 * tick / (2 * (.190 + .205)) / .1, rel_tol=1e-9)
            assert list(counts[-1].data) == list(packets[-1][1])
            for msg in odom:
                assert msg.header.frame_id == 'odom' and msg.child_frame_id == 'base_link'
                assert msg.header.stamp.sec > 0
                assert msg.twist.covariance[0] == .0025 and msg.twist.covariance[7] == .01
                assert msg.twist.covariance[35] == .04
            wait_for(lambda: diagnostics[-1].status[0].level == DiagnosticStatus.OK)
            expected_keys = {'firmware_timestamp_ms', 'sample_sequence', 'receive_rate_hz',
                             'frame_age_s', 'fault_flags', 'safety_state', 'frames_valid',
                             'frames_dropped', 'crc_errors', 'sequence_drops', 'serial_port',
                             'baud_rate', 'bytes_received', 'cobs_errors', 'format_errors',
                             'overflow_errors', 'duplicate_frames', 'battery_mv', 'motor_commands'}
            assert expected_keys <= values(diagnostics[-1]).keys()
            assert values(diagnostics[-1])['motor_commands'] == '-1,0,1,2'
            assert not transforms, 'raw TF must be disabled for EKF'
            bad = bytearray(healthy(4, 1400, packets[-1][1])); bad[-3] ^= 1
            os.write(master, bad + b'\xff' * 300 + b'\x00')
            os.write(master, healthy(5, 1500, packets[-1][1], fault=16))
            wait_for(lambda: diagnostics[-1].status[0].message == 'STM32 reports a controller fault')
            assert int(values(diagnostics[-1])['crc_errors']) == 1
            assert int(values(diagnostics[-1])['overflow_errors']) == 1
            os.write(master, healthy(0, 10, (0, 0, 0, 0)))
            wait_for(lambda: len(odom) >= 6)
            assert odom[-1].pose.pose.position.x == 0 and odom[-1].twist.twist.linear.x == 0
            wait_for(lambda: diagnostics[-1].status[0].message == 'no recent CHASSIS_STATE frame', 2)
            # Disconnect with an unfinished frame, replace device, and reconnect.
            os.write(master, b'\x03\x01')
            os.close(master); descriptors.remove(master)
            os.close(slave); descriptors.remove(slave)
            wait_for(lambda: 'disconnected' in diagnostics[-1].status[0].message or
                     'open serial' in diagnostics[-1].status[0].message, 3)
            master, slave = pty_link()
            wait_for(lambda: diagnostics[-1].status[0].message == 'no recent CHASSIS_STATE frame')
            previous = len(odom)
            os.write(master, healthy(1, 110, (100, -100, -100, 100)))
            wait_for(lambda: len(odom) > previous)
            wait_for(lambda: diagnostics[-1].status[0].level == DiagnosticStatus.OK)
            assert int(values(diagnostics[-1])['crc_errors']) == 1, 'stats lost on reconnect'
            stop(proc)
            # Test standalone TF in a fresh process, reusing the PTY after close.
            # TIOCEXCL releases only when all slave descriptors close.
            os.close(slave); descriptors.remove(slave)
            os.close(master); descriptors.remove(master)
            master, slave = pty_link()
            transforms.clear(); diagnostics.clear()
            proc = spawn(True)
            wait_for(lambda: diagnostics and diagnostics[-1].status[0].message == 'no recent CHASSIS_STATE frame')
            os.write(master, healthy(0, 1000, (0, 0, 0, 0)))
            wait_for(lambda: transforms)
            assert transforms[-1].transforms[0].child_frame_id == 'base_link'
            stop(proc)
            invalid = subprocess.run([binary, '--ros-args', '-p', 'wheel_radius_m:=0.0'],
                                     stdout=log, stderr=log, timeout=5)
            assert invalid.returncode != 0, 'invalid geometry accepted'
            print('PASS: real ROS C++ bridge, PTY, firmware dt/reset, diagnostics, faults, stale, reconnect, TF')
        except Exception:
            log.flush(); log.seek(0); print(log.read(), file=sys.stderr)
            raise
        finally:
            for proc in processes:
                proc.send_signal(signal.SIGINT)
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill(); proc.wait(timeout=5)
            for descriptor in descriptors:
                os.close(descriptor)
            log.close()
            listener.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()

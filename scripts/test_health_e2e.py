"""Manual cross-workspace smoke test: bridge + ROS-only monitor, no hardware."""
import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import rclpy
from nav_msgs.msg import Odometry

ROOT = Path(__file__).resolve().parents[1]
path = ROOT / 'pi_ws/src/learm_bridge_cpp/test/test_ros_health.py'
spec = importlib.util.spec_from_file_location('health_frames', path)
frames = importlib.util.module_from_spec(spec)
spec.loader.exec_module(frames)


def main():
    bridge_binary, monitor_binary = [str(Path(p).resolve()) for p in sys.argv[1:3]]
    scratch = ROOT / 'pi_ws/build/health_e2e'
    scratch.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    listener = rclpy.create_node('health_e2e_test')
    odometry = []
    subscription = listener.create_subscription(Odometry, '/odom', odometry.append, 100)
    master, slave = os.openpty()
    processes = []
    with tempfile.TemporaryDirectory(dir=scratch) as folder:
        bridge_log = open(Path(folder) / 'bridge.log', 'w+')
        monitor_log = open(Path(folder) / 'monitor.log', 'w+')
        try:
            bridge = subprocess.Popen([bridge_binary, '--ros-args', '-p', f'port:={os.ttyname(slave)}',
                                       '-p', 'publish_tf:=false', '-p', 'diagnostics_period_s:=0.1'],
                                      stdout=bridge_log, stderr=bridge_log)
            monitor = subprocess.Popen([monitor_binary], stdout=monitor_log, stderr=monitor_log)
            processes.extend([bridge, monitor])
            # Send progressing health and chassis together after DDS discovery.
            deadline = time.monotonic() + 4
            sequence = 0
            while time.monotonic() < deadline:
                stamp = 1000 + sequence * 50
                os.write(master, frames.packet(stamp) + frames.fixtures.frame(sequence, stamp, (0,0,0,0)))
                rclpy.spin_once(listener, timeout_sec=.05)
                time.sleep(.03)
                sequence += 1
            assert odometry, 'odometry did not publish while health monitoring ran'
            monitor_log.flush(); monitor_log.seek(0)
            assert 'Link ONLINE' in monitor_log.read(), 'monitor never showed ONLINE'
            # No serial/health updates: the display must independently expire.
            deadline = time.monotonic() + 3.2
            while time.monotonic() < deadline:
                rclpy.spin_once(listener, timeout_sec=.05)
            monitor_log.flush(); monitor_log.seek(0)
            assert 'Link OFFLINE' in monitor_log.read(), 'monitor retained a fresh link after silence'
            monitor.send_signal(signal.SIGINT); monitor.wait(timeout=5)
            assert monitor.returncode == 0
            processes.remove(monitor)
            before = len(odometry)
            os.write(master, frames.packet(10000) + frames.fixtures.frame(sequence, 10000, (0,0,0,0)))
            deadline = time.monotonic() + 2
            while len(odometry) == before and time.monotonic() < deadline:
                rclpy.spin_once(listener, timeout_sec=.05)
            assert bridge.poll() is None and len(odometry) > before, 'closing monitor affected bridge'
            print('PASS: concurrent odometry/health display, silence OFFLINE, monitor exit leaves bridge running')
        except Exception:
            for log in (bridge_log, monitor_log):
                log.flush(); log.seek(0); print(log.read(), file=sys.stderr)
            raise
        finally:
            for proc in processes:
                if proc.poll() is None:
                    proc.send_signal(signal.SIGINT)
                    try: proc.wait(timeout=5)
                    except subprocess.TimeoutExpired: proc.kill(); proc.wait(timeout=5)
            bridge_log.close(); monitor_log.close()
            os.close(master); os.close(slave)
            listener.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()

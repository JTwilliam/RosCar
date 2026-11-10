"""Offline orchestration checks: no ROS nodes, GUI, serial or SSH are started."""
import importlib.util
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

import yaml
from launch import LaunchContext
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('pi_system', ROOT / 'scripts/pi_system.launch.py')
SYSTEM = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SYSTEM)


class Launchers(unittest.TestCase):
    def context(self, **overrides):
        context = LaunchContext()
        context.launch_configurations.update({
            'enable_lidar': 'true', 'external_mount_tf': 'false', 'bridge': 'python',
            'imu_mount': '0.1 0.2 0.3 0 0 1.57',
            'lidar_mount': '0 0 0.18 0 0 0',
            'port': '/dev/test-stm32', 'baud': '115200', 'bus': '1', 'address': '104',
            **overrides,
        })
        return context

    def test_full_system_is_pi_only(self):
        with patch.object(SYSTEM, 'get_package_share_directory', return_value='/mock/share'):
            actions = SYSTEM.system(self.context())
        self.assertEqual(sum(isinstance(a, Node) for a in actions), 2)
        self.assertEqual(sum(isinstance(a, IncludeLaunchDescription) for a in actions), 3)

    def test_external_tf_and_no_lidar(self):
        with patch.object(SYSTEM, 'get_package_share_directory', return_value='/mock/share'):
            actions = SYSTEM.system(self.context(external_mount_tf='true', enable_lidar='false',
                                                  imu_mount='', lidar_mount=''))
        self.assertEqual(len(actions), 2)
        self.assertTrue(all(isinstance(a, IncludeLaunchDescription) for a in actions))

    def test_cpp_bridge_selects_cpp_package(self):
        with patch.object(SYSTEM, 'get_package_share_directory', return_value='/mock/share') as lookup:
            SYSTEM.system(self.context(bridge='cpp'))
        self.assertIn(('learm_bridge_cpp',), [call.args for call in lookup.call_args_list])
        with self.assertRaises(ValueError):
            SYSTEM.system(self.context(bridge='invalid'))

    def test_invalid_mounts_fail_before_package_lookup(self):
        for mount in ('', '0 0 0', '0 0 0 0 0 nan', '0 0 0 0 0 inf'):
            with self.subTest(mount=mount), patch.object(SYSTEM, 'get_package_share_directory') as lookup:
                with self.assertRaises(ValueError):
                    SYSTEM.system(self.context(imu_mount=mount))
                lookup.assert_not_called()

    def test_shell_rejects_missing_or_bad_config_without_starting_ros(self):
        scratch = ROOT / 'pi_ws/build/script_checks'
        scratch.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(dir=scratch) as folder:
            config = Path(folder) / 'robot.env'
            config.write_text('IMU_MOUNT=""\n', encoding='utf-8')
            env = {**os.environ, 'ROSCAR_CONFIG': str(config)}
            result = subprocess.run(['bash', str(ROOT / 'scripts/raspberry_pi.sh'), 'all', '--dry-run'],
                                    env=env, capture_output=True, text=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('IMU_MOUNT requires six values', result.stderr)
            result = subprocess.run(['bash', str(ROOT / 'scripts/desktop.sh'), 'wrong'],
                                    env=env, capture_output=True, text=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('Unknown mode', result.stderr)

    def test_rviz_topics_and_qos(self):
        data = yaml.safe_load((ROOT / 'config/desktop.rviz').read_text())['Visualization Manager']
        self.assertEqual(data['Global Options']['Fixed Frame'], 'odom')
        displays = {d['Name']: d for d in data['Displays']}
        self.assertEqual(displays['Laser Scan']['Topic']['Reliability Policy'], 'Best Effort')
        self.assertEqual(displays['Laser Scan']['Topic']['Value'], '/scan')
        self.assertEqual(displays['Filtered Odometry']['Topic']['Value'], '/odometry/filtered')
        self.assertEqual(displays['Wheel Odometry']['Topic']['Value'], '/odom')


if __name__ == '__main__':
    unittest.main(verbosity=2)

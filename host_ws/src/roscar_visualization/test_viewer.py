"""Display logic regression, without starting a GUI or accessing hardware."""
import importlib.util
from pathlib import Path
import unittest
from sensor_msgs.msg import Imu

spec = importlib.util.spec_from_file_location('host_viewer', Path(__file__).parent / 'scripts/imu_viewer.py')
viewer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(viewer)


class ViewerTest(unittest.TestCase):
    def test_normalization_and_reference_frame(self):
        message = Imu()
        message.header.frame_id = 'imu_link'
        message.orientation.z = 2.0
        message.orientation.w = 2.0
        pose, markers = viewer.make_visuals(message)
        self.assertEqual(pose.header.frame_id, 'imu_view')
        self.assertAlmostEqual(pose.pose.orientation.z, 2 ** -.5)
        self.assertEqual(len(markers.markers), 4)
        self.assertEqual(message.header.frame_id, 'imu_link')

    def test_invalid_orientation_rejected(self):
        message = Imu()
        message.orientation.w = 0.0
        with self.assertRaises(ValueError):
            viewer.make_visuals(message)
        message.orientation.w = 1.0
        message.orientation_covariance[0] = -1
        with self.assertRaises(ValueError):
            viewer.make_visuals(message)


if __name__ == '__main__':
    unittest.main()

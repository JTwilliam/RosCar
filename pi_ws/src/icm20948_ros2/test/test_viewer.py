import math

import pytest
from sensor_msgs.msg import Imu

from icm20948_ros2.viewer import make_visuals


def test_visualization_preserves_attitude_and_uses_independent_frame():
    message = Imu()
    message.header.frame_id = 'imu_link'
    message.header.stamp.sec = 123
    message.orientation.z = math.sqrt(2)
    message.orientation.w = math.sqrt(2)  # Deliberately non-unit
    pose, markers = make_visuals(message)
    assert pose.header.frame_id == 'imu_view'
    assert pose.header.stamp.sec == 123
    assert pose.pose.orientation.z == pytest.approx(math.sqrt(.5))
    assert pose.pose.orientation.w == pytest.approx(math.sqrt(.5))
    assert len(markers.markers) == 4
    for marker in markers.markers:
        assert marker.pose.orientation == pose.pose.orientation
        assert marker.lifetime.nanosec > 0
    assert markers.markers[1].points[1].x > 0
    assert markers.markers[2].points[1].y > 0
    assert markers.markers[3].points[1].z > 0
    assert message.header.frame_id == 'imu_link'


def test_visualization_rejects_missing_invalid_and_raw_orientation():
    message = Imu()
    message.orientation.w = 0.0
    with pytest.raises(ValueError, match='Zero'):
        make_visuals(message)
    message.orientation.w = float('nan')
    with pytest.raises(ValueError, match='Non-finite'):
        make_visuals(message)
    message.orientation.w = 1.0
    message.orientation_covariance[0] = -1
    with pytest.raises(ValueError, match='no orientation'):
        make_visuals(message)

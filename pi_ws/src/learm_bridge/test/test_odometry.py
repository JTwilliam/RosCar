import math

import pytest

from learm_bridge.odometry import MecanumOdometry


def test_first_sample_only_establishes_encoder_baseline():
    odometry = MecanumOdometry()

    estimate = odometry.update((100, 200, 300, 400), timestamp_s=1.0)

    assert estimate.x_m == 0.0
    assert estimate.y_m == 0.0
    assert estimate.yaw_rad == 0.0


def test_forward_motion_uses_armrobot_encoder_order_and_signs():
    odometry = MecanumOdometry()
    odometry.update((0, 0, 0, 0), timestamp_s=1.0)

    estimate = odometry.update((100, -100, -100, 100), timestamp_s=1.1)

    expected_distance = 100.0 * odometry.meters_per_tick
    assert math.isclose(estimate.x_m, expected_distance, rel_tol=1e-9)
    assert math.isclose(estimate.y_m, 0.0, abs_tol=1e-12)
    assert math.isclose(estimate.vx_m_s, expected_distance / 0.1, rel_tol=1e-9)


def test_leftward_motion_has_positive_ros_y():
    odometry = MecanumOdometry()
    odometry.update((0, 0, 0, 0), timestamp_s=1.0)

    estimate = odometry.update((100, 100, 100, 100), timestamp_s=1.1)

    expected_distance = 100.0 * odometry.meters_per_tick
    assert math.isclose(estimate.y_m, expected_distance, rel_tol=1e-9)
    assert math.isclose(estimate.x_m, 0.0, abs_tol=1e-12)
    assert math.isclose(estimate.yaw_rad, 0.0, abs_tol=1e-12)


def test_counter_clockwise_rotation_has_positive_yaw():
    odometry = MecanumOdometry()
    odometry.update((0, 0, 0, 0), timestamp_s=1.0)

    estimate = odometry.update((100, 100, -100, -100), timestamp_s=1.1)

    assert estimate.yaw_rad > 0.0
    assert math.isclose(estimate.x_m, 0.0, abs_tol=1e-12)
    assert math.isclose(estimate.y_m, 0.0, abs_tol=1e-12)


def test_nonpositive_geometry_is_rejected():
    with pytest.raises(ValueError):
        MecanumOdometry(track_width_m=0.0)

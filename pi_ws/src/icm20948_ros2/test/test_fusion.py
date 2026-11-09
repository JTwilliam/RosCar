import math

import pytest

from icm20948_ros2.fusion import (
    Mahony6D, MeasurementFilter, StationaryCalibration, euler_quaternion, norm,
)
from icm20948_ros2.sensor import GRAVITY


def test_level_initialization_and_stationary_normalization():
    fusion = Mahony6D(integral_gain=0.0)
    for _ in range(1000):
        q = fusion.update((0, 0, GRAVITY), (0, 0, 0), 0.01)
    assert q == pytest.approx((0, 0, 0, 1))
    assert norm(q) == pytest.approx(1)


def test_roll_pitch_initialization_convention():
    roll, pitch = 0.3, -0.2
    acceleration = (
        -GRAVITY * math.sin(pitch),
        GRAVITY * math.sin(roll) * math.cos(pitch),
        GRAVITY * math.cos(roll) * math.cos(pitch),
    )
    assert Mahony6D().update(acceleration, (0, 0, 0), .01) == pytest.approx(
        euler_quaternion(roll, pitch, 0))


def test_z_rotation_is_positive_and_yaw_remains_unobservable():
    fusion = Mahony6D()
    fusion.update((0, 0, GRAVITY), (0, 0, 0), .01)
    for _ in range(100):
        q = fusion.update((0, 0, GRAVITY), (0, 0, math.pi / 2), .01)
    assert q == pytest.approx(euler_quaternion(0, 0, math.pi / 2), abs=2e-5)
    for _ in range(100):
        # Gravity cannot pull a nonzero yaw back to zero.
        q = fusion.update((0, 0, GRAVITY), (0, 0, 0), .01)
    assert q == pytest.approx(euler_quaternion(0, 0, math.pi / 2), abs=2e-5)


def test_acceleration_correction_converges_with_correct_sign():
    fusion = Mahony6D(integral_gain=0.0)
    fusion.q = euler_quaternion(0.4, -0.3, 0)
    for _ in range(1500):
        q = fusion.update((0, 0, GRAVITY), (0, 0, 0), .01)
    # Yaw is unobservable; require level roll/pitch, not a particular yaw.
    assert abs(q[0]) < 1e-6
    assert abs(q[1]) < 1e-6


def test_dynamic_acceleration_and_long_gap_are_not_integrated_as_tilt():
    fusion = Mahony6D()
    assert fusion.update((0, 0, 0), (0, 0, 0), .01) is None
    initial = fusion.update((0, 0, GRAVITY), (0, 0, 0), .01)
    assert fusion.update((GRAVITY, 0, GRAVITY), (0, 0, 0), .01) == pytest.approx(initial)
    assert fusion.update((0, 0, GRAVITY), (0, 0, 10), .5) is None
    assert fusion.q == initial


def test_stationary_calibration_does_not_subtract_gravity():
    calibration = StationaryCalibration(20)
    for _ in range(19):
        assert calibration.add((0, 0, GRAVITY), (.01, -.02, .03)) is None
    assert calibration.add((0, 0, GRAVITY), (.01, -.02, .03)) == pytest.approx((.01, -.02, .03))


def test_moving_calibration_window_is_rejected_and_can_retry():
    calibration = StationaryCalibration(20)
    with pytest.raises(ValueError, match='Motion'):
        for i in range(20):
            calibration.add((0, 0, GRAVITY), (.05 * (-1) ** i, 0, 0))
    assert not calibration.samples
    result = None
    for _ in range(20):
        result = calibration.add((0, 0, GRAVITY), (.01, 0, 0))
    assert result == pytest.approx((.01, 0, 0))


def test_lpf_matches_reference_and_is_independent_of_poll_rate():
    slow, fast = MeasurementFilter(), MeasurementFilter()
    for f in (slow, fast):
        f.update((0, 0, GRAVITY), (0, 0, 0), .02)
    for _ in range(10):
        a1, g1 = slow.update((1, 0, GRAVITY), (0, 0, 1), .02)
    for _ in range(20):
        a2, g2 = fast.update((1, 0, GRAVITY), (0, 0, 1), .01)
    assert a1 == pytest.approx(a2)
    assert g1 == pytest.approx(g2)
    assert g1[2] == pytest.approx(1 - .8 ** 10)
    assert a1[0] == pytest.approx(1 - .7 ** 10)


def test_lpf_reduces_alternating_noise_but_preserves_slow_rotation():
    f = MeasurementFilter()
    values = []
    for i in range(300):
        _, gyro = f.update((0, 0, GRAVITY), ((-1) ** i, 0, .005), .01)
        if i > 200:
            values.append(abs(gyro[0]))
    assert max(values) < .1
    assert gyro[2] == pytest.approx(.005)
    optional = MeasurementFilter(gyro_z_deadband=.015)
    assert optional.update((0, 0, GRAVITY), (0, 0, .005), .01)[1][2] == 0


def test_integral_is_bounded_and_does_not_correct_z_gyro():
    f = Mahony6D(integral_gain=.0001)
    f.q = euler_quaternion(.5, 0, 0)
    for _ in range(1000):
        f.update((0, 0, GRAVITY), (0, 0, 0), .01)
    assert all(abs(v) <= 2 for v in f.integral)
    assert f.q[2] == pytest.approx(0)
    assert f.update((0, 0, GRAVITY), (0, 0, 0), .3) is None
    assert f.integral == [0, 0]

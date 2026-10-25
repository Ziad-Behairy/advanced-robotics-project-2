import numpy as np
import pytest

from rm_flatness_nav.geometry import quat_to_rot, range_bearing, transform_point


def test_bearing_example_from_note():
    # tag 1.5 m ahead and 0.3 m to the right: REP-103 right is -y, so the bearing is negative
    d, theta = range_bearing([1.5, -0.3, 0.0])
    assert theta == pytest.approx(-0.197, abs=1e-3)
    assert abs(theta) > 0.18          # outside the +-10 deg window: keep aligning
    assert d == pytest.approx(np.hypot(1.5, 0.3))


def test_identity_quaternion():
    assert np.allclose(quat_to_rot(0, 0, 0, 1), np.eye(3))


def test_tilted_optical_frame_to_base():
    # Optical frame (x right, y down, z forward) pitched down by 30 deg, 0.3 m above the base origin.
    pitch = np.radians(30)
    r_level = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])   # optical -> base, camera level
    r_pitch = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
    rot = r_pitch @ r_level
    # quaternion of rot, round trip through quat_to_rot
    w = np.sqrt(1 + np.trace(rot)) / 2
    q = [(rot[2, 1] - rot[1, 2]) / (4 * w), (rot[0, 2] - rot[2, 0]) / (4 * w), (rot[1, 0] - rot[0, 1]) / (4 * w), w]
    assert np.allclose(quat_to_rot(*q), rot)

    # A floor point 1.2 m ahead, 0.1 m left of the base, 0.3 m below the camera
    p_base_true = np.array([1.2, 0.1, 0.0])
    trans = np.array([0.0, 0.0, 0.3])
    p_cam = rot.T @ (p_base_true - trans)
    p_base = transform_point(quat_to_rot(*q), trans, p_cam)
    assert np.allclose(p_base, p_base_true)
    d, theta = range_bearing(p_base)
    assert theta > 0 and d == pytest.approx(np.hypot(1.2, 0.1))


def test_slant_range_overshoots_standoff():
    # At the 20 cm standoff, the camera's straight-line range |p_cam| includes the camera height,
    # so braking on it would stop late. The ground range from the base frame is what the brake needs.
    pitch = np.radians(25)
    r_level = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
    r_pitch = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
    rot, trans = r_pitch @ r_level, np.array([0.0, 0.0, 0.25])
    p_cam = rot.T @ (np.array([0.20, 0.0, 0.0]) - trans)
    assert np.linalg.norm(p_cam) == pytest.approx(0.32, abs=0.01)
    assert range_bearing(transform_point(rot, trans, p_cam))[0] == pytest.approx(0.20)


def test_camera_to_level_round_trip():
    from rm_flatness_nav.geometry import camera_to_level
    pitch = np.radians(6.3)
    # floor point 0.9 m ahead, 0.1 m left, 0.313 m below the camera, expressed in the pitched optical frame
    fwd, left, up = 0.9, 0.1, -0.313
    z = fwd * np.cos(pitch) - up * np.sin(pitch)
    y = -fwd * np.sin(pitch) - up * np.cos(pitch)
    x = -left
    assert np.allclose(camera_to_level([x, y, z], pitch), [fwd, left, up])

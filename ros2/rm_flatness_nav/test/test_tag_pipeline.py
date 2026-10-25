"""End-to-end check of detector + frame transform on a rendered floor tag.

The bitmap in data/ is T3 (tag36h11 ID 3) extracted from the printed sheet. It is rendered lying on the
floor, seen by the 720p calibrated pinhole camera pitched down, then detected with pupil-apriltags.
"""
from pathlib import Path

import cv2
import numpy as np
import pytest

pupil_apriltags = pytest.importorskip('pupil_apriltags')

from rm_flatness_nav.geometry import camera_from_floor_tag, range_bearing, transform_point  # noqa: E402

TAG = cv2.imread(str(Path(__file__).parent / 'data' / 'tag36h11_03.png'), cv2.IMREAD_GRAYSCALE)
SIZE = 0.155                                                          # measured black edge (m)
K = np.array([[628.58, 0, 638.02], [0, 625.79, 361.60], [0, 0, 1]])   # rm_camera_calibration_720


def camera_pose(pitch_deg, height):
    p = np.radians(pitch_deg)
    r_level = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])           # optical -> base, camera level
    r_pitch = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    return r_pitch @ r_level, np.array([0.0, 0.0, height])


def render_floor_tag(rot, trans, x, y, yaw=0.3):
    half = 0.5 * SIZE * 10 / 8                                        # outer edge incl. white border
    c, s = np.cos(yaw), np.sin(yaw)
    corners = [(-half, half), (half, half), (half, -half), (-half, -half)]  # image u -> +x, v -> -y
    pts = np.array([[x + c * u - s * v, y + s * u + c * v, 0.0] for u, v in corners])
    pc = (rot.T @ (pts - trans).T).T
    uv = (K @ pc.T).T
    uv = (uv[:, :2] / uv[:, 2:]).astype(np.float32)
    big = cv2.resize(TAG, (1000, 1000), interpolation=cv2.INTER_NEAREST)
    hm = cv2.getPerspectiveTransform(np.float32([[0, 0], [999, 0], [999, 999], [0, 999]]), uv)
    img = np.full((720, 1280), 170, np.uint8)
    warped = cv2.warpPerspective(big, hm, (1280, 720), flags=cv2.INTER_AREA)
    mask = cv2.warpPerspective(np.full_like(big, 255), hm, (1280, 720))
    img[mask > 0] = warped[mask > 0]
    return cv2.GaussianBlur(img, (3, 3), 0.8)


@pytest.mark.parametrize('x, y', [(0.4, 0.0), (0.6, 0.15), (0.9, -0.2), (1.1, 0.0)])
def test_range_and_bearing(x, y):
    rot, trans = camera_pose(25, 0.25)
    det = pupil_apriltags.Detector(families='tag36h11', quad_decimate=1.0)
    hits = [d for d in det.detect(render_floor_tag(rot, trans, x, y), True,
                                  (K[0, 0], K[1, 1], K[0, 2], K[1, 2]), SIZE) if d.tag_id == 3]
    assert hits, 'T3 not detected'
    d, theta = range_bearing(transform_point(rot, trans, hits[0].pose_t.ravel()))
    assert d == pytest.approx(np.hypot(x, y), rel=0.02)
    assert theta == pytest.approx(np.arctan2(y, x), abs=0.01)


@pytest.mark.parametrize('pitch, height', [(10, 0.26), (25, 0.25), (0, 0.27)])
def test_camera_height_and_pitch_from_floor_tag(pitch, height):
    rot, trans = camera_pose(pitch, height)
    det = pupil_apriltags.Detector(families='tag36h11', quad_decimate=1.0)
    hits = [d for d in det.detect(render_floor_tag(rot, trans, 0.7, 0.05), True,
                                  (K[0, 0], K[1, 1], K[0, 2], K[1, 2]), SIZE) if d.tag_id == 3]
    assert hits, 'T3 not detected'
    h, pitch_down, roll = camera_from_floor_tag(hits[0].pose_R, hits[0].pose_t)
    assert h == pytest.approx(height, abs=0.01)
    assert np.degrees(pitch_down) == pytest.approx(pitch, abs=1.5)
    assert abs(np.degrees(roll)) < 1.5

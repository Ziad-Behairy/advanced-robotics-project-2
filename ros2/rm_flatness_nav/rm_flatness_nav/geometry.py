"""Frame helpers for the tag detector: camera optical frame -> robot base frame -> ground range and bearing."""
import numpy as np


def quat_to_rot(x, y, z, w):
    """Rotation matrix of a unit quaternion (ROS order x, y, z, w)."""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def transform_point(rot, trans, p):
    """p expressed in the parent frame, given the child->parent rotation and translation."""
    return rot @ np.asarray(p, dtype=float) + np.asarray(trans, dtype=float)


def range_bearing(p_base):
    """Horizontal distance d and bearing theta (REP-103: + = left) of a point in the base frame."""
    x, y = p_base[0], p_base[1]
    return float(np.hypot(x, y)), float(np.arctan2(y, x))


def camera_from_floor_tag(tag_rot, tag_t):
    """Camera height above the floor, downward pitch and roll, from one tag lying flat on the floor.

    tag_rot, tag_t: tag pose in the camera optical frame (x right, y down, z forward), as pupil-apriltags
    returns it. The tag normal is the floor normal, so this needs no arm model or TF.
    """
    n = np.asarray(tag_rot, dtype=float)[:, 2]
    t = np.asarray(tag_t, dtype=float).ravel()
    if n @ t > 0:          # make n point from the floor up toward the camera
        n = -n
    height = float(-n @ t)
    pitch_down = float(np.arcsin(-n[2]))        # optical axis angle below the horizon
    roll = float(np.arctan2(n[0], -n[1]))       # 0 when the image x axis is level
    return height, pitch_down, roll


def camera_to_level(p_cam, pitch_down):
    """Point from the optical frame (x right, y down, z forward) of a camera pitched down by pitch_down (rad)
    to a level frame at the camera: (forward, left, up)."""
    x, y, z = (float(v) for v in np.asarray(p_cam).ravel())
    c, s = np.cos(pitch_down), np.sin(pitch_down)
    return z * c - y * s, -x, -y * c - z * s

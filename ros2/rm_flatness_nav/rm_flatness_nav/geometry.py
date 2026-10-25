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

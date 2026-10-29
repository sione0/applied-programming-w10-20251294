"""Provided two-joint slice rule, not physical robot collision geometry."""

import numpy as np

PLANNING_BOUNDS = ((-0.8, 0.8), (-1.2, -0.35))
FORBIDDEN_RECTANGLE = ((-0.20, 0.20), (-0.85, -0.65))
FIXED_JOINTS_RAD = (0.0, -2.10, 0.0, 1.60, 0.7854)


def point_array(value):
    """Return an owned finite (q1,q2) array, in radians."""
    try:
        point = np.array(value, dtype=np.float64, copy=True)
    except (TypeError, ValueError) as error:
        raise ValueError('joint state must contain two finite numbers') from error
    if point.shape != (2,) or not np.isfinite(point).all():
        raise ValueError('joint state must have finite shape (2,)')
    return point


def slice_edge_is_allowed(left, right):
    """Check a whole straight segment against bounds and a closed rectangle.

    The conservative 1e-12 segment-parameter tolerance preserves corner
    touches despite floating-point rounding. This is the Week 9 software
    constraint. It does not test the arm's physical geometry.
    """
    left, right = point_array(left), point_array(right)
    for point in (left, right):
        if any(not low <= value <= high
               for value, (low, high) in zip(point, PLANNING_BOUNDS)):
            return False
    enter, leave = 0.0, 1.0
    for start, finish, (low, high) in zip(left, right, FORBIDDEN_RECTANGLE):
        delta = finish - start
        if delta == 0.0:
            if start < low or start > high:
                return True
            continue
        near, far = sorted(((low - start) / delta, (high - start) / delta))
        enter, leave = max(enter, near), min(leave, far)
        if enter > leave + 1e-12:
            return True
    return False


def slice_state_is_allowed(point):
    return slice_edge_is_allowed(point, point)


def validate_robot_path(path):
    """Return an owned (N,2) path with permitted states and segments."""
    try:
        points = np.array(path, dtype=np.float64, copy=True)
    except (TypeError, ValueError) as error:
        raise ValueError('robot path must be a finite (N,2) array') from error
    if points.ndim != 2 or points.shape[1] != 2 or not 1 <= len(points) <= 64:
        raise ValueError('provided GPU runner accepts 1..64 two-joint waypoints')
    if not np.isfinite(points).all():
        raise ValueError('robot path must contain finite values')
    if any(not slice_state_is_allowed(point) for point in points):
        raise ValueError('robot path violates the declared joint-state rule')
    if any(not slice_edge_is_allowed(a, b) for a, b in zip(points, points[1:])):
        raise ValueError('robot path violates the declared whole-edge rule')
    return points


def full_joint_targets(points):
    points = validate_robot_path(points)
    fixed = np.tile(FIXED_JOINTS_RAD, (len(points), 1))
    return np.column_stack((points, fixed))


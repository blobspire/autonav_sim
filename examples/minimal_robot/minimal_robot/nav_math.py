"""Pure, ROS-free math for the minimal_navigator.

Extracted from minimal_navigator.py (which imports rclpy and so can't be imported
on the host) so these small, load-bearing functions are unit-testable in the fast
host gate.
"""
import math


def yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    """Yaw (rotation about +Z) from a quaternion (x, y, z, w)."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def lookahead_point(prev_point: tuple[float, float],
                    target: tuple[float, float],
                    x: float, y: float,
                    lookahead: float) -> tuple[float, float]:
    """A point `lookahead` metres ahead of the robot's projection onto the path
    segment `prev_point` -> `target`. Following this tracks the LINE rather than
    cutting the corner toward `target`. Degenerate (zero-length) segment -> the
    target itself."""
    px, py = prev_point
    tx, ty = target
    seg_x, seg_y = tx - px, ty - py
    seg_len = math.hypot(seg_x, seg_y)
    if seg_len < 1e-6:
        return tx, ty
    ux, uy = seg_x / seg_len, seg_y / seg_len        # unit vector along segment
    along = (x - px) * ux + (y - py) * uy            # projection onto segment
    look = min(seg_len, max(0.0, along) + lookahead)
    return px + ux * look, py + uy * look

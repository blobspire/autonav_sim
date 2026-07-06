"""Host tests for the bundled minimal robot (examples/minimal_robot): the profile
loads with a full geometry block, and the navigator's ROS-free math is correct.

These guard the two headline "plug your own robot in" artifacts on the fast host
gate (previously only verified at VM launch)."""
import math
import sys
from dataclasses import fields
from pathlib import Path

from igvc_competition_sim.robot_profile import load_robot_profile

REPO = Path(__file__).resolve().parents[2]
MINIMAL = REPO / "examples" / "minimal_robot"
MINIMAL_PROFILE = MINIMAL / "config" / "minimal_profile.yaml"

# Make the (ROS-free) minimal_robot.nav_math importable without pulling in the
# navigator node's rclpy import.
sys.path.insert(0, str(MINIMAL))
from minimal_robot import nav_math  # noqa: E402


def test_minimal_profile_loads_with_full_geometry():
    profile = load_robot_profile(MINIMAL_PROFILE)
    assert profile.name == "minibot"
    assert profile.description_ref.startswith("package://minimal_robot/")
    assert profile.geometry is not None
    # RobotSpec is a frozen dataclass with no defaults, so a missing / typo'd /
    # extra geometry key raises at load — this also guards the 17-field contract.
    assert len(fields(profile.geometry)) == 17
    assert profile.geometry.wheel_track_m == 0.44
    assert profile.geometry.wheel_radius_m == 0.10


def test_nav_math_yaw_from_quaternion():
    assert abs(nav_math.yaw_from_quaternion(0, 0, 0, 1)) < 1e-9  # identity -> 0
    s = math.sin(math.pi / 4)
    c = math.cos(math.pi / 4)
    assert abs(nav_math.yaw_from_quaternion(0, 0, s, c) - math.pi / 2) < 1e-6


def test_nav_math_normalize_angle():
    assert abs(nav_math.normalize_angle(1.5 * math.pi) - (-0.5 * math.pi)) < 1e-9
    assert abs(nav_math.normalize_angle(-1.5 * math.pi) - (0.5 * math.pi)) < 1e-9
    assert abs(nav_math.normalize_angle(0.3) - 0.3) < 1e-9


def test_nav_math_lookahead_tracks_the_line_not_the_corner():
    # Segment (0,0)->(10,0); robot at (2,1) off the line; lookahead 1.5 m.
    lax, lay = nav_math.lookahead_point((0.0, 0.0), (10.0, 0.0), 2.0, 1.0, 1.5)
    assert abs(lay) < 1e-9          # the lookahead point is ON the line (y=0)
    assert abs(lax - 3.5) < 1e-9    # projection (x=2) + 1.5 lookahead
    # Clamps at the segment end.
    lax2, _ = nav_math.lookahead_point((0.0, 0.0), (10.0, 0.0), 9.5, 0.0, 1.5)
    assert abs(lax2 - 10.0) < 1e-9
    # Degenerate zero-length segment -> the target itself (no div-by-zero).
    assert nav_math.lookahead_point((5.0, 5.0), (5.0, 5.0), 0.0, 0.0, 1.0) == (5.0, 5.0)

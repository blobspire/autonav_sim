from pathlib import Path

from igvc_competition_sim.course import load_course
from igvc_competition_sim.generate_world import generate_world
from igvc_competition_sim.robot_profile import RobotProfile, load_robot_profile

PKG = Path(__file__).resolve().parents[1]


def _normalize(text: str) -> str:
    return "\n".join(line.rstrip() for line in text.splitlines()) + "\n"


def _committed_world() -> str:
    return (PKG / "worlds" / "igvc_competition_compact.sdf").read_text(
        encoding="utf-8")


def _generate_default() -> str:
    course = load_course(PKG / "config" / "igvc_competition_compact.yaml")
    profile = load_robot_profile(PKG / "profiles" / "shogi" / "profile.yaml")
    return _normalize(generate_world(course, profile))


def test_default_profile_reproduces_committed_world():
    # Non-circular byte-identity oracle: freshly generated output must equal the
    # INDEPENDENT committed course-only SDF on disk (never compared to itself).
    assert _generate_default() == _committed_world()


def test_committed_world_is_robot_agnostic():
    # The robot now spawns separately (ros_gz_sim create at launch), so the
    # baked world must contain NO robot: no shogi model, no drive plugin, no
    # cmd_vel sink, no onboard camera sensor.
    world = _committed_world()
    assert "<model name='shogi'>" not in world
    assert "DiffDrive" not in world
    assert "/cmd_vel_gazebo" not in world
    assert "<sensor name='zed2i_rgbd'" not in world


def test_committed_world_keeps_the_course():
    # "course-only" must not silently degrade to "empty": course geometry and
    # world-level infrastructure the robot never owned must remain.
    world = _committed_world()
    assert "<model name='asphalt_ground'>" in world        # ground plane
    assert "<model name='center_barrel_20ft'>" in world    # obstacle (barrel)
    assert "<model name='legal_approach_ramp_up'>" in world  # ramp segment
    assert "ignition::gazebo::systems::Sensors" in world   # world Sensors plugin
    assert "<world name='igvc_competition'>" in world


def test_omitting_profile_generates_course_only():
    # The profile=None default path still works and yields a course-only world.
    course = load_course(PKG / "config" / "igvc_competition_compact.yaml")
    world = generate_world(course)  # no profile arg
    assert "<model name='shogi'>" not in world
    assert "DiffDrive" not in world
    assert "<model name='asphalt_ground'>" in world


def test_profile_no_longer_affects_world_output():
    # The world is robot-agnostic: swapping the robot profile must NOT change
    # the generated world (positive proof the robot left generate_world).
    course = load_course(PKG / "config" / "igvc_competition_compact.yaml")
    shogi = load_robot_profile(PKG / "profiles" / "shogi" / "profile.yaml")
    rover = RobotProfile(name="rover", geometry=shogi.geometry)
    world_shogi = generate_world(course, shogi)
    world_rover = generate_world(course, rover)
    assert world_shogi == world_rover
    assert "rover" not in world_rover
    assert "shogi" not in world_rover

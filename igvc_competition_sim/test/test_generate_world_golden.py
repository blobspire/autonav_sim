from pathlib import Path

from igvc_competition_sim.course import load_course
from igvc_competition_sim.generate_world import generate_world
from igvc_competition_sim.robot_profile import RobotProfile, load_robot_profile

PKG = Path(__file__).resolve().parents[1]


def _normalize(text: str) -> str:
    return "\n".join(line.rstrip() for line in text.splitlines()) + "\n"


def test_default_profile_reproduces_committed_world():
    course = load_course(PKG / "config" / "igvc_competition_compact.yaml")
    profile = load_robot_profile(PKG / "profiles" / "shogi" / "profile.yaml")
    generated = _normalize(generate_world(course, profile))
    committed = (PKG / "worlds" / "igvc_competition_compact.sdf").read_text(
        encoding="utf-8")
    assert generated == committed


def test_omitting_profile_uses_shogi_default():
    course = load_course(PKG / "config" / "igvc_competition_compact.yaml")
    world = generate_world(course)  # no profile arg
    assert "<model name='shogi'>" in world
    assert "<odom_topic>/model/shogi/odometry</odom_topic>" in world


def test_custom_profile_changes_model_name_and_topics():
    course = load_course(PKG / "config" / "igvc_competition_compact.yaml")
    shogi_profile = load_robot_profile(PKG / "profiles" / "shogi" / "profile.yaml")
    world = generate_world(course, RobotProfile(name="rover", geometry=shogi_profile.geometry))
    assert "<model name='rover'>" in world
    assert "<odom_topic>/model/rover/odometry</odom_topic>" in world
    assert "<tf_topic>/model/rover/tf</tf_topic>" in world
    assert "/model/shogi/" not in world

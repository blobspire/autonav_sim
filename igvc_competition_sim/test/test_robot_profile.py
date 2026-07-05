from pathlib import Path

import pytest

from igvc_competition_sim.robot_profile import RobotProfile, load_robot_profile


def test_derives_gazebo_topics_from_name():
    profile = RobotProfile(name="shogi")
    assert profile.gz_odom_topic == "/model/shogi/odometry"
    assert profile.gz_tf_topic == "/model/shogi/tf"


def test_load_profile_reads_name_and_description(tmp_path: Path):
    p = tmp_path / "profile.yaml"
    p.write_text("name: rover\ndescription: test bot\n", encoding="utf-8")
    profile = load_robot_profile(p)
    assert profile.name == "rover"
    assert profile.description == "test bot"
    assert profile.gz_odom_topic == "/model/rover/odometry"


def test_missing_name_raises(tmp_path: Path):
    p = tmp_path / "profile.yaml"
    p.write_text("description: no name here\n", encoding="utf-8")
    with pytest.raises(ValueError, match="missing required 'name'"):
        load_robot_profile(p)


def test_invalid_name_raises(tmp_path: Path):
    p = tmp_path / "profile.yaml"
    p.write_text("name: 'bad name/slash'\n", encoding="utf-8")
    with pytest.raises(ValueError, match=r"\[A-Za-z0-9_\]"):
        load_robot_profile(p)


def test_default_shogi_profile_loads():
    from igvc_competition_sim.robot_profile import (
        DEFAULT_ROBOT_PROFILE,
        load_robot_profile,
    )
    assert DEFAULT_ROBOT_PROFILE.is_file()
    profile = load_robot_profile()
    assert profile.name == "shogi"
    assert profile.gz_odom_topic == "/model/shogi/odometry"

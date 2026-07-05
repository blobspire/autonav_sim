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


def test_geometry_none_when_absent(tmp_path):
    p = tmp_path / "profile.yaml"
    p.write_text("name: bare\n", encoding="utf-8")
    profile = load_robot_profile(p)
    assert profile.geometry is None


def test_geometry_parsed_when_present(tmp_path):
    p = tmp_path / "profile.yaml"
    p.write_text(
        "name: g\n"
        "geometry:\n"
        "  base_link_to_nav_center_m: 0.225\n"
        "  lidar_x_from_base_link_m: 0.6598\n"
        "  lidar_z_from_base_link_m: 0.20568\n"
        "  base_link_height_above_ground_m: 0.11303\n"
        "  gps_x_from_base_link_m: -0.2122\n"
        "  gps_y_from_base_link_m: -0.000105\n"
        "  gps_z_from_base_link_m: 0.66161\n"
        "  wheel_track_m: 0.72326\n"
        "  wheel_radius_m: 0.12946\n"
        "  physical_half_length_m: 0.545\n"
        "  physical_half_width_m: 0.410\n"
        "  footprint_padding_m: 0.050\n"
        "  max_linear_speed_mps: 0.50\n"
        "  max_angular_speed_radps: 1.0\n"
        "  cmd_latency_s: 0.08\n"
        "  linear_time_constant_s: 0.20\n"
        "  angular_time_constant_s: 0.18\n",
        encoding="utf-8",
    )
    profile = load_robot_profile(p)
    assert profile.geometry is not None
    assert profile.geometry.wheel_track_m == 0.72326
    assert profile.geometry.physical_half_length_m == 0.545


def test_default_shogi_profile_has_geometry():
    profile = load_robot_profile()
    assert profile.geometry is not None
    assert profile.geometry.wheel_track_m == 0.72326

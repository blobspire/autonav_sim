from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import re
from typing import Any

import yaml

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
_DEFAULT_PROFILE_NAME = "shogi"
_VALID_NAME = re.compile(r"[A-Za-z0-9_]+")


def _default_robot_profile() -> Path:
    source_tree = (
        PACKAGE_ROOT / "profiles" / _DEFAULT_PROFILE_NAME / "profile.yaml"
    )
    if source_tree.is_file():
        return source_tree
    try:
        from ament_index_python.packages import get_package_share_directory
    except ImportError:
        return source_tree
    return (
        Path(get_package_share_directory("igvc_competition_sim"))
        / "profiles" / _DEFAULT_PROFILE_NAME / "profile.yaml"
    )


DEFAULT_ROBOT_PROFILE = _default_robot_profile()


@dataclass(frozen=True)
class RobotSpec:
    base_link_to_nav_center_m: float
    lidar_x_from_base_link_m: float
    lidar_z_from_base_link_m: float
    base_link_height_above_ground_m: float
    gps_x_from_base_link_m: float
    gps_y_from_base_link_m: float
    gps_z_from_base_link_m: float
    wheel_track_m: float
    wheel_radius_m: float
    physical_half_length_m: float
    physical_half_width_m: float
    footprint_padding_m: float
    max_linear_speed_mps: float
    max_angular_speed_radps: float
    cmd_latency_s: float
    linear_time_constant_s: float
    angular_time_constant_s: float


@dataclass(frozen=True)
class RobotProfile:
    """Identity of the robot the sim spawns and bridges."""

    name: str
    description: str = ""
    geometry: "RobotSpec | None" = None

    @property
    def gz_odom_topic(self) -> str:
        return f"/model/{self.name}/odometry"

    @property
    def gz_tf_topic(self) -> str:
        return f"/model/{self.name}/tf"


def load_robot_profile(path: str | Path | None = None) -> RobotProfile:
    profile_path = Path(path) if path else DEFAULT_ROBOT_PROFILE
    raw: Any = yaml.safe_load(Path(profile_path).read_text(encoding="utf-8"))
    data = raw or {}
    if not isinstance(data, dict):
        raise ValueError(
            f"robot profile {profile_path} must be a YAML mapping")
    name = str(data.get("name", "")).strip()
    if not name:
        raise ValueError(
            f"robot profile {profile_path} missing required 'name'")
    if not _VALID_NAME.fullmatch(name):
        raise ValueError(
            f"robot profile name '{name}' must match [A-Za-z0-9_]+ "
            "(it becomes the Gazebo model name and /model/<name>/odometry)")
    geometry_raw = data.get("geometry")
    geometry: RobotSpec | None = None
    if geometry_raw is not None:
        if not isinstance(geometry_raw, dict):
            raise ValueError(
                f"robot profile {profile_path} 'geometry' must be a mapping")
        geometry = RobotSpec(
            **{key: float(value) for key, value in geometry_raw.items()})
    return RobotProfile(
        name=name,
        description=str(data.get("description", "")),
        geometry=geometry,
    )

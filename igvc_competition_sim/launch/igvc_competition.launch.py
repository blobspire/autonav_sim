#!/usr/bin/env python3
from __future__ import annotations

import hashlib
import os
from pathlib import Path
import re
import shutil
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _package_share(package: str) -> str:
    return get_package_share_directory(package)


def _default_course_config() -> str:
    return os.path.join(
        _package_share("igvc_competition_sim"),
        "config",
        "igvc_competition_compact.yaml",
    )


def _default_world() -> str:
    return os.path.join(
        _package_share("igvc_competition_sim"),
        "worlds",
        "igvc_competition_compact.sdf",
    )


def _default_dynamics_calibration() -> str:
    return os.path.join(
        _package_share("igvc_competition_sim"),
        "config",
        "dynamics_calibration.yaml",
    )


def _default_nav2_params() -> str:
    try:
        candidate = Path(_package_share("slam")) / "config" / "nav2_params_camera.yaml"
        if candidate.is_file():
            return str(candidate)
    except Exception:
        pass
    return ""


def _default_bt_xml() -> str:
    try:
        candidate = Path(_package_share("slam")) / "behavior_trees" / "bt_nav.xml"
        if candidate.is_file():
            return str(candidate)
    except Exception:
        pass
    return ""


def _load_robot_description(robot_description_path: str = "") -> str:
    candidates = [Path(robot_description_path)] if robot_description_path else []
    candidates.append(Path(_package_share("bringup")) / "description" / "shogi.urdf")
    for path in candidates:
        if path.is_file():
            return path.read_text(encoding="utf-8")
    searched = "\n  ".join(str(path) for path in candidates)
    raise FileNotFoundError(
        "Could not find shogi.urdf. Build and source the workspace containing "
        "the AutoNav bringup package, or pass "
        "robot_description_path:=/absolute/path/to/shogi.urdf. Searched:\n  "
        + searched)


def _gazebo_process(context, *args, **kwargs):
    if not _truthy(context, "launch_gazebo"):
        return []
    world = LaunchConfiguration("world").perform(context)
    server_only = (
        LaunchConfiguration("gazebo_server_only").perform(context).lower()
        in ("1", "true", "yes", "on")
    )
    server_args = ["-s"] if server_only else []
    gz = shutil.which("gz")
    ign = shutil.which("ign")
    # Prefer Ignition Fortress (`ign gazebo`). On hosts that also ship the
    # legacy Gazebo Classic `gz` tool (gz-tools 11, /usr/bin/gz), `gz sim` is
    # NOT a valid command and exits immediately with "Invalid arguments", so
    # `ign` MUST take precedence. Garden+/Harmonic hosts (unified `gz sim`, no
    # `ign`) fall through to `gz sim`. (auto_camera env-portability fix.)
    if ign:
        cmd = [ign, "gazebo", *server_args, "-r", world]
    elif gz:
        cmd = [gz, "sim", *server_args, "-r", world]
    else:
        cmd = ["ign", "gazebo", *server_args, "-r", world]
    return [ExecuteProcess(cmd=cmd, output="screen")]


def _truthy(context, name: str) -> bool:
    return LaunchConfiguration(name).perform(context).lower() in (
        "1", "true", "yes", "on")


def _validate_world_sync(context, *args, **kwargs):
    if not _truthy(context, "validate_world_sync"):
        return []
    course_path = Path(LaunchConfiguration("course_config").perform(context))
    world_path = Path(LaunchConfiguration("world").perform(context))
    if not course_path.is_file() or not world_path.is_file():
        return []
    from igvc_competition_sim.course import load_course
    from igvc_competition_sim.generate_world import generate_world

    expected = "\n".join(
        line.rstrip()
        for line in generate_world(load_course(course_path)).splitlines()
    ) + "\n"
    actual = world_path.read_text(encoding="utf-8")
    if actual != expected:
        actual_sha = hashlib.sha256(actual.encode("utf-8")).hexdigest()
        expected_sha = hashlib.sha256(expected.encode("utf-8")).hexdigest()
        raise RuntimeError(
            "Generated world does not match course YAML: "
            f"{world_path} actual_sha256={actual_sha} "
            f"expected_sha256={expected_sha}")
    return []


def _line_detection_mode(context) -> str:
    mode = LaunchConfiguration("line_detection_mode").perform(context).lower()
    if mode not in ("camera", "ground_truth", "lidar"):
        raise RuntimeError(
            "line_detection_mode must be camera, ground_truth, or lidar")
    return mode


def _robot_state_publisher(context, *args, **kwargs):
    if not _truthy(context, "launch_robot_state_publisher"):
        return []
    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": _load_robot_description(
                    LaunchConfiguration("robot_description_path").perform(context)),
                "use_sim_time": True,
            }],
        )
    ]


def _nav2_params_with_bt(params_file: str, bt_xml: str) -> str:
    source = Path(params_file)
    text = source.read_text(encoding="utf-8")
    for key in ("default_nav_to_pose_bt_xml",
                "default_nav_through_poses_bt_xml"):
        text = re.sub(
            rf"({key}:\s*).+",
            rf'\1"{bt_xml}"',
            text,
        )
    digest = hashlib.sha1(
        (str(source) + "\n" + bt_xml).encode("utf-8")).hexdigest()[:12]
    output = Path(tempfile.gettempdir()) / f"igvc_nav2_params_{digest}.yaml"
    output.write_text(text, encoding="utf-8")
    return str(output)


def _nav2_process(context, *args, **kwargs):
    if not _truthy(context, "launch_nav"):
        return []
    params_arg = LaunchConfiguration("nav2_params").perform(context)
    bt_arg = LaunchConfiguration("bt_xml").perform(context)
    params_source = params_arg or _default_nav2_params()
    bt_xml = bt_arg or _default_bt_xml()
    if not params_source:
        raise FileNotFoundError(
            "Could not find slam/config/nav2_params_camera.yaml from the "
            "active ROS package index. Build and source the workspace "
            "containing the AutoNav slam package, or pass "
            "nav2_params:=/absolute/path/to/nav2_params_camera.yaml.")
    if not bt_xml:
        raise FileNotFoundError(
            "Could not find slam/behavior_trees/bt_nav.xml from the active "
            "ROS package index. Build and source the workspace containing the "
            "AutoNav slam package, or pass bt_xml:=/absolute/path/to/bt_nav.xml.")
    nav2_launch = os.path.join(
        _package_share("nav2_bringup"),
        "launch",
        "navigation_launch.py",
    )
    params_file = _nav2_params_with_bt(params_source, bt_xml)
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                "params_file": params_file,
                "use_sim_time": "true",
            }.items(),
        )
    ]


def _breadcrumb_buffer_process(context, *args, **kwargs):
    if not _truthy(context, "launch_nav"):
        return []
    if not _truthy(context, "launch_breadcrumb_buffer"):
        return []
    params_arg = LaunchConfiguration("nav2_params").perform(context)
    params_source = params_arg or _default_nav2_params()
    if not params_source:
        raise FileNotFoundError(
            "Could not find slam/config/nav2_params_camera.yaml from the "
            "active ROS package index. Build and source the workspace "
            "containing the AutoNav slam package, or pass "
            "nav2_params:=/absolute/path/to/nav2_params_camera.yaml.")
    return [
        Node(
            package="custom_behavior_tree_plugins",
            executable="breadcrumb_buffer",
            name="breadcrumb_buffer",
            output="screen",
            parameters=[params_source, {"use_sim_time": True}],
        )
    ]


def _camera_bridge_process(context, *args, **kwargs):
    if not _truthy(context, "launch_camera_bridge"):
        return []
    if _line_detection_mode(context) != "camera":
        return []
    return [
        Node(
            package="igvc_competition_sim",
            executable="igvc_camera_bridge",
            name="igvc_camera_bridge",
            output="screen",
            parameters=[{"use_sim_time": True}],
        )
    ]


def _harness_process(context, *args, **kwargs):
    if not _truthy(context, "launch_harness"):
        return []
    return [
        Node(
            package="igvc_competition_sim",
            executable="igvc_sensor_harness",
            name="igvc_sensor_harness",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "course_config": LaunchConfiguration("course_config"),
                "fallback_integrate_cmd": LaunchConfiguration(
                    "fallback_integrate_cmd"),
                "publish_full_lidar_cloud": ParameterValue(
                    LaunchConfiguration("publish_full_lidar_cloud"),
                    value_type=bool,
                ),
                "publish_ground_truth_pca": LaunchConfiguration(
                    "ground_truth_pca"),
                "publish_ground_truth_lines": (
                    _line_detection_mode(context) == "ground_truth"),
                "ground_truth_line_rate_hz": ParameterValue(
                    LaunchConfiguration("ground_truth_line_rate_hz"),
                    value_type=float,
                ),
                "ground_truth_line_view_limited": ParameterValue(
                    LaunchConfiguration("ground_truth_line_view_limited"),
                    value_type=bool,
                ),
                "ground_truth_line_range_min_m": ParameterValue(
                    LaunchConfiguration("ground_truth_line_range_min_m"),
                    value_type=float,
                ),
                "ground_truth_line_range_max_m": ParameterValue(
                    LaunchConfiguration("ground_truth_line_range_max_m"),
                    value_type=float,
                ),
                "ground_truth_line_angle_min_rad": ParameterValue(
                    LaunchConfiguration("ground_truth_line_angle_min_rad"),
                    value_type=float,
                ),
                "ground_truth_line_angle_max_rad": ParameterValue(
                    LaunchConfiguration("ground_truth_line_angle_max_rad"),
                    value_type=float,
                ),
                "max_ground_truth_line_points": ParameterValue(
                    LaunchConfiguration("max_ground_truth_line_points"),
                    value_type=int,
                ),
                "publish_odom_tf": ParameterValue(
                    LaunchConfiguration("publish_harness_odom_tf"),
                    value_type=bool,
                ),
            }],
        )
    ]


def _odom_bridge_process(context, *args, **kwargs):
    if not _truthy(context, "launch_odom_bridge"):
        return []
    return [
        Node(
            package="igvc_competition_sim",
            executable="igvc_odom_bridge",
            name="igvc_odom_bridge",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "max_relay_rate_hz": ParameterValue(
                    LaunchConfiguration("odom_bridge_rate_hz"),
                    value_type=float,
                ),
            }],
        )
    ]


def _calibrated_dynamics_process(context, *args, **kwargs):
    if not _truthy(context, "launch_dynamics"):
        return []
    return [
        Node(
            package="igvc_competition_sim",
            executable="igvc_calibrated_dynamics",
            name="igvc_calibrated_dynamics",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "calibration_config": LaunchConfiguration(
                    "dynamics_calibration"),
                "enabled": ParameterValue(
                    LaunchConfiguration("use_calibrated_dynamics"),
                    value_type=bool,
                ),
            }],
        )
    ]


def _detection_process(context, *args, **kwargs):
    if not _truthy(context, "launch_detection"):
        return []
    mode = _line_detection_mode(context)
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                _package_share("autonav_detection"),
                "launch",
                "detection.launch.py",
            )),
            launch_arguments={
                "enable_line": "true" if mode == "camera" else "false",
                "enable_grade": (
                    "false" if _truthy(context, "ground_truth_pca")
                    else "true"
                ),
                "enable_lidar_line": "true" if mode == "lidar" else "false",
                "use_sim_time": "true",
            }.items(),
        )
    ]


def generate_launch_description() -> LaunchDescription:
    course_config = LaunchConfiguration("course_config")
    run_id = LaunchConfiguration("run_id")
    launch_bridge = LaunchConfiguration("launch_bridge")
    launch_monitor = LaunchConfiguration("launch_monitor")
    launch_pca_scan_converters = LaunchConfiguration("launch_pca_scan_converters")
    launch_gps_handler = LaunchConfiguration("launch_gps_handler")

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="igvc_gz_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel_gazebo@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/model/shogi/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/igvc_sim/zed/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/igvc_sim/zed/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/igvc_sim/zed/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
        condition=IfCondition(launch_bridge),
    )

    monitor = Node(
        package="igvc_competition_sim",
        executable="igvc_course_monitor",
        name="igvc_course_monitor",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "course_config": course_config,
            "run_id": run_id,
        }],
        condition=IfCondition(launch_monitor),
    )

    pca_scan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pca_cloud_to_laserscan",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "target_frame": "base_link",
            "queue_size": 1,
            "min_height": -0.10,
            "max_height": 1.50,
            "angle_min": -1.5708,
            "angle_max": 1.5708,
            "angle_increment": 0.0087,
            "scan_time": 0.1,
            "range_min": 0.30,
            "range_max": 25.0,
            "use_inf": True,
        }],
        remappings=[
            ("cloud_in", "/scan_pca_filtered_points"),
            ("scan", "/scan_pca_filtered"),
        ],
        condition=IfCondition(launch_pca_scan_converters),
    )

    pca_scan_clear = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pca_cloud_to_laserscan_clear",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "target_frame": "base_link",
            "queue_size": 1,
            "min_height": -0.10,
            "max_height": 1.50,
            "angle_min": -1.2217,
            "angle_max": 1.2217,
            "angle_increment": 0.0087,
            "scan_time": 0.1,
            "range_min": 0.30,
            "range_max": 25.0,
            "use_inf": True,
        }],
        remappings=[
            ("cloud_in", "/scan_pca_filtered_points"),
            ("scan", "/scan_pca_filtered_clear"),
        ],
        condition=IfCondition(launch_pca_scan_converters),
    )

    gps_handler = Node(
        package="gps_waypoint_handler",
        executable="gps_handler_node",
        name="gps_handler_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(launch_gps_handler),
    )

    return LaunchDescription([
        DeclareLaunchArgument("course_config", default_value=_default_course_config()),
        DeclareLaunchArgument("world", default_value=_default_world()),
        DeclareLaunchArgument("validate_world_sync", default_value="true"),
        DeclareLaunchArgument("run_id", default_value=""),
        DeclareLaunchArgument("launch_gazebo", default_value="true"),
        DeclareLaunchArgument("gazebo_server_only", default_value="true"),
        DeclareLaunchArgument("launch_bridge", default_value="true"),
        DeclareLaunchArgument("launch_camera_bridge", default_value="true"),
        DeclareLaunchArgument("launch_harness", default_value="true"),
        DeclareLaunchArgument("launch_odom_bridge", default_value="true"),
        DeclareLaunchArgument("odom_bridge_rate_hz", default_value="30.0"),
        DeclareLaunchArgument("publish_harness_odom_tf", default_value="false"),
        DeclareLaunchArgument("launch_dynamics", default_value="true"),
        DeclareLaunchArgument("launch_robot_state_publisher", default_value="true"),
        DeclareLaunchArgument("launch_detection", default_value="true"),
        DeclareLaunchArgument("launch_monitor", default_value="true"),
        DeclareLaunchArgument("launch_pca_scan_converters", default_value="true"),
        DeclareLaunchArgument("launch_gps_handler", default_value="true"),
        DeclareLaunchArgument("launch_nav", default_value="true"),
        DeclareLaunchArgument("launch_breadcrumb_buffer", default_value="true"),
        DeclareLaunchArgument("use_calibrated_dynamics", default_value="true"),
        DeclareLaunchArgument(
            "dynamics_calibration",
            default_value=_default_dynamics_calibration(),
        ),
        DeclareLaunchArgument("ground_truth_pca", default_value="false"),
        DeclareLaunchArgument("ground_truth_line_rate_hz", default_value="20.0"),
        DeclareLaunchArgument("ground_truth_line_view_limited", default_value="true"),
        DeclareLaunchArgument("ground_truth_line_range_min_m", default_value="0.8"),
        DeclareLaunchArgument("ground_truth_line_range_max_m", default_value="6.0"),
        DeclareLaunchArgument("ground_truth_line_angle_min_rad", default_value="-0.96"),
        DeclareLaunchArgument("ground_truth_line_angle_max_rad", default_value="0.96"),
        DeclareLaunchArgument("max_ground_truth_line_points", default_value="2500"),
        DeclareLaunchArgument(
            "line_detection_mode",
            default_value="camera",
            description="Line source: camera, ground_truth, or lidar.",
        ),
        DeclareLaunchArgument("fallback_integrate_cmd", default_value="false"),
        DeclareLaunchArgument("publish_full_lidar_cloud", default_value="true"),
        DeclareLaunchArgument(
            "nav2_params",
            default_value=_default_nav2_params(),
        ),
        DeclareLaunchArgument("bt_xml", default_value=_default_bt_xml()),
        DeclareLaunchArgument(
            "robot_description_path",
            default_value="",
            description=(
                "Optional absolute path to shogi.urdf. Empty means use the "
                "bringup package installed in the active workspace."
            ),
        ),
        SetEnvironmentVariable(
            "IGN_GAZEBO_RESOURCE_PATH",
            os.pathsep.join([
                _package_share("igvc_competition_sim"),
                str(Path(_package_share("bringup")).parent),
                os.environ.get("IGN_GAZEBO_RESOURCE_PATH", ""),
            ]),
        ),
        OpaqueFunction(function=_validate_world_sync),
        OpaqueFunction(function=_gazebo_process),
        bridge,
        OpaqueFunction(function=_calibrated_dynamics_process),
        OpaqueFunction(function=_camera_bridge_process),
        OpaqueFunction(function=_odom_bridge_process),
        OpaqueFunction(function=_robot_state_publisher),
        OpaqueFunction(function=_harness_process),
        monitor,
        OpaqueFunction(function=_detection_process),
        pca_scan,
        pca_scan_clear,
        gps_handler,
        OpaqueFunction(function=_breadcrumb_buffer_process),
        OpaqueFunction(function=_nav2_process),
    ])

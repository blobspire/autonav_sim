#!/usr/bin/env python3
"""One-command minimal-robot demo.

Spawns the bundled `minibot` into the IGVC competition course and drives it to
the finish with `minimal_navigator` — CPU-only and with NO AutoNav packages.
This is the instant clone-and-run showcase and the worked "plug your own robot
in" example: it simply launches the sim (igvc_competition) with a minimal-robot
profile and every AutoNav node turned off, then starts the navigator.

    ros2 launch minimal_robot minimal_demo.launch.py            # with the Gazebo GUI
    ros2 launch minimal_robot minimal_demo.launch.py headless:=true   # VM / CI
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_share = get_package_share_directory("igvc_competition_sim")
    minimal_share = get_package_share_directory("minimal_robot")
    profile = os.path.join(minimal_share, "config", "minimal_profile.yaml")
    sim_launch = os.path.join(sim_share, "launch", "igvc_competition.launch.py")

    return LaunchDescription([
        DeclareLaunchArgument(
            "headless", default_value="false",
            description="Run Gazebo without the GUI (for the VM / CI)."),
        # The generic sim, with a minimal-robot profile and every AutoNav node
        # off (lidar line mode => no camera bridge, no ground-truth-lines, so no
        # autonav_interfaces). The harness/monitor use the true ground-truth pose.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch),
            launch_arguments={
                "robot_profile": profile,
                "launch_nav": "false",
                "launch_detection": "false",
                "launch_gps_handler": "false",
                "launch_pca_scan_converters": "false",
                "launch_breadcrumb_buffer": "false",
                "launch_camera_bridge": "false",
                "line_detection_mode": "lidar",
                "publish_full_lidar_cloud": "false",
                "gazebo_server_only": LaunchConfiguration("headless"),
            }.items(),
        ),
        # The minimal robot's autonomy. It waits for the first pose, so starting
        # it alongside the sim is fine.
        Node(
            package="minimal_robot",
            executable="minimal_navigator",
            name="minimal_navigator",
            output="screen",
        ),
    ])

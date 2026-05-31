#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_WS="${ROS_WS:-$(cd "$SCRIPT_DIR/../../.." && pwd)}"
COURSE_CONFIG="${COURSE_CONFIG:-}"
DYNAMICS_CALIBRATION="${DYNAMICS_CALIBRATION:-}"
LINE_DETECTION_MODE="${LINE_DETECTION_MODE:-camera}"
GROUND_TRUTH_PCA="${GROUND_TRUTH_PCA:-false}"
PUBLISH_FULL_LIDAR_CLOUD="${PUBLISH_FULL_LIDAR_CLOUD:-}"
USE_CALIBRATED_DYNAMICS="${USE_CALIBRATED_DYNAMICS:-true}"
LAUNCH_MONITOR="${LAUNCH_MONITOR:-false}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS Humble setup not found at /opt/ros/humble/setup.bash" >&2
  exit 1
fi

if [[ ! -f "$ROS_WS/install/setup.bash" ]]; then
  echo "Workspace install setup not found:" >&2
  echo "  $ROS_WS/install/setup.bash" >&2
  echo "Build the workspace first: cd $ROS_WS && colcon build" >&2
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
source "$ROS_WS/install/setup.bash"
set -u

pkg_share() {
  local package="$1"
  local prefix
  prefix="$(ros2 pkg prefix "$package")"
  printf '%s/share/%s\n' "$prefix" "$package"
}

SIM_SHARE="$(pkg_share igvc_competition_sim)"
SLAM_SHARE="$(pkg_share slam)"
COURSE_CONFIG="${COURSE_CONFIG:-$SIM_SHARE/config/igvc_competition_compact.yaml}"
DYNAMICS_CALIBRATION="${DYNAMICS_CALIBRATION:-$SIM_SHARE/config/dynamics_calibration.yaml}"
if [[ -z "${NAV2_PARAMS:-}" ]]; then
  if [[ "$LINE_DETECTION_MODE" == "lidar" ]]; then
    NAV2_PARAMS="$SLAM_SHARE/config/nav2_params_lidar.yaml"
  else
    NAV2_PARAMS="$SLAM_SHARE/config/nav2_params_camera.yaml"
  fi
fi
if [[ -z "$PUBLISH_FULL_LIDAR_CLOUD" ]]; then
  if [[ "$GROUND_TRUTH_PCA" == "true" ]]; then
    PUBLISH_FULL_LIDAR_CLOUD="false"
  else
    PUBLISH_FULL_LIDAR_CLOUD="true"
  fi
fi

export ROS_DOMAIN_ID ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION

exec ros2 launch igvc_competition_sim igvc_competition.launch.py \
  course_config:="$COURSE_CONFIG" \
  ground_truth_pca:="$GROUND_TRUTH_PCA" \
  publish_full_lidar_cloud:="$PUBLISH_FULL_LIDAR_CLOUD" \
  line_detection_mode:="$LINE_DETECTION_MODE" \
  nav2_params:="$NAV2_PARAMS" \
  use_calibrated_dynamics:="$USE_CALIBRATED_DYNAMICS" \
  dynamics_calibration:="$DYNAMICS_CALIBRATION" \
  launch_gazebo:=false \
  launch_bridge:=false \
  launch_camera_bridge:=true \
  launch_harness:=true \
  launch_odom_bridge:=true \
  publish_harness_odom_tf:=false \
  launch_dynamics:=true \
  launch_robot_state_publisher:=true \
  launch_detection:=true \
  launch_monitor:="$LAUNCH_MONITOR" \
  launch_pca_scan_converters:=true \
  launch_gps_handler:=true \
  launch_nav:=true

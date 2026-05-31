#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_WS="${ROS_WS:-$(cd "$SCRIPT_DIR/../../.." && pwd)}"
COURSE_CONFIG="${COURSE_CONFIG:-}"
WORLD="${WORLD:-}"
GAZEBO_SERVER_ONLY="${GAZEBO_SERVER_ONLY:-true}"
LINE_DETECTION_MODE="${LINE_DETECTION_MODE:-camera}"
GROUND_TRUTH_PCA="${GROUND_TRUTH_PCA:-false}"
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
COURSE_CONFIG="${COURSE_CONFIG:-$SIM_SHARE/config/igvc_competition_compact.yaml}"
WORLD="${WORLD:-$SIM_SHARE/worlds/igvc_competition_compact.sdf}"

export ROS_DOMAIN_ID ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION

exec ros2 launch igvc_competition_sim igvc_competition.launch.py \
  course_config:="$COURSE_CONFIG" \
  world:="$WORLD" \
  ground_truth_pca:="$GROUND_TRUTH_PCA" \
  line_detection_mode:="$LINE_DETECTION_MODE" \
  launch_gazebo:=true \
  gazebo_server_only:="$GAZEBO_SERVER_ONLY" \
  launch_bridge:=true \
  launch_camera_bridge:=false \
  launch_harness:=false \
  launch_odom_bridge:=false \
  launch_dynamics:=false \
  launch_robot_state_publisher:=false \
  launch_detection:=false \
  launch_monitor:=true \
  launch_pca_scan_converters:=false \
  launch_gps_handler:=false \
  launch_nav:=false

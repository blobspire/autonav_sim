#!/usr/bin/env bash
# One-command bundled minimal-robot demo for autonav_sim.
#
# Prereqs: ROS 2 Humble + Gazebo Fortress, and a colcon workspace containing
# autonav_sim built and sourced (see docs/quickstart.md). Then:
#
#     ./scripts/run_minimal_demo.sh            # with the Gazebo GUI
#     ./scripts/run_minimal_demo.sh headless   # no GUI (VM / CI)
#
# It spawns the bundled `minibot` into the IGVC course and drives it to the
# finish with the minimal_navigator — CPU-only, no AutoNav packages required.
set -euo pipefail

HEADLESS="false"
if [[ "${1:-}" == "headless" || "${1:-}" == "--headless" ]]; then
  HEADLESS="true"
fi

if ! ros2 pkg prefix minimal_robot >/dev/null 2>&1; then
  echo "error: the 'minimal_robot' package isn't on the ROS path." >&2
  echo "  Build + source your workspace first (see docs/quickstart.md):" >&2
  echo "    colcon build && source install/setup.bash" >&2
  exit 1
fi

exec ros2 launch minimal_robot minimal_demo.launch.py "headless:=${HEADLESS}"

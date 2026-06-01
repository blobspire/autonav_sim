# Branch Autoresearch Context: auto_main

Generated: 2026-06-01T01:30:00

Robot branch: `auto_main`<br>
Base branch: `hailmary_deploy`<br>
Merge base: `0fe4da9bbad63d999c348498267b3584263ee91c`<br>
Robot head: `6fd4e2b3a591a6e3bba7398d1b6a63c60bc2f3e9`<br>
Affected subsystems: costmaps, gps_waypoint, launch_env, perception, planning_control

## Branch Delta

- `.gitignore`
- `AUTORESEARCH_PATH.md`
- `docs/AUTONOMY_DECISION_LOG.md`
- `docs/HUMAN-WRITTEN-README.md`
- `docs/HYPERPARAMETER.md`
- `docs/IGVC_COMPETITION_RULES.md`
- `docs/IGVC_FORTRESS_SIM.md`
- `docs/LAUNCH_STACK.md`
- `docs/LIDAR_LINE_AVOIDANCE_COURSE.md`
- `docs/PACKAGES.md`
- `docs/README.md`
- `docs/ROS_BAG_ANALYSIS.md`
- `docs/SENSORS.md`
- `docs/TROUBLESHOOTING.md`
- `env/docker/dockerfiles/Dockerfile`
- `env/docker/dockerfiles/Dockerfile.base`
- `isaac_ros-dev/config/fastdds_laptop.xml`
- `isaac_ros-dev/config/mission_precheck.py`
- `isaac_ros-dev/config/run-detect.sh`
- `isaac_ros-dev/config/run-lidar-lines.sh`
- `isaac_ros-dev/config/run-lines.sh`
- `isaac_ros-dev/config/run-nav2.sh`
- `isaac_ros-dev/config/run-sim.sh`
- `isaac_ros-dev/config/run_lidar_line_test.sh`
- `isaac_ros-dev/config/send_goal.sh`
- `isaac_ros-dev/config/stored_waypoints.txt`
- `isaac_ros-dev/src/autonav-gui-hud/autonav_gui_hud/hud_node.py`
- `isaac_ros-dev/src/autonav_detection/CMakeLists.txt`
- `isaac_ros-dev/src/autonav_detection/config/grade_detector.yaml`
- `isaac_ros-dev/src/autonav_detection/config/lidar_line_detector.yaml`
- `isaac_ros-dev/src/autonav_detection/config/line_detector.yaml`
- `isaac_ros-dev/src/autonav_detection/include/autonav_detection/cuda.cuh`
- `isaac_ros-dev/src/autonav_detection/launch/detection.launch.py`
- `isaac_ros-dev/src/autonav_detection/package.xml`
- `isaac_ros-dev/src/autonav_detection/src/grade/pca_node.cpp`
- `isaac_ros-dev/src/autonav_detection/src/lidar_line/node.cpp`
- `isaac_ros-dev/src/autonav_detection/src/line/cuda.cu`
- `isaac_ros-dev/src/autonav_detection/src/line/detection.cpp`
- `isaac_ros-dev/src/autonav_detection/src/line/node.cpp`
- `isaac_ros-dev/src/autonav_hybrid_planner/CMakeLists.txt`
- `isaac_ros-dev/src/autonav_hybrid_planner/autonav_hybrid_planner.xml`
- `isaac_ros-dev/src/autonav_hybrid_planner/include/autonav_hybrid_planner/local_then_straight_planner.hpp`
- `isaac_ros-dev/src/autonav_hybrid_planner/package.xml`
- `isaac_ros-dev/src/autonav_hybrid_planner/src/local_then_straight_planner.cpp`
- `isaac_ros-dev/src/autonav_sim/COLCON_IGNORE`
- `isaac_ros-dev/src/bringup/config/zed_override.yaml`
- `isaac_ros-dev/src/bringup/description/shogi.urdf`
- `isaac_ros-dev/src/control/config/node_params.yaml`
- `isaac_ros-dev/src/control/src/control.cpp`
- `isaac_ros-dev/src/custom_behavior_tree_plugins/CMakeLists.txt`
- `isaac_ros-dev/src/custom_behavior_tree_plugins/breadcrumb_reverse.xml`
- `isaac_ros-dev/src/custom_behavior_tree_plugins/include/breadcrumb/breadcrumb_buffer.hpp`
- `isaac_ros-dev/src/custom_behavior_tree_plugins/include/breadcrumb/breadcrumb_reverse.hpp`
- `isaac_ros-dev/src/custom_behavior_tree_plugins/include/breadcrumb/forward_blocked_check.hpp`
- `isaac_ros-dev/src/custom_behavior_tree_plugins/include/breadcrumb/is_forward_blocked.hpp`
- `isaac_ros-dev/src/custom_behavior_tree_plugins/include/gradient_escape/goal_bender.hpp`
- `isaac_ros-dev/src/custom_behavior_tree_plugins/package.xml`
- `isaac_ros-dev/src/custom_behavior_tree_plugins/src/breadcrumb_buffer.cpp`
- `isaac_ros-dev/src/custom_behavior_tree_plugins/src/breadcrumb_reverse.cpp`
- `isaac_ros-dev/src/custom_behavior_tree_plugins/src/goal_bender.cpp`
- `isaac_ros-dev/src/custom_behavior_tree_plugins/src/is_forward_blocked.cpp`
- `isaac_ros-dev/src/custom_behavior_tree_plugins/src/path_footprint_safe_condition.cpp`
- `isaac_ros-dev/src/custom_behavior_tree_plugins/src/path_significantly_changed_decorator.cpp`
- `isaac_ros-dev/src/gps_waypoint_handler/gps_waypoint_handler/gps_handler_node.py`
- `isaac_ros-dev/src/gps_waypoint_handler/package.xml`
- `isaac_ros-dev/src/line_layer/include/line_layer/line_layer.hpp`
- `isaac_ros-dev/src/line_layer/src/line_layer.cpp`
- `isaac_ros-dev/src/local_mirror_layer/include/local_mirror_layer/local_mirror_layer.hpp`
- `isaac_ros-dev/src/local_mirror_layer/src/local_mirror_layer.cpp`
- `isaac_ros-dev/src/map_padder/map_padder/map_padder_node.py`
- `isaac_ros-dev/src/sim/COLCON_IGNORE`
- `isaac_ros-dev/src/slam/behavior_trees/bt_nav.xml`
- `isaac_ros-dev/src/slam/config/nav2_paramsv2.yaml`
- `isaac_ros-dev/src/slam/launch/nav.launch.py`
- `isaac_ros-dev/src/slam/launch/slam.launch.py`
- `scripts/analyze_bt_control_churn.py`
- `scripts/analyze_costmap_footprint.py`
- `scripts/analyze_dwb_evaluation.py`
- `scripts/analyze_executed_footprint_costmap_collision.py`
- `scripts/analyze_global_plan_costmap_collision.py`
- `scripts/analyze_lidar_line_bag.py`
- `scripts/analyze_lidar_line_course_clearance.py`
- `scripts/analyze_lidar_line_plan_gap.py`
- `scripts/analyze_lidar_line_scenario.py`
- `scripts/analyze_lidar_line_timeline.py`
- `scripts/analyze_nav2_action_result.py`
- `scripts/analyze_pointcloud_footprint.py`
- `scripts/run_lidar_line_bag_analysis.sh`

## Inherited Findings

- **global**: IGVC course geometry, official full-loop validation, scorer integrity, and no-course-softening rules (course and scorer constraints are independent of robot branch)
- **needs_revalidation**: GPS waypoint handoff 0.45s delay fixed stale NavigateToPose goal consumption on Hailmary (gps_waypoint files changed)
- **needs_revalidation**: GoalBender and PathGoalConsistent global timeout/radius/angle-only tweaks caused cross-course regressions (planning/control or costmap files changed)
- **needs_revalidation**: Camera-line fidelity against real bags remains perception-specific future work (perception files changed)
- **global**: Pre-official-full-loop kept/discarded results are fast-suite evidence only (official_full_loop was added after the earlier fast-suite findings)

## 2026-06-01 Blender Oracle Findings

Use only `blender_competition_course` for the current nightly. The older
generated courses are not the authoritative validation target for `auto_main`.

Baseline result:

- Run dir: `results/timebox/20260601_040501`
- Robot: `auto_main@6fd4e2b3a`
- Sim/scorer: `autonav_sim@380abd69d245`
- Result: stopped early after nine repeated waypoint-1 aborts.
- Evidence: each attempt returned `NavigateToPose finished with ABORTED` within
  about 0.5 s of waypoint 1; progress stayed at `0.49-1.25 m`; course monitor
  reported no line crossing or obstacle contact.

Candidate result:

- Hypothesis: avoid self-preempting the first waypoint by using only the direct
  `NavigateToPose` action for the initial handoff instead of also publishing the
  same pose on `/goal_pose`.
- Change: first topic-side publish routed through `/goal_update`; the direct
  `NavigateToPose` action still starts the leg.
- Run dir: `results/timebox/20260601_041335`
- Status: `needs_rerun`, not committed.
- Evidence: the immediate abort disappeared and progress improved to
  `14.577 m` and `28.558 m`, with no course-monitor line/obstacle failure.
  Both completed attempts still timed out with `mission_status=124`.
- New dominant failure: repeated `GradientEscape` followed by
  `DriveOnHeading` `Collision Ahead` / `backup failed` churn after forward
  progress. This is now a recovery/blocked-forward target, not the original
  first-waypoint action abort.

The handoff candidate was reverted after logging so the next agent starts from
clean `auto_main` unless it intentionally reapplies that patch from the ledger
evidence.

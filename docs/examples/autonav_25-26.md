# Example: AutoNav 2025-26 (the real robot, oracle perception)

This is the real-world integration example: **[`AutoNav_25-26`](https://github.com/KazakhStallion/AutoNav_25-26)**,
the team's actual IGVC robot stack, driven through the simulator's course with
**oracle perception** — the sim feeds AutoNav's Nav2 planner *ground-truth* lane
lines and obstacles instead of running the camera/CUDA vision pipeline. It needs
**no GPU**, so it runs on the CPU-only VM.

Where the [bundled minimal robot](../../examples/minimal_robot/) proves the
sim is *reusable*, this example proves it still drives the **real** stack — the
regression check that generalizing the core (a robot profile + a robot-agnostic
world + a spawned URDF) didn't break the integration it was built for.

## What it demonstrates

- AutoNav's **Nav2** stack (bt_navigator / controller / planner / behaviors +
  the `gps_waypoint_handler` mission server) planning and driving the course.
- The robot spawned from AutoNav's **sim-complete `shogi.urdf`** (`ros_gz_sim
  create`) into the course-only world — the Phase 1b contract.
- The sim's **oracle perception**: the harness publishes perfect lane lines and a
  ground-truth obstacle cloud, so Nav2's costmap is exercised without vision.
- **Honest scoring**: the course monitor scores on `/igvc_sim/ground_truth_odom`
  (the true physics pose), so wheel slip or clipping a line can't be hidden.

## Prerequisites

- The `autonav-gazebo-sim` VM (ROS 2 Humble + Gazebo Fortress). Gazebo can't run
  on the macOS host.
- The workspace built from `vcs.yaml` (pulls both `AutoNav_25-26` and
  `autonav_sim`), including AutoNav's `slam`, `nav2_bringup`, `line_layer`,
  `pointcloud_to_laserscan`, `gps_waypoint_handler`, and `bringup` packages:

  ```bash
  mkdir -p ws/src && cd ws
  vcs import src < /path/to/autonav_sim/vcs.yaml
  colcon build            # AutoNav's control/ (real-robot only) can be skipped
  source install/setup.bash
  ```

## Run it

The committed launcher wires the whole thing up (stack → wait → mission → score):

```bash
# from the igvc_competition_sim package source dir — it sets the oracle env itself:
./Run_IGVC_COMPETITION_FORTRESS_ORACLE_TEST.command
```

Or drive it by hand — launch the stack, then start the mission once Nav2 is up:

```bash
# 1) the stack (full AutoNav + oracle perception, headless)
ros2 launch igvc_competition_sim igvc_competition.launch.py \
  line_detection_mode:=ground_truth ground_truth_pca:=true gazebo_server_only:=true

# 2) once bt_navigator is 'active', run the waypoint mission
ros2 run igvc_competition_sim igvc_mission_runner \
  --course-config <course>.yaml --timeout-sec 300
```

`line_detection_mode:=ground_truth` + `ground_truth_pca:=true` is the oracle
switch: it turns off every CUDA sub-node of `autonav_detection` and has the
harness publish the ground-truth perception instead. `nav2_params` defaults to
AutoNav's `slam` config automatically.

## What's happening under the hood

| topic | producer (oracle) | consumer |
|-------|-------------------|----------|
| `/line_points` | harness (ground-truth lanes) | Nav2 `line_layer` costmap plugin |
| `/scan_pca_filtered_points` | harness (ground-truth PCA cloud) | `pointcloud_to_laserscan` → obstacle layers |
| `/scan_fullframe`, `/cloud_all_fields_fullframe` | harness lidar | voxel / obstacle layers |
| `/map_padded` | harness | Nav2 `static_layer` (costmap geometry) |
| `/gps_fix`, `/odom`, `/local_ekf/odom` | harness | localization / `gps_handler` |
| `/navigate_to_waypoint` | `igvc_mission_runner` | `gps_handler` → Nav2 `navigate_to_pose` → `/cmd_vel` |

## Expected result

AutoNav plans and drives the course to the finish (all mission waypoints,
`finish_reached: true`). The run is scored honestly on the true pose — so
`/igvc_sim/score` will report any **course violations** (a clipped boundary line,
an obstacle brush, a ramp-edge departure) even when navigation succeeds. Tuning
those away is AutoNav's own planning/control work (and a natural target for the
[autoresearch](../../igvc_competition_sim/autoresearch/) loop), not the sim's — the
sim's job is to spawn the robot, feed it honest sensors, and score it truthfully.

> **Note (nav2_params):** AutoNav's `slam` package renamed its Nav2 configs over
> time (`nav2_params_camera.yaml`/`_lidar.yaml` → `nav2_paramsv2.yaml`/
> `nav2_params.yaml`). The launch resolves whichever exists, preferring the
> current names; pass `nav2_params:=/abs/path.yaml` to override.

# IGVC Fortress Simulation

The active Gazebo simulation target is `igvc_competition_sim`, built for ROS 2
Humble plus Gazebo Fortress. In the split-repo layout this package lives in the
standalone `autonav_sim` repository and consumes the active AutoNav checkout
through normal ROS package discovery.

## Purpose

This simulation is meant to test the robot stack, not a parallel simulator
stack. The launch file starts the same AutoNav detection, Nav2, behavior tree,
custom costmap layers, GPS waypoint action server, and analysis topics used by
the robot.

Gazebo provides the course scene, robot dynamics target, and a rendered RGB-D
camera. `igvc_camera_bridge` relays that camera into the same ZED topics the
real camera line detector consumes. `igvc_sensor_harness` publishes the rest of
the robot-facing sensor contract:

- `/zed/zed_node/rgb/color/rect/image`
- `/zed/zed_node/rgb/color/rect/camera_info`
- `/zed/zed_node/depth/depth_registered`
- `/cloud_all_fields_fullframe`
- `/scan_fullframe`
- `/scan_pca_filtered_points`
- `/gps_fix`
- `/odom`
- `/local_ekf/odom`
- `/map_padded`
- `/joint_states`
- `/autonomous_mode`

The default line source is camera detection against plain white tape. The
SICK-style cloud is still available for PCA obstacle testing and opt-in
retroreflective/lidar-line regression work.

## Course

The compact course source is:

```bash
src/autonav_sim/igvc_competition_sim/config/igvc_competition_compact.yaml
```

It includes:

- 10-20 ft lane widths.
- 3-inch plain white boundary tape plus an internal no-cross line.
- Legal 5 ft passages and narrower decoy geometry.
- Barrels/posts and a ramp below 15% grade.
- A four-leg mission using `/navigate_to_waypoint`, with the first local leg
  giving the GPS EKF enough motion before GPS waypoint legs.
- First-44-ft speed-check metadata and live scoring stations.

Regenerate the SDF after changing the YAML:

```bash
cd ~/autonav_ws/src/autonav_sim/igvc_competition_sim
python3 -m igvc_competition_sim.generate_world \
  --course-config config/igvc_competition_compact.yaml \
  --output worlds/igvc_competition_compact.sdf
```

## Run

Build the workspace first:

```bash
cd ~/autonav_ws
colcon build --symlink-install
source install/setup.bash
```

Run the full test:

```bash
cd ~/autonav_ws/src/autonav_sim/igvc_competition_sim
./Run_IGVC_COMPETITION_FORTRESS_TEST.command
```

Run the oracle isolation test before blaming the robot stack:

```bash
cd ~/autonav_ws/src/autonav_sim/igvc_competition_sim
./Run_IGVC_COMPETITION_FORTRESS_ORACLE_TEST.command
```

The oracle test uses ground-truth tape on `/line_points` plus ground-truth
barrel/post obstacles on `/scan_pca_filtered_points`. If oracle fails, inspect
Nav2/control/config. If oracle passes but camera/PCA mode fails, inspect sim
perception, sensor timing, or costmap ingestion first.

Analyze a completed run:

```bash
ros2 run igvc_competition_sim igvc_run_analyzer /path/to/run_dir
ros2 run igvc_competition_sim igvc_line_health_analyzer /path/to/run_dir \
  --course-config config/igvc_competition_compact.yaml
```

The runner starts Gazebo Fortress, bridges `/clock`, `/cmd_vel`, Gazebo
odometry, and the RGB-D camera, launches the robot stack, records a bag, sends the configured
mission through `/navigate_to_waypoint`, and saves the live monitor score.

Useful environment overrides:

- `COURSE_CONFIG=/path/to/course.yaml`
- `RUN_DIR=/path/to/output`
- `MISSION_TIMEOUT_SEC=300`
- `GROUND_TRUTH_PCA=true`
- `LINE_DETECTION_MODE=camera` (default), `ground_truth`, or `lidar`
- `NAV2_PARAMS=/path/to/nav2_params_lidar.yaml` when using `LINE_DETECTION_MODE=lidar`
- `GAZEBO_SERVER_ONLY=false` to request the Gazebo GUI on a machine with a display
- `LAUNCH_GAZEBO=false` for harness-only debugging
- `LAUNCH_BRIDGE=false` when using a custom bridge

## Distributed Laptop + Jetson Mode

Use this mode when Gazebo runs on the laptop/ROS VM and the spare Jetson runs
the robot stack. This keeps Gazebo rendering and physics off the Jetson while
still exercising the Jetson CPU/GPU with Nav2, MPPI, camera-line CUDA
projection, PCA, costmaps, BT plugins, and command generation.

Network baseline:

- Mac/laptop Ethernet: `10.66.0.1/24`
- Spare Jetson Ethernet: `10.66.0.2/24`
- Jetson perception Gazebo VM shared address: `192.168.105.2/24`
- Jetson perception Gazebo VM direct bridge address: `10.66.0.4/24` on the
  direct Jetson Ethernet bridge. Keep it for link checks, but do not use it as
  the DDS data locator on this Mac; raw UDP/TCP over that bridged path is
  unreliable even when ICMP ping works.
- SSH alias: `jetson-spare-eth`
- Recommended Jetson-in-the-loop ROS graph: `ROS_DOMAIN_ID=72`,
  `ROS_LOCALHOST_ONLY=0`
- Confirm the link is gigabit before testing: `1000baseT <full-duplex>` on the
  laptop and `Speed: 1000Mb/s` on the Jetson.

For the current Mac/Lima dual-lane setup, the Jetson perception Gazebo VM is
`autonav-ros22`. Its `~/.lima/autonav-ros22/lima.yaml` should include both the
stable Lima shared management network and the direct Jetson Ethernet bridge:

```yaml
networks:
- lima: shared
- lima: jetsoneth
```

The VM should assign the Jetson-side interface `10.66.0.4/24`; on the current
machine this is handled by the VM-local systemd unit
`autonav-jetsoneth-ip.service`. The same unit should route Jetson traffic over
Lima shared networking so DDS data uses `192.168.105.2`:

```text
10.66.0.2 via 192.168.105.1 dev lima0 src 192.168.105.2
```

The Jetson Ethernet connection should have the reciprocal persistent route:

```text
192.168.105.0/24 via 10.66.0.1
```

If the Mac hotspot or Ethernet cabling changes, do not trust a stale running
VM. Stop any master-owned Jetson smoke, restart `autonav-ros22`, and re-run the
preflight until pings and raw UDP probes pass between Jetson `10.66.0.2` and
VM `192.168.105.2`.

Before running camera-line tests, confirm the generated SDF world contains the
Gazebo Sensors system. Without it, Gazebo publishes `/clock` and odometry but
does not publish `/igvc_sim/zed/*` camera topics:

```bash
grep -n "ignition-gazebo-sensors-system" \
  ~/autonav_ws/src/autonav_sim/igvc_competition_sim/worlds/igvc_competition_compact.sdf
```

Run the Jetson stack inside the `koopa-kingdom` container:

```bash
ssh jetson-spare-eth
cd /home/vtcro/AutoNav_25-26
ROS_DOMAIN_ID=72 AUTONAV_CONTAINER_GUI=0 \
  AUTONAV_SIM_SOURCE=/home/vtcro/autonav_sim \
  FASTRTPS_DEFAULT_PROFILES_FILE=/autonav_sim/igvc_competition_sim/config/fastdds_jetson_robot.xml \
  FASTDDS_DEFAULT_PROFILES_FILE=/autonav_sim/igvc_competition_sim/config/fastdds_jetson_robot.xml \
  ./env/docker/run-container.sh --no-attach
docker exec -it -u admin \
  -e ROS_DOMAIN_ID=72 \
  -e ROS_LOCALHOST_ONLY=0 \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/autonav_sim/igvc_competition_sim/config/fastdds_jetson_robot.xml \
  -e FASTDDS_DEFAULT_PROFILES_FILE=/autonav_sim/igvc_competition_sim/config/fastdds_jetson_robot.xml \
  -e ROS_WS=/autonav/isaac_ros-dev \
  -e AUTONAV_ROS_WS=/autonav/isaac_ros-dev \
  koopa-kingdom \
  /bin/bash -lc 'cd /autonav_sim/igvc_competition_sim && ./Run_IGVC_COMPETITION_FORTRESS_JETSON_STACK.command'
```

If the Jetson uses separate workspaces, set `ROS_WS` to the standalone sim
workspace and `AUTONAV_ROS_WS` to the built AutoNav robot-stack workspace. The
script sources `AUTONAV_ROS_WS` first and the sim overlay second.

Run Gazebo and the simulation adapters on the laptop/ROS VM:

```bash
cd /home/cole.guest/autonav_jetson_sim_ws/src/autonav_sim/igvc_competition_sim
ROS_DOMAIN_ID=72 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  FASTRTPS_DEFAULT_PROFILES_FILE=/home/cole.guest/autonav_jetson_sim_ws/src/autonav_sim/igvc_competition_sim/config/fastdds_jetson_sim_vm.xml \
  FASTDDS_DEFAULT_PROFILES_FILE=/home/cole.guest/autonav_jetson_sim_ws/src/autonav_sim/igvc_competition_sim/config/fastdds_jetson_sim_vm.xml \
  ODOM_BRIDGE_RATE_HZ=60.0 \
  ./Run_IGVC_COMPETITION_FORTRESS_SIM_ONLY.command
```

Keep the Jetson-lane VM workspace under `/home/cole.guest`, not `/tmp`.
Restarting the Lima VM can clear `/tmp`, which removes the built sim workspace,
Fast DDS profile path, and installed `autonav_interfaces`.

Use the sim-specific Fast DDS profiles above for distributed runs. They pin
the ROS graph to reachable locators: `192.168.105.2` for the VM and
`10.66.0.2` for the Jetson. Without them, the VM can advertise unreachable
Lima/hotspot locators; the Jetson may then miss `/clock`, camera, GPS, and odom
publishers even though topic names appear in the graph. The profiles also keep
`127.0.0.1` enabled so local ROS participants on the same VM or Jetson can
discover each other without relying on Ethernet hairpin behavior.
The Jetson-lane sim side should use a denser `60 Hz` odom/TF relay so Nav2
costmap message filters have transform samples around the 20 Hz PCA scan
timestamps despite cross-host DDS jitter.

Quick link validation:

```bash
# VM -> Jetson
limactl shell autonav-ros22 ping -c 2 10.66.0.2

# Jetson -> VM DDS path
ssh jetson-spare-eth 'ping -c 2 192.168.105.2'

# ROS camera topics visible from inside the Jetson container
ssh jetson-spare-eth 'docker exec -u admin \
  -e ROS_DOMAIN_ID=72 \
  -e ROS_LOCALHOST_ONLY=0 \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/autonav_sim/igvc_competition_sim/config/fastdds_jetson_robot.xml \
  -e FASTDDS_DEFAULT_PROFILES_FILE=/autonav_sim/igvc_competition_sim/config/fastdds_jetson_robot.xml \
  koopa-kingdom \
  bash -lc "source /opt/ros/humble/setup.bash && \
    source /autonav/isaac_ros-dev/install/setup.bash && \
    timeout 4s ros2 topic echo --once --qos-reliability best_effort /zed/zed_node/rgb/color/rect/image >/dev/null"'
```

Role split:

- Laptop/VM: Gazebo Fortress, `ros_gz_bridge`, `/clock`, rendered RGB-D camera,
  Gazebo odometry, course monitor, `igvc_camera_bridge`, `igvc_odom_bridge`,
  `igvc_sensor_harness`, and calibrated `/cmd_vel -> /cmd_vel_gazebo`
  dynamics. These are simulation adapters, so keeping them off the Jetson
  prevents the sim from stealing CPU from Nav2/MPPI.
- Jetson: robot state publisher, CUDA camera line detector, PCA detector,
  pointcloud-to-laserscan converters, GPS waypoint action server, Nav2, MPPI,
  custom BT plugins, and costmaps.

The Jetson stack wrapper caps NumPy/BLAS thread pools at one thread by default.
This keeps the GPS waypoint EKF from spending CPU on OpenBLAS thread fanout for
small matrix operations, leaving more headroom for MPPI.

Do not publish odom/TF from both sides in distributed mode. The VM sim-only
script now defaults `launch_odom_bridge:=true publish_harness_odom_tf:=false`,
and the Jetson stack script defaults `launch_odom_bridge:=false`, so Nav2 sees
one monotonic 30 Hz base transform stream. This mirrors the expected
robot-localization cadence closely enough for the GPS waypoint handler without
overdriving its EKF callback. For fallback/debug, set
`LAUNCH_SIM_ADAPTERS=true` on the Jetson stack script, but do not run both
sides with sim adapters enabled at the same time.

The camera bridge intentionally republishes synchronized RGB, depth, and
camera-info messages under the ZED topic names instead of forwarding Gazebo
RGB/depth independently. This prevents the detector from pairing a fresh RGB
frame with stale depth, and it holds paired camera frames until `/local_ekf/odom`
has reached the frame stamp so stamped TF lookups remain deterministic. The
bridge also matches the real bag-observed ZED output shape: `bgra8` RGB,
`32FC1` depth, `zed_left_camera_frame_optical`, 960x540, and camera intrinsics
fx=fy=539.702, cx=472.965, cy=255.161. The rendered Gazebo camera uses the
equivalent rectified horizontal FOV of about 1.454 rad, not the raw ZED
marketing HFOV. In distributed mode these image/depth/info outputs must use
sensor-data/best-effort QoS, and the line detector must subscribe with
best-effort QoS. Reliable raw camera streams can saturate or backpressure the
routed VM-to-Jetson DDS link and stall unrelated `/clock` and `/tf` samples.

The camera line detector also buffers recent RGB/depth inputs and selects the newest
synchronized pair on each detection tick, which avoids callback-order races
between the two image subscriptions. In the distributed profile it also prefers
the newest synchronized pair whose stamped odom-to-base TF is already present in
the detector's TF buffer, avoiding transient future-extrapolation drops when TF
arrives slightly behind the camera bridge. If no TF-ready camera pair exists on
a tick, the detector republishes its held line set instead of projecting with a
future transform.

This mode intentionally moves raw RGB-D camera topics over the Ethernet link.
Use wired gigabit Ethernet or better; the Jetson USB gadget link is only
100 Mbps in this setup and is not appropriate for full-rate raw camera/depth.
Any RViz or scripted `NavigateToPose` sender used against this distributed sim
must use `/clock` / `use_sim_time` or stamp the goal header with zero time. A
wall-time goal stamp makes the BT path consistency guard compare wall time
against sim-time path stamps and can trigger false stale-path recoveries.

Line-source modes:

- `camera`: bridges Gazebo RGB-D into ZED topics, runs the CUDA camera line detector, and uses `nav2_params_camera.yaml`.
- `ground_truth`: publishes sampled course tape directly on `/line_points` to isolate Nav2 planning/control from camera perception.
- `lidar`: runs the SICK RSSI lidar-line detector for legacy retroreflective-tape regressions; pair it with `nav2_params_lidar.yaml`.

Potholes are intentionally not modeled in the active competition sim because
they are not part of the current competition course contract.

## Pass/Fail

`igvc_course_monitor` publishes `/igvc_sim/score` and `/igvc_sim/fail`. Current
hard failures are:

- Footprint crossing boundary tape, dashed tape, or internal no-cross lines.
- Footprint contact with barrels/posts.
- Leaving the legal ramp corridor while on the ramp.
- First 44 ft average speed below 1 mph.
- Speed above 5 mph.
- Stop/blocking interval over 60 s.

The bag still needs the same detailed post-run inspection used for lidar-line
regressions before treating a simulated pass as physical-robot evidence.

# Quickstart

`autonav_sim` is a general-purpose **"plug your own robot in"** IGVC AutoNav
simulator (ROS 2 Humble + Gazebo Fortress). This gets you from a fresh clone to
watching a robot autonomously drive the mock competition course.

## Prerequisites

- **ROS 2 Humble** + **Gazebo Fortress** (Ignition). Gazebo can't run on macOS —
  use a Linux host or a VM.
- `colcon`, `git`.

## Clone, build, run — the bundled minimal robot

```bash
mkdir -p ws/src && cd ws/src
git clone https://github.com/blobspire/autonav_sim.git
cd ..
colcon build
source install/setup.bash

# One command: spawn the minibot into the course and drive it to the finish.
ros2 launch minimal_robot minimal_demo.launch.py
# ...headless (no GUI), e.g. on a VM / CI:
ros2 launch minimal_robot minimal_demo.launch.py headless:=true
# ...or the wrapper script:
./src/autonav_sim/scripts/run_minimal_demo.sh          # add 'headless' for no GUI
```

You'll see the **minibot** — a small, CPU-only differential-drive robot that
needs **no AutoNav packages** — spawn at the start and autonomously navigate the
IGVC course: track the lane waypoints, avoid the barrels (backing out and
retrying if it wedges), climb the ramp, and reach the finish. The
`course_monitor` publishes live scoring on `/igvc_sim/score`.

The minimal robot is deliberately the simplest honest example. The **real
competition robot** (AutoNav 2025-26), referenced via `vcs.yaml`, is the
full-stack integration example.

## Plug in your own robot

The minimal robot (`examples/minimal_robot/`) is the worked reference. To run the
sim with your robot you add just two things:

1. **A robot description** — a sim-complete URDF (a diff-drive `<gazebo>` plugin +
   inertials/collisions so it spawns and drives). See
   `examples/minimal_robot/urdf/minimal_robot.urdf`.
2. **A robot profile** — identity + geometry, pointing `description_ref` at your
   URDF. See `igvc_competition_sim/profiles/README.md` and
   `examples/minimal_robot/config/minimal_profile.yaml`.

Then launch with your profile:

```bash
ros2 launch igvc_competition_sim igvc_competition.launch.py \
  robot_profile:=/abs/path/to/your_profile.yaml
```

The sim spawns your robot at the course start, publishes camera / lidar
(`/scan_fullframe`) / GPS (`/gps_fix`) / odom, and subscribes to `/cmd_vel`. Your
robot's stack does the rest.

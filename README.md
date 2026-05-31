# autonav_sim

Standalone Gazebo Fortress / IGVC simulation repository for the AutoNav ROS 2
Humble stack.

This repo contains the simulation package `igvc_competition_sim`, course assets,
run scripts, and simulation docs. It intentionally does not copy the robot stack.
Robot packages such as `bringup`, `slam`, `autonav_detection`,
`autonav_interfaces`, custom BT plugins, Nav2 params, URDF, and BT XML come from
the active AutoNav checkout in the same colcon workspace.

## Workspace Layout

Use both repos side by side:

```bash
mkdir -p ~/autonav_ws/src
cd ~/autonav_ws/src

# Pick whichever AutoNav branch you want to test.
git clone https://github.com/KazakhStallion/AutoNav_25-26.git AutoNav_25-26
git clone https://github.com/blobspire/autonav_sim.git autonav_sim

cd ~/autonav_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Local development without cloning again:

```bash
mkdir -p ~/autonav_ws/src
ln -s ~/code/git/AutoNav_25-26 ~/autonav_ws/src/AutoNav_25-26
ln -s ~/code/git/autonav_sim ~/autonav_ws/src/autonav_sim
cd ~/autonav_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

`colcon` recursively discovers packages under `src/AutoNav_25-26/isaac_ros-dev/src`
and `src/autonav_sim/igvc_competition_sim`.

Until the old in-tree sim package is removed from AutoNav, disable it in each
workspace so `colcon` does not see two packages named `igvc_competition_sim`:

```bash
touch ~/autonav_ws/src/AutoNav_25-26/isaac_ros-dev/src/igvc_competition_sim/COLCON_IGNORE
```

The bootstrap script does this automatically by default. It is an untracked
workspace-local file, not a source deletion.

## Import With vcs

Import both repos with `vcs.yaml`:

```bash
mkdir -p ~/autonav_ws/src
cd ~/autonav_ws/src
vcs import < /path/to/autonav_sim/vcs.yaml
cd ~/autonav_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

## Bootstrap Helper

For local setup:

```bash
cd /Users/cole/code/git/autonav_sim
./scripts/bootstrap_workspace.sh
```

Useful overrides:

```bash
AUTONAV_SOURCE=/Users/cole/code/git/AutoNav_25-26 \
SIM_SOURCE=/Users/cole/code/git/autonav_sim \
AUTONAV_WS=~/autonav_ws \
./scripts/bootstrap_workspace.sh --symlink
```

## Running

Full local/headless test:

```bash
cd ~/autonav_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
cd src/autonav_sim/igvc_competition_sim
./Run_IGVC_COMPETITION_FORTRESS_TEST.command
```

Oracle isolation test:

```bash
cd ~/autonav_ws/src/autonav_sim/igvc_competition_sim
./Run_IGVC_COMPETITION_FORTRESS_ORACLE_TEST.command
```

The oracle run uses ground-truth course tape and ground-truth barrel/post
obstacles. Use it to separate robot-stack planning/control failures from
camera-line or PCA perception issues.

Sim-only host:

```bash
cd ~/autonav_ws/src/autonav_sim/igvc_competition_sim
ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  ./Run_IGVC_COMPETITION_FORTRESS_SIM_ONLY.command
```

Robot-stack / Jetson host:

```bash
cd ~/autonav_ws/src/autonav_sim/igvc_competition_sim
ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  ./Run_IGVC_COMPETITION_FORTRESS_JETSON_STACK.command
```

The wrappers default `ROS_WS` to the containing workspace root. Override
`ROS_WS=/path/to/workspace` when running from another location.

## Package Boundaries

`igvc_competition_sim` uses package discovery for robot-stack assets:

- `get_package_share_directory("bringup")` for `shogi.urdf`.
- `get_package_share_directory("slam")` for Nav2 params and BT XML.
- `get_package_share_directory("autonav_detection")` for detector launch.
- `autonav_interfaces` as the installed message/action package.

Do not copy those packages into this repo. To test another robot branch, switch
the branch in `~/autonav_ws/src/AutoNav_25-26`, rebuild the workspace, and rerun
the sim.

## Migration Notes

The original `igvc_competition_sim` package remains in AutoNav for now. Safe
removal should happen only after:

1. `autonav_sim` is pushed to its own remote.
2. CI or a developer workspace builds both repos side by side.
3. The full Fortress test and distributed sim/Jetson smoke tests pass from the
   split repo.
4. AutoNav docs/scripts that reference the old package path are updated to point
   at `src/autonav_sim/igvc_competition_sim`.
5. AutoNav removes only the old sim package, not robot packages or shared
   interfaces.

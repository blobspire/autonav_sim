# Robot profiles

A **robot profile** is how a robot plugs into the simulator: it declares which
robot to spawn + bridge, where its sensors sit, and how big it is. A team
integrates by adding one profile (plus their robot's URDF).

## Using a profile

```bash
ros2 launch igvc_competition_sim igvc_competition.launch.py \
  robot_profile:=/abs/path/to/<your_robot>/profile.yaml
```

Bundled profiles:
- `profiles/shogi/profile.yaml` — the AutoNav 2025-26 reference robot (default).
- `examples/minimal_robot/config/minimal_profile.yaml` — the bundled minimal demo
  robot (`minibot`); see `examples/minimal_robot/` and `docs/quickstart.md`.

## Fields

| field             | required | meaning |
|-------------------|----------|---------|
| `name`            | yes  | Gazebo model name (`[A-Za-z0-9_]+`); forms `/model/<name>/odometry` and `/model/<name>/tf`. |
| `description_ref` | yes* | The spawnable robot description (URDF), used for BOTH the Gazebo spawn and `robot_state_publisher`. A `package://<pkg>/path` URI or an absolute path. (*If empty, the launch falls back to the bringup package's `shogi.urdf`.) |
| `geometry`        | yes  | Robot geometry + low-speed dynamics the sim's own nodes need (see below). |
| `spawn`           | no   | Optional spawn-pose override (`x`, `y`, `z`, `yaw`); each field independently falls back to the course start (or wheel radius for `z`) when unset. |
| `description`     | no   | Human-readable note. |
| `schema_version`  | no   | Profile schema version (currently `1`). |

### `geometry` (all fields required, floats)

The world generator, `sensor_harness` (which *simulates* the lidar / GPS / odom
from these offsets) and `course_monitor` (scoring footprint) read these — they
should match the robot's actual URDF dimensions:

- **Scoring footprint:** `physical_half_length_m`, `physical_half_width_m`,
  `footprint_padding_m`, `base_link_to_nav_center_m`.
- **Drive / odom:** `wheel_track_m`, `wheel_radius_m`.
- **Simulated lidar placement:** `lidar_x_from_base_link_m`,
  `lidar_z_from_base_link_m`, `base_link_height_above_ground_m`.
- **Simulated GPS placement:** `gps_x_from_base_link_m`, `gps_y_from_base_link_m`,
  `gps_z_from_base_link_m`.
- **Speed clamp + optional low-speed dynamics:** `max_linear_speed_mps`,
  `max_angular_speed_radps`, `cmd_latency_s`, `linear_time_constant_s`,
  `angular_time_constant_s` (use small/ideal values when you have no calibration).

See `profiles/shogi/profile.yaml` (the reference) and
`examples/minimal_robot/config/minimal_profile.yaml` (a minimal worked example)
for concrete values.

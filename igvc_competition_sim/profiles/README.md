# Robot profiles

A **robot profile** tells the simulator which robot it is spawning and
bridging. Today a profile declares the robot's Gazebo **model name**, which
also forms the simulator-side odometry/TF topics (`/model/<name>/odometry`,
`/model/<name>/tf`).

## Using a profile

```bash
ros2 launch igvc_competition_sim igvc_competition.launch.py \
  robot_profile:=/abs/path/to/profiles/<your_robot>/profile.yaml
```

The default is `profiles/shogi/profile.yaml` (the AutoNav 2025-26 reference
robot).

## Fields

| field            | required | meaning                                              |
|------------------|----------|------------------------------------------------------|
| `name`           | yes      | Gazebo model name; must match `[A-Za-z0-9_]+`.       |
| `description`    | no       | Human-readable note.                                 |
| `schema_version` | no       | Profile schema version (currently `1`).              |

> Robot **geometry/dynamics** (wheel track, sensor offsets, footprint) still
> live in the course config's `robot:` block. Migrating those into the
> profile is planned next.

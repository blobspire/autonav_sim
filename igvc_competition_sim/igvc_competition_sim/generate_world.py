from __future__ import annotations

import argparse
import math
from pathlib import Path
import re

from .course import Course, DEFAULT_COURSE_CONFIG, course_bounds, load_course
from .robot_profile import RobotProfile, load_robot_profile


def _clean_name(name: str) -> str:
    return re.sub(r"[^A-Za-z0-9_]+", "_", name).strip("_") or "model"


def _material(name: str, rgba: tuple[float, float, float, float]) -> str:
    r, g, b, a = rgba
    return (
        f"<material><ambient>{r} {g} {b} {a}</ambient>"
        f"<diffuse>{r} {g} {b} {a}</diffuse>"
        f"<specular>0.05 0.05 0.05 1</specular></material>"
    )


def _box_visual_model(name: str,
                      pose: tuple[float, float, float, float, float, float],
                      size: tuple[float, float, float],
                      rgba: tuple[float, float, float, float],
                      collide: bool = False,
                      static: bool = True) -> str:
    collision = ""
    if collide:
        collision = (
            "<collision name='collision'><geometry><box>"
            f"<size>{size[0]:.4f} {size[1]:.4f} {size[2]:.4f}</size>"
            "</box></geometry></collision>"
        )
    return f"""
    <model name='{_clean_name(name)}'>
      <static>{1 if static else 0}</static>
      <pose>{pose[0]:.4f} {pose[1]:.4f} {pose[2]:.4f} {pose[3]:.6f} {pose[4]:.6f} {pose[5]:.6f}</pose>
      <link name='link'>
        {collision}
        <visual name='visual'>
          <geometry><box><size>{size[0]:.4f} {size[1]:.4f} {size[2]:.4f}</size></box></geometry>
          {_material('mat', rgba)}
        </visual>
      </link>
    </model>"""


def _tape_model(name: str,
                start: tuple[float, float],
                end: tuple[float, float],
                width_m: float) -> str:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length = max(0.01, math.hypot(dx, dy))
    yaw = math.atan2(dy, dx)
    cx = 0.5 * (start[0] + end[0])
    cy = 0.5 * (start[1] + end[1])
    # Emissive white so the tape renders near-saturated (~230) like real white
    # IGVC tape in sunlight -- the real-tuned line detector (brightness 220 /
    # mew 200) needs this; do NOT lower the detector thresholds to match a dim
    # render. auto_camera sim-fidelity fix.
    return f"""
    <model name='{_clean_name(name)}'>
      <static>1</static>
      <pose>{cx:.4f} {cy:.4f} 0.011 0 0 {yaw:.6f}</pose>
      <link name='link'>
        <visual name='visual'>
          <geometry><box><size>{length:.4f} {width_m:.4f} 0.012</size></box></geometry>
          <material><ambient>0.98 0.98 0.98 1</ambient><diffuse>0.98 0.98 0.98 1</diffuse><specular>0.05 0.05 0.05 1</specular><emissive>0.9 0.9 0.9 1</emissive></material>
        </visual>
      </link>
    </model>"""


def _sloped_tape_model(name: str,
                       pose: tuple[float, float, float, float, float, float],
                       length: float,
                       width_m: float) -> str:
    return f"""
    <model name='{_clean_name(name)}'>
      <static>1</static>
      <pose>{pose[0]:.4f} {pose[1]:.4f} {pose[2]:.4f} {pose[3]:.6f} {pose[4]:.6f} {pose[5]:.6f}</pose>
      <link name='link'>
        <visual name='visual'>
          <geometry><box><size>{length:.4f} {width_m:.4f} 0.012</size></box></geometry>
          <material><ambient>0.98 0.98 0.98 1</ambient><diffuse>0.98 0.98 0.98 1</diffuse><specular>0.05 0.05 0.05 1</specular><emissive>0.9 0.9 0.9 1</emissive></material>
        </visual>
      </link>
    </model>"""


def _cylinder_model(name: str,
                    x: float,
                    y: float,
                    radius: float,
                    height: float,
                    rgba: tuple[float, float, float, float],
                    collide: bool) -> str:
    collision = ""
    if collide:
        collision = (
            "<collision name='collision'><pose>0 0 "
            f"{height * 0.5:.4f} 0 0 0</pose><geometry><cylinder>"
            f"<radius>{radius:.4f}</radius><length>{height:.4f}</length>"
            "</cylinder></geometry></collision>"
        )
    return f"""
    <model name='{_clean_name(name)}'>
      <static>1</static>
      <pose>{x:.4f} {y:.4f} 0 0 0 0</pose>
      <link name='link'>
        {collision}
        <visual name='visual'>
          <pose>0 0 {height * 0.5:.4f} 0 0 0</pose>
          <geometry><cylinder><radius>{radius:.4f}</radius><length>{height:.4f}</length></cylinder></geometry>
          {_material('mat', rgba)}
        </visual>
      </link>
    </model>"""


def _cone_model(name: str,
                x: float,
                y: float,
                radius: float,
                height: float,
                collide: bool) -> str:
    collision = ""
    if collide:
        collision = (
            "<collision name='collision'><pose>0 0 "
            f"{height * 0.5:.4f} 0 0 0</pose><geometry><cylinder>"
            f"<radius>{radius:.4f}</radius><length>{height:.4f}</length>"
            "</cylinder></geometry></collision>"
        )
    layers: list[str] = []
    layer_count = 5
    for idx in range(layer_count):
        frac = idx / layer_count
        layer_radius = max(radius * (1.0 - frac), radius * 0.18)
        layer_height = height / layer_count
        z = layer_height * (idx + 0.5)
        layers.append(
            f"""
        <visual name='visual_{idx}'>
          <pose>0 0 {z:.4f} 0 0 0</pose>
          <geometry><cylinder><radius>{layer_radius:.4f}</radius><length>{layer_height:.4f}</length></cylinder></geometry>
          {_material('mat', (0.95, 0.32, 0.05, 1.0))}
        </visual>"""
        )
    return f"""
    <model name='{_clean_name(name)}'>
      <static>1</static>
      <pose>{x:.4f} {y:.4f} 0 0 0 0</pose>
      <link name='link'>
        {collision}
        {''.join(layers)}
      </link>
    </model>"""


def _ramp_model(course: Course) -> str:
    out: list[str] = []
    for ramp in course.ramps:
        run_length_m = (
            ramp.run_length_m
            if ramp.run_length_m is not None
            else ramp.end_x_m - ramp.start_x_m
        )
        half_run = 0.5 * run_length_m
        if half_run <= 1e-6:
            continue
        center_x = (
            ramp.center_x_m
            if ramp.center_x_m is not None
            else 0.5 * (ramp.start_x_m + ramp.end_x_m)
        )
        cos_yaw = math.cos(ramp.yaw_rad)
        sin_yaw = math.sin(ramp.yaw_rad)
        length = math.hypot(half_run, ramp.rise_m)
        pitch = math.atan2(ramp.rise_m, half_run)
        for suffix, local_x, segment_pitch in (
                ("up", -0.5 * half_run, -pitch),
                ("down", 0.5 * half_run, pitch),
        ):
            segment_x = center_x + cos_yaw * local_x
            segment_y = ramp.center_y_m + sin_yaw * local_x
            out.append(_box_visual_model(
                f"{ramp.name}_{suffix}",
                (
                    segment_x,
                    segment_y,
                    0.5 * ramp.rise_m,
                    0.0,
                    segment_pitch,
                    ramp.yaw_rad,
                ),
                (length, ramp.width_m, 0.08),
                (0.45, 0.45, 0.42, 1.0),
                collide=True,
            ))
    return "\n".join(out)


def _ramp_line_models(course: Course) -> str:
    out: list[str] = []
    tape_width = course.tapes[0].width_m if course.tapes else 0.0762
    for ramp in course.ramps:
        run_length_m = (
            ramp.run_length_m
            if ramp.run_length_m is not None
            else ramp.end_x_m - ramp.start_x_m
        )
        half_run = 0.5 * run_length_m
        if half_run <= 1e-6:
            continue
        center_x = (
            ramp.center_x_m
            if ramp.center_x_m is not None
            else 0.5 * (ramp.start_x_m + ramp.end_x_m)
        )
        cos_yaw = math.cos(ramp.yaw_rad)
        sin_yaw = math.sin(ramp.yaw_rad)
        length = math.hypot(half_run, ramp.rise_m)
        pitch = math.atan2(ramp.rise_m, half_run)
        z = 0.5 * ramp.rise_m + 0.052
        for suffix, local_x, segment_pitch in (
                ("up", -0.5 * half_run, -pitch),
                ("down", 0.5 * half_run, pitch),
        ):
            for side, y_sign in (("left", 1.0), ("right", -1.0)):
                local_y = y_sign * ramp.width_m * 0.5
                line_x = center_x + cos_yaw * local_x - sin_yaw * local_y
                line_y = ramp.center_y_m + sin_yaw * local_x + cos_yaw * local_y
                out.append(_sloped_tape_model(
                    f"{ramp.name}_{suffix}_{side}_white_line",
                    (
                        line_x,
                        line_y,
                        z,
                        0.0,
                        segment_pitch,
                        ramp.yaw_rad,
                    ),
                    length,
                    tape_width,
                ))
    return "\n".join(out)


def generate_world(course: Course, profile: RobotProfile | None = None) -> str:
    # `profile` is accepted for API/back-compat (validate_world_sync and the CLI
    # pass it) but unused here: the generated world is robot-agnostic — the robot
    # spawns separately via `ros_gz_sim create` at launch.
    del profile
    min_x, min_y, max_x, max_y = course_bounds(course, margin_m=8.0)
    ground_size_x = max(60.0, max_x - min_x)
    ground_size_y = max(30.0, max_y - min_y)
    ground_x = 0.5 * (min_x + max_x)
    ground_y = 0.5 * (min_y + max_y)

    models: list[str] = []
    models.append(_box_visual_model(
        "asphalt_ground",
        (ground_x, ground_y, -0.025, 0.0, 0.0, 0.0),
        (ground_size_x, ground_size_y, 0.05),
        (0.16, 0.16, 0.15, 1.0),
        collide=True,
    ))
    for tape in course.tapes:
        models.append(_tape_model(tape.name, tape.start, tape.end, tape.width_m))
    for obstacle in course.obstacles:
        color = (0.95, 0.32, 0.05, 1.0)
        if obstacle.kind == "post":
            color = (0.25, 0.25, 0.25, 1.0)
        if obstacle.kind == "cone":
            models.append(_cone_model(
                obstacle.name,
                obstacle.center[0],
                obstacle.center[1],
                obstacle.radius_m,
                obstacle.height_m,
                collide=True,
            ))
        else:
            models.append(_cylinder_model(
                obstacle.name,
                obstacle.center[0],
                obstacle.center[1],
                obstacle.radius_m,
                obstacle.height_m,
                color,
                collide=True,
            ))
    models.append(_ramp_model(course))
    models.append(_ramp_line_models(course))

    return f"""<?xml version='1.0'?>
<sdf version='1.9'>
  <world name='igvc_competition'>
    <plugin filename='ignition-gazebo-physics-system' name='ignition::gazebo::systems::Physics'/>
    <plugin filename='ignition-gazebo-user-commands-system' name='ignition::gazebo::systems::UserCommands'/>
    <plugin filename='ignition-gazebo-scene-broadcaster-system' name='ignition::gazebo::systems::SceneBroadcaster'/>
    <!-- Sensors system: REQUIRED for the rgbd camera to render. When a world
         lists explicit system <plugin> tags, gz-sim loads ONLY those, so the
         camera produces no images without this (line detection sees nothing).
         auto_camera env-compat fix. -->
    <plugin filename='ignition-gazebo-sensors-system' name='ignition::gazebo::systems::Sensors'>
      <render_engine>ogre2</render_engine>
    </plugin>
    <light name='sun' type='directional'>
      <pose>0 0 20 0 0 0</pose>
      <diffuse>0.8 0.8 0.75 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.35 0.15 -0.92</direction>
    </light>
    <gravity>0 0 -9.80665</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type='adiabatic'/>
    <physics name='default_physics' type='ode'>
      <max_step_size>0.010</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>100</real_time_update_rate>
    </physics>
    <scene>
      <ambient>0.45 0.45 0.45 1</ambient>
      <background>0.70 0.72 0.75 1</background>
      <shadows>true</shadows>
    </scene>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>{course.datum_latitude_deg:.8f}</latitude_deg>
      <longitude_deg>{course.datum_longitude_deg:.8f}</longitude_deg>
      <elevation>{course.datum_altitude_m:.3f}</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>
    {"".join(models)}
  </world>
</sdf>
"""


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--course-config", default=str(DEFAULT_COURSE_CONFIG))
    parser.add_argument("--output", required=True)
    parser.add_argument(
        "--robot-profile", default="",
        help="Path to a robot profile.yaml (default: bundled shogi profile).")
    args = parser.parse_args()

    course = load_course(args.course_config)
    profile = load_robot_profile(args.robot_profile or None)
    output = Path(args.output).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    text = "\n".join(
        line.rstrip()
        for line in generate_world(course, profile).splitlines())
    output.write_text(text + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()

from __future__ import annotations

import argparse
import math
from pathlib import Path
import re

from .course import Course, DEFAULT_COURSE_CONFIG, course_bounds, load_course
from .robot_profile import RobotProfile, load_robot_profile

BRINGUP_MESH_URI_PREFIX = "model://bringup/description/meshes"
IN_TO_M = 0.0254
ROBOT_TOTAL_MASS_KG = 117.0 * 0.45359237
ROBOT_CG_HEIGHT_M = 10.5 * IN_TO_M
ROBOT_CG_FORWARD_OF_DRIVE_AXLE_M = 5.58 * IN_TO_M
CASTER_SWIVEL_MASS_KG = 0.45
CASTER_WHEEL_MASS_KG = 0.35
DRIVE_WHEEL_MASS_KG = 2.0
SHOGI_BODY_MASS_KG = ROBOT_TOTAL_MASS_KG - (
    2.0 * DRIVE_WHEEL_MASS_KG + CASTER_SWIVEL_MASS_KG +
    CASTER_WHEEL_MASS_KG
)


def _clean_name(name: str) -> str:
    return re.sub(r"[^A-Za-z0-9_]+", "_", name).strip("_") or "model"


def _material(name: str, rgba: tuple[float, float, float, float]) -> str:
    r, g, b, a = rgba
    return (
        f"<material><ambient>{r} {g} {b} {a}</ambient>"
        f"<diffuse>{r} {g} {b} {a}</diffuse>"
        f"<specular>0.05 0.05 0.05 1</specular></material>"
    )


def _mesh_uri(mesh_name: str) -> str:
    return f"{BRINGUP_MESH_URI_PREFIX}/{mesh_name}"


def _mesh_visual(name: str,
                 mesh_name: str,
                 pose: tuple[float, float, float, float, float, float],
                 rgba: tuple[float, float, float, float]) -> str:
    return f"""
        <visual name='{_clean_name(name)}'>
          <pose>{pose[0]:.6f} {pose[1]:.6f} {pose[2]:.6f} {pose[3]:.6f} {pose[4]:.6f} {pose[5]:.6f}</pose>
          <geometry><mesh><uri>{_mesh_uri(mesh_name)}</uri></mesh></geometry>
          {_material('mat', rgba)}
        </visual>"""


def _box_collision(name: str,
                   pose: tuple[float, float, float, float, float, float],
                   size: tuple[float, float, float],
                   mu: float = 0.9,
                   mu2: float | None = None) -> str:
    if mu2 is None:
        mu2 = mu
    return f"""
        <collision name='{_clean_name(name)}'>
          <pose>{pose[0]:.6f} {pose[1]:.6f} {pose[2]:.6f} {pose[3]:.6f} {pose[4]:.6f} {pose[5]:.6f}</pose>
          <geometry><box><size>{size[0]:.4f} {size[1]:.4f} {size[2]:.4f}</size></box></geometry>
          <surface><friction><ode><mu>{mu:.4f}</mu><mu2>{mu2:.4f}</mu2></ode></friction></surface>
        </collision>"""


def _box_visual(name: str,
                pose: tuple[float, float, float, float, float, float],
                size: tuple[float, float, float],
                rgba: tuple[float, float, float, float]) -> str:
    return f"""
        <visual name='{_clean_name(name)}'>
          <pose>{pose[0]:.6f} {pose[1]:.6f} {pose[2]:.6f} {pose[3]:.6f} {pose[4]:.6f} {pose[5]:.6f}</pose>
          <geometry><box><size>{size[0]:.4f} {size[1]:.4f} {size[2]:.4f}</size></box></geometry>
          {_material('mat', rgba)}
        </visual>"""


def _cylinder_collision(name: str,
                        radius: float,
                        length: float,
                        pose: tuple[float, float, float, float, float, float]
                        | None = None,
                        mu: float = 1.2,
                        mu2: float | None = None) -> str:
    if mu2 is None:
        mu2 = mu
    pose_xml = ""
    if pose is not None:
        pose_xml = (
            f"<pose>{pose[0]:.6f} {pose[1]:.6f} {pose[2]:.6f} "
            f"{pose[3]:.6f} {pose[4]:.6f} {pose[5]:.6f}</pose>")
    return f"""
        <collision name='{_clean_name(name)}'>
          {pose_xml}
          <geometry><cylinder><radius>{radius:.5f}</radius><length>{length:.4f}</length></cylinder></geometry>
          <surface><friction><ode><mu>{mu:.4f}</mu><mu2>{mu2:.4f}</mu2></ode></friction></surface>
        </collision>"""


def _cylinder_visual(name: str,
                     radius: float,
                     length: float,
                     pose: tuple[float, float, float, float, float, float],
                     rgba: tuple[float, float, float, float]) -> str:
    return f"""
        <visual name='{_clean_name(name)}'>
          <pose>{pose[0]:.6f} {pose[1]:.6f} {pose[2]:.6f} {pose[3]:.6f} {pose[4]:.6f} {pose[5]:.6f}</pose>
          <geometry><cylinder><radius>{radius:.5f}</radius><length>{length:.4f}</length></cylinder></geometry>
          {_material('mat', rgba)}
        </visual>"""


def _inertial(mass: float,
              ixx: float,
              iyy: float,
              izz: float,
              pose: tuple[float, float, float, float, float, float]
              | None = None) -> str:
    pose_xml = ""
    if pose is not None:
        pose_xml = (
            f"<pose>{pose[0]:.6f} {pose[1]:.6f} {pose[2]:.6f} "
            f"{pose[3]:.6f} {pose[4]:.6f} {pose[5]:.6f}</pose>")
    return f"""
        <inertial>
          {pose_xml}
          <mass>{mass:.4f}</mass>
          <inertia>
            <ixx>{ixx:.5f}</ixx><iyy>{iyy:.5f}</iyy><izz>{izz:.5f}</izz>
            <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
          </inertia>
        </inertial>"""


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


def _robot_model(course: Course, profile: RobotProfile) -> str:
    robot = course.robot
    track = robot.wheel_track_m
    radius = robot.wheel_radius_m
    wheel_width = 0.1000
    caster_pivot_x = 0.577000
    caster_pivot_y = -0.000795
    caster_pivot_z = -0.040880
    caster_trail = 0.0450
    caster_wheel_radius = 0.0750
    caster_wheel_width = 0.0640
    caster_wheel_x = caster_pivot_x - caster_trail
    caster_wheel_y = caster_pivot_y
    caster_wheel_z = caster_wheel_radius - radius
    target_cg_x = ROBOT_CG_FORWARD_OF_DRIVE_AXLE_M
    target_cg_y = 0.0
    target_cg_z = ROBOT_CG_HEIGHT_M - radius
    child_cg_moment_x = (
        CASTER_SWIVEL_MASS_KG * caster_pivot_x +
        CASTER_WHEEL_MASS_KG * caster_wheel_x)
    child_cg_moment_y = (
        CASTER_SWIVEL_MASS_KG * caster_pivot_y +
        CASTER_WHEEL_MASS_KG * caster_wheel_y)
    child_cg_moment_z = (
        CASTER_SWIVEL_MASS_KG * caster_pivot_z +
        CASTER_WHEEL_MASS_KG * caster_wheel_z)
    body_cg_x = (
        ROBOT_TOTAL_MASS_KG * target_cg_x - child_cg_moment_x
    ) / SHOGI_BODY_MASS_KG
    body_cg_y = (
        ROBOT_TOTAL_MASS_KG * target_cg_y - child_cg_moment_y
    ) / SHOGI_BODY_MASS_KG
    body_cg_z = (
        ROBOT_TOTAL_MASS_KG * target_cg_z - child_cg_moment_z
    ) / SHOGI_BODY_MASS_KG
    z0 = radius
    return f"""
    <model name='{profile.name}'>
      <pose>{course.start.x:.4f} {course.start.y:.4f} {z0:.4f} 0 0 {course.start.yaw:.6f}</pose>
      <link name='base_link'>
        {_inertial(SHOGI_BODY_MASS_KG, 2.20, 4.30, 4.80,
                   (body_cg_x, body_cg_y, body_cg_z, 0.0, 0.0, 0.0))}
        {_box_collision('body_collision',
                        (robot.base_link_to_nav_center_m, 0.0, 0.1800,
                         0.0, 0.0, 0.0),
                        (1.0900, 0.8200, 0.2600))}
        {_mesh_visual('base_link_mesh',
                      'base_link.STL',
                      (-1.712700, -0.778935, -0.344190,
                       0.0, 0.0, 1.570796),
                      (0.79, 0.82, 0.93, 1.0))}
        {_mesh_visual('lidar_mesh',
                      'Lidar_Link.STL',
                      (0.659800, 0.000105, 0.205680,
                       -3.141593, -0.000004, 0.0),
                      (1.0, 1.0, 1.0, 1.0))}
        {_mesh_visual('camera_mesh',
                      'Camera_Link.STL',
                      (0.657600, 0.009075, 0.307230,
                       0.0, 0.349070, 0.0),
                      (0.79, 0.82, 0.93, 1.0))}
        {_mesh_visual('gps_mesh',
                      'GPS_Link.STL',
                      (-0.212200, -0.000105, 0.661610,
                       0.0, 0.000004, 0.0),
                      (1.0, 1.0, 1.0, 1.0))}
        <sensor name='zed2i_rgbd' type='rgbd_camera'>
          <pose>0.657600 0.009075 0.307230 0 0.349070 0</pose>
          <always_on>1</always_on>
          <update_rate>15</update_rate>
          <visualize>false</visualize>
          <topic>/igvc_sim/zed</topic>
          <camera>
            <horizontal_fov>1.453833</horizontal_fov>
            <image>
              <width>960</width>
              <height>540</height>
              <format>R8G8B8</format>
            </image>
            <clip>
              <near>0.10</near>
              <far>15.0</far>
            </clip>
          </camera>
        </sensor>
      </link>
      <link name='Caster_link'>
        <pose>{caster_pivot_x:.6f} {caster_pivot_y:.6f} {caster_pivot_z:.6f} 0 0 0</pose>
        {_inertial(CASTER_SWIVEL_MASS_KG, 0.0009, 0.0009, 0.0005)}
        {_cylinder_visual('caster_mount',
                          0.0280,
                          0.0550,
                          (0.0, 0.0, 0.0250, 0.0, 0.0, 0.0),
                          (0.55, 0.58, 0.66, 1.0))}
        {_cylinder_visual('caster_swivel_pin',
                          0.0180,
                          0.0500,
                          (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                          (0.32, 0.34, 0.40, 1.0))}
        {_box_visual('caster_fork_left',
                     (-caster_trail, 0.0420, -0.0030, 0.0, 0.0, 0.0),
                     (0.0360, 0.0160, 0.1500),
                     (0.42, 0.45, 0.52, 1.0))}
        {_box_visual('caster_fork_right',
                     (-caster_trail, -0.0420, -0.0030, 0.0, 0.0, 0.0),
                     (0.0360, 0.0160, 0.1500),
                     (0.42, 0.45, 0.52, 1.0))}
      </link>
      <joint name='Caster_Swivel' type='revolute'>
        <parent>base_link</parent>
        <child>Caster_link</child>
        <axis><xyz>0 0 1</xyz><dynamics><damping>0.004</damping><friction>0.0005</friction></dynamics><limit><lower>-1e16</lower><upper>1e16</upper></limit></axis>
      </joint>
      <link name='caster_wheel_link'>
        <pose>{caster_wheel_x:.6f} {caster_wheel_y:.6f} {caster_wheel_z:.6f} 0 0 0</pose>
        {_inertial(CASTER_WHEEL_MASS_KG, 0.0007, 0.0007, 0.0004)}
        {_cylinder_collision('caster_wheel_collision',
                             caster_wheel_radius,
                             caster_wheel_width,
                             (0.0, 0.0, 0.0, 1.570796, 0.0, 0.0),
                             mu=0.75,
                             mu2=0.02)}
        {_cylinder_visual('caster_wheel_visual',
                          caster_wheel_radius,
                          caster_wheel_width,
                          (0.0, 0.0, 0.0, 1.570796, 0.0, 0.0),
                          (0.04, 0.04, 0.04, 1.0))}
      </link>
      <joint name='Caster_Wheel_Roll' type='revolute'>
        <parent>Caster_link</parent>
        <child>caster_wheel_link</child>
        <axis><xyz>0 1 0</xyz><dynamics><damping>0.003</damping><friction>0.0001</friction></dynamics><limit><lower>-1e16</lower><upper>1e16</upper></limit></axis>
      </joint>
      <link name='left_wheel_link'>
        <pose>0 {track * 0.5:.5f} 0 1.570796 0 0</pose>
        {_inertial(DRIVE_WHEEL_MASS_KG, 0.025, 0.025, 0.025)}
        {_cylinder_collision('left_drive_wheel_collision', radius, wheel_width)}
        {_mesh_visual('left_wheel_mesh',
                      'Left_Wheel_Link.STL',
                      (0.0, 0.0, -0.767804, 0.0, -1.570796, 0.0),
                      (1.0, 1.0, 1.0, 1.0))}
      </link>
      <link name='right_wheel_link'>
        <pose>0 {-track * 0.5:.5f} 0 1.570796 0 0</pose>
        {_inertial(DRIVE_WHEEL_MASS_KG, 0.025, 0.025, 0.025)}
        {_cylinder_collision('right_drive_wheel_collision', radius, wheel_width)}
        {_mesh_visual('right_wheel_mesh',
                      'Right_Wheel_Link.STL',
                      (0.0, 0.0, 0.044545, 0.0, -1.570796, 0.0),
                      (1.0, 1.0, 1.0, 1.0))}
      </link>
      <joint name='Left_Wheel' type='revolute'>
        <parent>base_link</parent>
        <child>left_wheel_link</child>
        <axis><xyz>0 0 -1</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit></axis>
      </joint>
      <joint name='Right_Wheel' type='revolute'>
        <parent>base_link</parent>
        <child>right_wheel_link</child>
        <axis><xyz>0 0 -1</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit></axis>
      </joint>
      <plugin filename='ignition-gazebo-diff-drive-system' name='ignition::gazebo::systems::DiffDrive'>
        <left_joint>Left_Wheel</left_joint>
        <right_joint>Right_Wheel</right_joint>
        <wheel_separation>{track:.5f}</wheel_separation>
        <wheel_radius>{radius:.5f}</wheel_radius>
        <topic>/cmd_vel_gazebo</topic>
        <odom_topic>{profile.gz_odom_topic}</odom_topic>
        <tf_topic>{profile.gz_tf_topic}</tf_topic>
        <frame_id>odom</frame_id>
        <child_frame_id>base_link</child_frame_id>
        <odom_publish_frequency>50</odom_publish_frequency>
        <max_linear_acceleration>1.0</max_linear_acceleration>
        <max_angular_acceleration>2.0</max_angular_acceleration>
      </plugin>
    </model>"""


def generate_world(course: Course, profile: RobotProfile | None = None) -> str:
    if profile is None:
        profile = load_robot_profile()
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
    models.append(_robot_model(course, profile))

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

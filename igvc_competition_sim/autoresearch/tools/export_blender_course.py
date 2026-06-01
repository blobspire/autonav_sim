#!/usr/bin/env python3
"""Export an IGVC oracle course YAML from Blender object custom properties.

Run from Blender:

  blender --background Course.blend --python export_blender_course.py -- \
      --output course_from_blender.yaml

Expected custom properties:
  type: cone | ground | line | ramp | waypoint | start
  waypoint_order: integer, for waypoint objects

The script also accepts untyped empties with waypoint_order. It tolerates the
temporary Blender default property name "prop" as a waypoint_order fallback.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

import bpy
from mathutils import Vector


ROBOT_DEFAULTS = {
    "base_link_to_nav_center_m": 0.225,
    "lidar_x_from_base_link_m": 0.6598,
    "lidar_z_from_base_link_m": 0.20568,
    "base_link_height_above_ground_m": 0.11303,
    "gps_x_from_base_link_m": -0.2122,
    "gps_y_from_base_link_m": -0.000105,
    "gps_z_from_base_link_m": 0.66161,
    "wheel_track_m": 0.72326,
    "wheel_radius_m": 0.12946,
    "physical_half_length_m": 0.545,
    "physical_half_width_m": 0.410,
    "footprint_padding_m": 0.050,
    "max_linear_speed_mps": 0.50,
    "max_angular_speed_radps": 1.0,
    "cmd_latency_s": 0.08,
    "linear_time_constant_s": 0.20,
    "angular_time_constant_s": 0.18,
}


def parse_args() -> argparse.Namespace:
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--course-id", default="course_from_blender")
    parser.add_argument("--meters-per-unit", type=float, default=1.0)
    parser.add_argument("--line-width-m", type=float, default=0.0762)
    parser.add_argument("--line-min-segment-m", type=float, default=0.03)
    parser.add_argument("--no-infer-heading", action="store_true")
    parser.add_argument("--datum-latitude-deg", type=float, default=37.23027)
    parser.add_argument("--datum-longitude-deg", type=float, default=-80.42504)
    parser.add_argument("--datum-altitude-m", type=float, default=650.0)
    return parser.parse_args(argv)


def prop_string(obj: bpy.types.Object, key: str, default: str = "") -> str:
    value = obj.get(key, default)
    return str(value).strip()


def prop_float(obj: bpy.types.Object, key: str, default: float) -> float:
    try:
        return float(obj.get(key, default))
    except (TypeError, ValueError):
        return default


def prop_int(obj: bpy.types.Object, key: str) -> int | None:
    value = obj.get(key)
    if value is None:
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def course_type(obj: bpy.types.Object) -> str:
    return prop_string(obj, "type").lower()


def yaml_string(value: str) -> str:
    return json.dumps(value)


def fmt(value: float) -> str:
    if abs(value) < 0.0000005:
        value = 0.0
    return f"{value:.4f}".rstrip("0").rstrip(".")


def object_location(obj: bpy.types.Object) -> Vector:
    return obj.matrix_world.translation.copy()


class Frame:
    def __init__(self, origin: Vector, heading_rad: float, meters_per_unit: float):
        self.origin = origin
        self.heading_rad = heading_rad
        self.scale = meters_per_unit
        self.c = math.cos(heading_rad)
        self.s = math.sin(heading_rad)

    def xy(self, world: Vector) -> tuple[float, float]:
        dx = (world.x - self.origin.x) * self.scale
        dy = (world.y - self.origin.y) * self.scale
        # +x points along heading; +y is left of heading.
        return dx * self.c + dy * self.s, -dx * self.s + dy * self.c

    def z(self, world: Vector) -> float:
        return (world.z - self.origin.z) * self.scale


def collect_waypoints(warnings: list[str]) -> list[tuple[int, bpy.types.Object]]:
    waypoints: list[tuple[int, bpy.types.Object]] = []
    used = set()
    for obj in bpy.data.objects:
        order = prop_int(obj, "waypoint_order")
        if order is not None:
            waypoints.append((order, obj))
            used.add(obj.name)
            continue
        if course_type(obj) == "waypoint":
            name_order = None
            try:
                name_order = int(obj.name)
            except ValueError:
                pass
            waypoints.append((name_order if name_order is not None else 9999, obj))
            used.add(obj.name)

    for obj in bpy.data.objects:
        if obj.name in used or course_type(obj):
            continue
        fallback_order = prop_int(obj, "prop")
        if fallback_order is None:
            continue
        waypoints.append((fallback_order, obj))
        warnings.append(
            f"{obj.name}: used custom property 'prop'={fallback_order} "
            "as waypoint_order fallback"
        )

    waypoints.sort(key=lambda item: (item[0], item[1].name))
    return waypoints


def choose_frame(args: argparse.Namespace,
                 waypoints: list[tuple[int, bpy.types.Object]],
                 warnings: list[str]) -> tuple[Frame, bpy.types.Object]:
    starts = [obj for obj in bpy.data.objects if course_type(obj) == "start"]
    if not starts:
        raise SystemExit("No object with custom property type=start was found")
    start_obj = starts[0]
    if len(starts) > 1:
        warnings.append(f"multiple start objects found; using {start_obj.name}")

    heading = prop_float(start_obj, "yaw_rad", None)  # type: ignore[arg-type]
    if heading is None and not args.no_infer_heading and waypoints:
        target = object_location(waypoints[0][1])
        start = object_location(start_obj)
        dx = target.x - start.x
        dy = target.y - start.y
        if math.hypot(dx, dy) > 1e-6:
            heading = math.atan2(dy, dx)
            warnings.append(
                "inferred +x course heading from start to first waypoint; "
                f"heading_rad={heading:.6f}"
            )
    if heading is None:
        heading = start_obj.rotation_euler.z
        warnings.append(
            "using start object's Z rotation as course heading; "
            f"heading_rad={heading:.6f}"
        )

    return Frame(object_location(start_obj), heading, args.meters_per_unit), start_obj


def curve_points(obj: bpy.types.Object, frame: Frame) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    if obj.type != "CURVE":
        return points
    matrix = obj.matrix_world
    for spline in obj.data.splines:
        spline_points: list[Vector] = []
        if spline.type == "BEZIER":
            bez = spline.bezier_points
            max_idx = len(bez) if spline.use_cyclic_u else len(bez) - 1
            for idx in range(max_idx):
                p0 = bez[idx]
                p1 = bez[(idx + 1) % len(bez)]
                for sample in range(12):
                    t = sample / 12.0
                    a = (1.0 - t) ** 3
                    b = 3.0 * (1.0 - t) ** 2 * t
                    c = 3.0 * (1.0 - t) * t ** 2
                    d = t ** 3
                    spline_points.append(
                        p0.co * a
                        + p0.handle_right * b
                        + p1.handle_left * c
                        + p1.co * d
                    )
            if bez:
                spline_points.append(bez[0 if spline.use_cyclic_u else -1].co)
        else:
            spline_points = [Vector((p.co.x, p.co.y, p.co.z)) for p in spline.points]
            if spline.use_cyclic_u and spline_points:
                spline_points.append(spline_points[0])
        for point in spline_points:
            points.append(frame.xy(matrix @ point))
    return dedupe_consecutive(points)


def dedupe_consecutive(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    out: list[tuple[float, float]] = []
    for point in points:
        if not out or math.hypot(point[0] - out[-1][0], point[1] - out[-1][1]) > 1e-5:
            out.append(point)
    return out


def mesh_top_boundary_segments(obj: bpy.types.Object,
                               frame: Frame,
                               min_segment_m: float,
                               warnings: list[str]) -> list[tuple[tuple[float, float],
                                                                  tuple[float, float]]]:
    depsgraph = bpy.context.evaluated_depsgraph_get()
    eval_obj = obj.evaluated_get(depsgraph)
    mesh = eval_obj.to_mesh()
    try:
        if not mesh.vertices:
            return []
        matrix = eval_obj.matrix_world
        world_vertices = [matrix @ vertex.co for vertex in mesh.vertices]
        selected_counts: dict[tuple[int, int], int] = {}
        for poly in mesh.polygons:
            normal = matrix.to_3x3() @ poly.normal
            if normal.z < 0.05:
                continue
            verts = list(poly.vertices)
            for idx, start in enumerate(verts):
                end = verts[(idx + 1) % len(verts)]
                key = tuple(sorted((start, end)))
                selected_counts[key] = selected_counts.get(key, 0) + 1

        boundary_edges = [key for key, count in selected_counts.items() if count == 1]
        if not boundary_edges:
            warnings.append(f"{obj.name}: no top boundary edges found; falling back to mesh edges")
            boundary_edges = [
                tuple(sorted((edge.vertices[0], edge.vertices[1])))
                for edge in mesh.edges
            ]

        segments: list[tuple[tuple[float, float], tuple[float, float]]] = []
        seen: set[tuple[int, int]] = set()
        for start_idx, end_idx in boundary_edges:
            if (start_idx, end_idx) in seen:
                continue
            seen.add((start_idx, end_idx))
            start = frame.xy(world_vertices[start_idx])
            end = frame.xy(world_vertices[end_idx])
            if math.hypot(end[0] - start[0], end[1] - start[1]) < min_segment_m:
                continue
            segments.append((start, end))
        return segments
    finally:
        eval_obj.to_mesh_clear()


def line_segments(obj: bpy.types.Object,
                  frame: Frame,
                  args: argparse.Namespace,
                  warnings: list[str]) -> list[tuple[tuple[float, float],
                                                     tuple[float, float]]]:
    if obj.type == "CURVE":
        points = curve_points(obj, frame)
        return [
            (points[idx], points[idx + 1])
            for idx in range(len(points) - 1)
            if math.hypot(
                points[idx + 1][0] - points[idx][0],
                points[idx + 1][1] - points[idx][1],
            ) >= args.line_min_segment_m
        ]
    if obj.type == "MESH":
        return mesh_top_boundary_segments(
            obj, frame, args.line_min_segment_m, warnings
        )
    warnings.append(f"{obj.name}: unsupported line object type {obj.type}; skipped")
    return []


def mesh_world_vertices(obj: bpy.types.Object) -> list[Vector]:
    depsgraph = bpy.context.evaluated_depsgraph_get()
    eval_obj = obj.evaluated_get(depsgraph)
    mesh = eval_obj.to_mesh()
    try:
        return [eval_obj.matrix_world @ vertex.co for vertex in mesh.vertices]
    finally:
        eval_obj.to_mesh_clear()


def ramp_entry(obj: bpy.types.Object, frame: Frame) -> dict[str, float | str]:
    points = [frame.xy(vertex) + (frame.z(vertex),) for vertex in mesh_world_vertices(obj)]
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    zs = [point[2] for point in points]
    return {
        "name": obj.name,
        "start_x_m": min(xs),
        "end_x_m": max(xs),
        "center_y_m": 0.5 * (min(ys) + max(ys)),
        "width_m": max(ys) - min(ys),
        "rise_m": max(zs) - min(zs),
    }


def write_yaml(path: Path,
               args: argparse.Namespace,
               start_obj: bpy.types.Object,
               frame: Frame,
               waypoints: list[tuple[int, bpy.types.Object]],
               obstacles: list[dict[str, float | str]],
               ramps: list[dict[str, float | str]],
               internal_lines: list[dict[str, object]],
               warnings: list[str]) -> None:
    centerline_points = [(0.0, 0.0)] + [
        frame.xy(object_location(obj)) for _, obj in waypoints
    ]
    if len(centerline_points) < 2:
        centerline_points.append((1.0, 0.0))
        warnings.append("added dummy centerline station because no waypoints were found")
    finish = centerline_points[-1]

    lines: list[str] = []
    lines.append(f"course_id: {args.course_id}")
    lines.append('description: "Exported from Blender Course.blend"')
    lines.append("course_geometry: explicit_blender")
    lines.append("")
    for warning in warnings:
        lines.append(f"# export_warning: {warning}")
    if warnings:
        lines.append("")
    lines.append("datum:")
    lines.append(f"  latitude_deg: {fmt(args.datum_latitude_deg)}")
    lines.append(f"  longitude_deg: {fmt(args.datum_longitude_deg)}")
    lines.append(f"  altitude_m: {fmt(args.datum_altitude_m)}")
    lines.append("")
    lines.append("start:")
    lines.append("  x_m: 0.0")
    lines.append("  y_m: 0.0")
    lines.append("  yaw_rad: 0.0")
    lines.append("")
    lines.append("finish:")
    lines.append(f"  x_m: {fmt(finish[0])}")
    lines.append(f"  y_m: {fmt(finish[1])}")
    lines.append("  radius_m: 1.25")
    lines.append("")
    lines.append("robot:")
    for key, value in ROBOT_DEFAULTS.items():
        lines.append(f"  {key}: {fmt(value)}")
    lines.append("")
    lines.append(f"tape_width_m: {fmt(args.line_width_m)}")
    lines.append("")
    lines.append("centerline:")
    for x_m, y_m in centerline_points:
        lines.append(f"  - {{x_m: {fmt(x_m)}, y_m: {fmt(y_m)}, width_m: 6.096}}")
    lines.append("")
    lines.append("# Disable generated centerline boundaries; Blender line meshes below are authoritative.")
    lines.append("line_breaks:")
    for idx in range(len(centerline_points) - 1):
        lines.append(
            f"  - {{segment_index: {idx}, boundary: both, "
            "start_fraction: 0.0, end_fraction: 1.0}"
        )
    lines.append("")
    lines.append("internal_lines:")
    if internal_lines:
        for line in internal_lines:
            start = line["start"]
            end = line["end"]
            lines.append(f"  - name: {yaml_string(str(line['name']))}")
            lines.append(f"    start: {{x_m: {fmt(start[0])}, y_m: {fmt(start[1])}}}")
            lines.append(f"    end: {{x_m: {fmt(end[0])}, y_m: {fmt(end[1])}}}")
            lines.append(f"    width_m: {fmt(float(line['width_m']))}")
    else:
        lines.append("  []")
    lines.append("")
    lines.append("obstacles:")
    if obstacles:
        for obstacle in obstacles:
            lines.append(
                "  - {"
                f"name: {yaml_string(str(obstacle['name']))}, "
                f"type: {yaml_string(str(obstacle['type']))}, "
                f"x_m: {fmt(float(obstacle['x_m']))}, "
                f"y_m: {fmt(float(obstacle['y_m']))}, "
                f"radius_m: {fmt(float(obstacle['radius_m']))}, "
                f"height_m: {fmt(float(obstacle['height_m']))}"
                "}"
            )
    else:
        lines.append("  []")
    lines.append("")
    lines.append("ramps:")
    if ramps:
        for ramp in ramps:
            lines.append(f"  - name: {yaml_string(str(ramp['name']))}")
            lines.append(f"    start_x_m: {fmt(float(ramp['start_x_m']))}")
            lines.append(f"    end_x_m: {fmt(float(ramp['end_x_m']))}")
            lines.append(f"    center_y_m: {fmt(float(ramp['center_y_m']))}")
            lines.append(f"    width_m: {fmt(float(ramp['width_m']))}")
            lines.append(f"    rise_m: {fmt(float(ramp['rise_m']))}")
    else:
        lines.append("  []")
    lines.append("")
    lines.append("speed_check:")
    lines.append("  start_x_m: 0.0")
    lines.append("  end_distance_m: 13.4112")
    lines.append("  minimum_average_mps: 0.44704")
    lines.append("  maximum_speed_mps: 2.2352")
    lines.append("  blocking_stop_s: 60.0")
    lines.append("")
    lines.append("mission_waypoints:")
    if waypoints:
        for order, obj in waypoints:
            x_m, y_m = frame.xy(object_location(obj))
            kind = prop_string(obj, "waypoint_type", "gps") or "gps"
            label = prop_string(obj, "label", obj.name) or obj.name
            radius_m = prop_float(obj, "radius_m", 1.0)
            lines.append(
                "  - {"
                f"label: {yaml_string(label)}, "
                f"type: {yaml_string(kind)}, "
                f"x_m: {fmt(x_m)}, "
                f"y_m: {fmt(y_m)}, "
                f"radius_m: {fmt(radius_m)}, "
                f"order: {order}"
                "}"
            )
    else:
        lines.append("  []")
    lines.append("")
    lines.append("analysis_stations:")
    lines.append("  []")
    lines.append("")

    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    args = parse_args()
    warnings: list[str] = []
    waypoints = collect_waypoints(warnings)
    frame, start_obj = choose_frame(args, waypoints, warnings)

    obstacles: list[dict[str, float | str]] = []
    ramps: list[dict[str, float | str]] = []
    internal_lines: list[dict[str, object]] = []

    for obj in bpy.data.objects:
        typ = course_type(obj)
        if typ == "cone":
            x_m, y_m = frame.xy(object_location(obj))
            obstacles.append({
                "name": obj.name,
                "type": prop_string(obj, "obstacle_type", "cone") or "cone",
                "x_m": x_m,
                "y_m": y_m,
                "radius_m": prop_float(obj, "radius_m", max(obj.dimensions.x, obj.dimensions.y) * 0.5 * args.meters_per_unit),
                "height_m": prop_float(obj, "height_m", obj.dimensions.z * args.meters_per_unit),
            })
        elif typ == "line":
            segments = line_segments(obj, frame, args, warnings)
            for idx, (start, end) in enumerate(segments):
                internal_lines.append({
                    "name": f"{obj.name}_{idx:04d}",
                    "start": start,
                    "end": end,
                    "width_m": prop_float(obj, "width_m", args.line_width_m),
                })
        elif typ == "ramp":
            ramps.append(ramp_entry(obj, frame))
        elif typ in ("ground", "start", "waypoint", ""):
            continue
        else:
            warnings.append(f"{obj.name}: unknown type={typ!r}; skipped")

    output = args.output
    if output is None:
        blend_path = Path(bpy.data.filepath)
        output = blend_path.with_name("course_from_blender.yaml")
    write_yaml(
        output,
        args,
        start_obj,
        frame,
        waypoints,
        obstacles,
        ramps,
        internal_lines,
        warnings,
    )
    print(f"EXPORT_OUTPUT {output}")
    print(f"EXPORT_COUNTS cones={len(obstacles)} lines={len(internal_lines)} ramps={len(ramps)} waypoints={len(waypoints)}")
    for warning in warnings:
        print(f"EXPORT_WARNING {warning}")


if __name__ == "__main__":
    main()

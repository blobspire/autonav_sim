#!/usr/bin/env python3
"""Offline course feasibility validator (FROZEN harness component).

Without running the sim, confirm each authored course is at least geometrically
feasible for the padded robot: build a costmap-resolution occupancy grid where
tapes and obstacles are lethal, inflate by the robot inscribed radius
(physical_half_width + padding), and verify that start -> every mission waypoint
-> finish lie in one connected free region (8-connected). Catches gross errors
(a gap narrower than the robot, a waypoint inside an obstacle/tape) that would
otherwise only surface as a wasted sim run.

NOTE: this is a necessary (not sufficient) feasibility check -- it uses a
circular inscribed-radius inflation, so it can pass a course the kinodynamic
planner still finds hard. Route feasibility is confirmed for real on the first
sim baseline. ASCII output only. Exit 0 if all requested courses pass.
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.ndimage import binary_dilation, label

# import the FROZEN course loader from the package
PKG = Path(__file__).resolve().parents[2]  # .../igvc_competition_sim
sys.path.insert(0, str(PKG))
from igvc_competition_sim.course import (  # noqa: E402
    load_course,
    course_bounds,
    iter_course_points,
    load_yaml,
)

RES = 0.05
FT_TO_M = 0.3048
IN_TO_M = 0.0254
ROBOT_TOTAL_MASS_KG = 117.0 * 0.45359237
ROBOT_CG_HEIGHT_M = 10.5 * IN_TO_M
ROBOT_CG_FORWARD_OF_DRIVE_AXLE_M = 5.58 * IN_TO_M
MIN_LEGAL_PASSAGE_M = 1.524  # 5 ft IGVC minimum passage.
OFFICIAL_RULES_PROFILE = "igvc_2026_autonav_full_course"
OFFICIAL_AREA_LONG_M = 120.0 * FT_TO_M
OFFICIAL_AREA_SHORT_M = 100.0 * FT_TO_M
OFFICIAL_AREA_TOL_M = 1.0 * FT_TO_M
OFFICIAL_MIN_LENGTH_M = 450.0 * FT_TO_M
OFFICIAL_MAX_LENGTH_M = 575.0 * FT_TO_M
OFFICIAL_TRACK_MIN_M = 10.0 * FT_TO_M
OFFICIAL_TRACK_MAX_M = 20.0 * FT_TO_M
OFFICIAL_TURN_RADIUS_MIN_M = 5.0 * FT_TO_M
OFFICIAL_TAPE_WIDTH_M = 3.0 * IN_TO_M
OFFICIAL_TAPE_TOL_M = 0.25 * IN_TO_M
OFFICIAL_SPEED_DISTANCE_M = 44.0 * FT_TO_M
OFFICIAL_MIN_SPEED_MPS = 0.44704  # 1 mph
OFFICIAL_MAX_SPEED_MPS = 2.2352  # 5 mph
OFFICIAL_MAX_RAMP_GRADE = 0.15
GENERATED_BOUNDARY_WIDTH_TOL_M = 0.02


def _disk(radius_cells: int) -> np.ndarray:
    r = max(1, int(radius_cells))
    yy, xx = np.ogrid[-r:r + 1, -r:r + 1]
    return (xx * xx + yy * yy) <= r * r


def _nearest_centerline_gap(
        centerline: list[tuple[float, float, float]],
        x: float,
        y: float,
        radius_m: float) -> tuple[float, float, float]:
    best: tuple[float, float, float] | None = None
    for idx in range(len(centerline) - 1):
        ax, ay, aw = centerline[idx]
        bx, by, bw = centerline[idx + 1]
        dx = bx - ax
        dy = by - ay
        seg_len2 = dx * dx + dy * dy
        if seg_len2 <= 1e-9:
            continue
        t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / seg_len2))
        px = ax + t * dx
        py = ay + t * dy
        width = aw + t * (bw - aw)
        seg_len = math.sqrt(seg_len2)
        nx = -dy / seg_len
        ny = dx / seg_len
        lateral = (x - px) * nx + (y - py) * ny
        left_gap = 0.5 * width - lateral - radius_m
        right_gap = 0.5 * width + lateral - radius_m
        distance2 = (x - px) * (x - px) + (y - py) * (y - py)
        candidate = (distance2, left_gap, right_gap)
        if best is None or candidate[0] < best[0]:
            best = candidate
    if best is None:
        return 0.0, -math.inf, -math.inf
    _, left_gap, right_gap = best
    return max(left_gap, right_gap), left_gap, right_gap


def _centerline(data: dict) -> list[tuple[float, float, float]]:
    return [
        (float(p["x_m"]), float(p["y_m"]), float(p["width_m"]))
        for p in data.get("centerline", [])
    ]


def _uses_explicit_geometry(data: dict) -> bool:
    profile = str(data.get("course_geometry", "")).lower()
    return profile in ("explicit", "explicit_blender")


def _centerline_length(centerline: list[tuple[float, float, float]]) -> float:
    return sum(
        math.hypot(centerline[idx + 1][0] - centerline[idx][0],
                   centerline[idx + 1][1] - centerline[idx][1])
        for idx in range(len(centerline) - 1)
    )


def _turn_radius(a: tuple[float, float, float],
                 b: tuple[float, float, float],
                 c: tuple[float, float, float]) -> float:
    ax, ay, _ = a
    bx, by, _ = b
    cx, cy, _ = c
    side_a = math.hypot(cx - bx, cy - by)
    side_b = math.hypot(cx - ax, cy - ay)
    side_c = math.hypot(bx - ax, by - ay)
    area2 = abs((bx - ax) * (cy - ay) - (by - ay) * (cx - ax))
    if area2 <= 1e-9:
        return math.inf
    return (side_a * side_b * side_c) / (2.0 * area2)


def _exact_bounds(course) -> tuple[float, float, float, float]:
    points = tuple(iter_course_points(course))
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    return min(xs), min(ys), max(xs), max(ys)


def ramp_geometry_problems(course_path: Path) -> list[str]:
    data = load_yaml(course_path)
    if _uses_explicit_geometry(data):
        return []
    course = load_course(course_path)
    problems: list[str] = []
    for ramp in course.ramps:
        half_run = 0.5 * (ramp.end_x_m - ramp.start_x_m)
        if half_run <= 1e-6:
            problems.append(f"ramp {ramp.name} has non-positive half-run")
            continue
        grade = ramp.rise_m / half_run
        if grade > OFFICIAL_MAX_RAMP_GRADE + 1e-6:
            problems.append(
                f"ramp {ramp.name} up/down grade {grade * 100:.1f}% "
                "exceeds 15%; start_x_m/end_x_m define the full up-and-down "
                "ramp footprint")
    return problems


def _hairpin_inside_side(
        centerline: list[tuple[float, float, float]],
        idx: int) -> str | None:
    if idx <= 0 or idx >= len(centerline) - 2:
        return None
    previous = centerline[idx - 1]
    start = centerline[idx]
    end = centerline[idx + 1]
    next_point = centerline[idx + 2]
    prev_vec = (start[0] - previous[0], start[1] - previous[1])
    connector_vec = (end[0] - start[0], end[1] - start[1])
    next_vec = (next_point[0] - end[0], next_point[1] - end[1])
    prev_len = math.hypot(*prev_vec)
    connector_len = math.hypot(*connector_vec)
    next_len = math.hypot(*next_vec)
    if min(prev_len, connector_len, next_len) <= 1e-9:
        return None
    prev_u = (prev_vec[0] / prev_len, prev_vec[1] / prev_len)
    connector_u = (
        connector_vec[0] / connector_len,
        connector_vec[1] / connector_len,
    )
    next_u = (next_vec[0] / next_len, next_vec[1] / next_len)
    if prev_u[0] * next_u[0] + prev_u[1] * next_u[1] > -0.75:
        return None
    if abs(prev_u[0] * connector_u[0] + prev_u[1] * connector_u[1]) > 0.35:
        return None
    if abs(next_u[0] * connector_u[0] + next_u[1] * connector_u[1]) > 0.35:
        return None
    left_normal = (-connector_u[1], connector_u[0])
    connector_mid = ((start[0] + end[0]) * 0.5, (start[1] + end[1]) * 0.5)
    adjacent_mid = (
        (previous[0] + start[0] + end[0] + next_point[0]) * 0.25,
        (previous[1] + start[1] + end[1] + next_point[1]) * 0.25,
    )
    inward = (adjacent_mid[0] - connector_mid[0],
              adjacent_mid[1] - connector_mid[1])
    return "left" if inward[0] * left_normal[0] + (
        inward[1] * left_normal[1]) >= 0.0 else "right"


def _break_coverage(data: dict,
                    segment_index: int,
                    boundary: str) -> float:
    intervals: list[tuple[float, float]] = []
    for raw in data.get("line_breaks", []):
        if int(raw.get("segment_index", -1)) != segment_index:
            continue
        raw_boundary = str(raw.get("boundary", raw.get("side",
                                                       "both"))).lower()
        if raw_boundary not in ("both", boundary):
            continue
        start = max(0.0, min(1.0, float(raw.get("start_fraction", 0.0))))
        end = max(0.0, min(1.0, float(raw.get("end_fraction", 1.0))))
        if end > start + 1e-9:
            intervals.append((start, end))
    if not intervals:
        return 0.0
    intervals.sort()
    merged: list[tuple[float, float]] = []
    for start, end in intervals:
        if not merged or start > merged[-1][1]:
            merged.append((start, end))
        else:
            merged[-1] = (merged[-1][0], max(merged[-1][1], end))
    return sum(end - start for start, end in merged)


def _boundary_intentionally_open(data: dict,
                                 centerline: list[tuple[float, float, float]],
                                 segment_index: int,
                                 boundary: str) -> bool:
    if _break_coverage(data, segment_index, boundary) >= 0.99:
        return True
    if not bool(data.get("open_inside_hairpin_boundaries", False)):
        return False
    return _hairpin_inside_side(centerline, segment_index) == boundary


def generated_boundary_width_problems(course_path: Path) -> list[str]:
    data = load_yaml(course_path)
    if _uses_explicit_geometry(data):
        return []
    centerline = _centerline(data)
    if len(centerline) < 2:
        return []
    course = load_course(course_path)
    tapes = {tape.name: tape for tape in course.tapes}
    official = str(data.get("rules_profile", "")) == OFFICIAL_RULES_PROFILE
    minimum_required = OFFICIAL_TRACK_MIN_M if official else MIN_LEGAL_PASSAGE_M
    problems: list[str] = []

    for idx in range(len(centerline) - 1):
        left = tapes.get(f"left_boundary_{idx}")
        right = tapes.get(f"right_boundary_{idx}")
        if left is None or right is None:
            missing = []
            if left is None:
                missing.append("left")
            if right is None:
                missing.append("right")
            unexpected = [
                side for side in missing
                if not _boundary_intentionally_open(data, centerline, idx, side)
            ]
            if unexpected:
                problems.append(
                    f"missing generated boundary pair {idx} side(s) "
                    f"{','.join(unexpected)}")
            continue
        min_width = math.inf
        for sample in range(101):
            t = sample / 100.0
            lx = left.start[0] + (left.end[0] - left.start[0]) * t
            ly = left.start[1] + (left.end[1] - left.start[1]) * t
            rx = right.start[0] + (right.end[0] - right.start[0]) * t
            ry = right.start[1] + (right.end[1] - right.start[1]) * t
            min_width = min(min_width, math.hypot(lx - rx, ly - ry))
        if min_width + GENERATED_BOUNDARY_WIDTH_TOL_M < minimum_required:
            problems.append(
                f"generated boundary segment {idx} pinches to "
                f"{min_width / FT_TO_M:.2f}ft; required at least "
                f"{minimum_required / FT_TO_M:.2f}ft")
    if not official:
        return problems
    for idx in range(1, len(centerline) - 1):
        prev_x, prev_y, _ = centerline[idx - 1]
        cur_x, cur_y, cur_width = centerline[idx]
        next_x, next_y, _ = centerline[idx + 1]
        prev_len = math.hypot(cur_x - prev_x, cur_y - prev_y)
        next_len = math.hypot(next_x - cur_x, next_y - cur_y)
        if prev_len <= 1e-9 or next_len <= 1e-9:
            continue
        cross = ((cur_x - prev_x) * (next_y - cur_y)
                 - (cur_y - prev_y) * (next_x - cur_x))
        if abs(cross / (prev_len * next_len)) <= 1e-3:
            continue
        prev_left = tapes.get(f"left_boundary_{idx - 1}")
        prev_right = tapes.get(f"right_boundary_{idx - 1}")
        next_left = tapes.get(f"left_boundary_{idx}")
        next_right = tapes.get(f"right_boundary_{idx}")
        if (prev_left is None or prev_right is None or next_left is None
                or next_right is None):
            continue
        prev_ux = (cur_x - prev_x) / prev_len
        prev_uy = (cur_y - prev_y) / prev_len
        next_ux = (next_x - cur_x) / next_len
        next_uy = (next_y - cur_y) / next_len
        expected_trim = 0.5 * cur_width
        prev_left_trim = -((prev_left.end[0] - cur_x) * prev_ux
                           + (prev_left.end[1] - cur_y) * prev_uy)
        prev_right_trim = -((prev_right.end[0] - cur_x) * prev_ux
                            + (prev_right.end[1] - cur_y) * prev_uy)
        next_left_trim = ((next_left.start[0] - cur_x) * next_ux
                          + (next_left.start[1] - cur_y) * next_uy)
        next_right_trim = ((next_right.start[0] - cur_x) * next_ux
                           + (next_right.start[1] - cur_y) * next_uy)
        nearest_trim = min(prev_left_trim, prev_right_trim, next_left_trim,
                           next_right_trim)
        if nearest_trim + GENERATED_BOUNDARY_WIDTH_TOL_M < expected_trim:
            problems.append(
                f"generated boundary corner {idx} is not trimmed away from "
                f"the adjacent long lane mouth; nearest endpoint is "
                f"{nearest_trim:.2f}m along the segment, expected about "
                f"{expected_trim:.2f}m")
    return problems


def hairpin_connector_problems(course_path: Path) -> list[str]:
    data = load_yaml(course_path)
    if not bool(data.get("open_inside_hairpin_boundaries", False)):
        return []
    centerline = _centerline(data)
    course = load_course(course_path)
    tape_names = {tape.name for tape in course.tapes}
    problems: list[str] = []
    for idx in range(len(centerline) - 1):
        inside_side = _hairpin_inside_side(centerline, idx)
        if inside_side is None:
            continue
        inside_name = f"{inside_side}_boundary_{idx}"
        if inside_name not in tape_names:
            problems.append(
                f"hairpin connector segment {idx} is missing internal tape "
                f"{inside_name}; end-cap lines are not contiguous")
    return problems


def line_less_gps_section_problems(course_path: Path) -> list[str]:
    data = load_yaml(course_path)
    sections = data.get("line_less_gps_sections", [])
    if not sections:
        return []
    waypoints = {
        str(raw.get("label", "")): raw
        for raw in data.get("mission_waypoints", [])
    }
    problems: list[str] = []
    for raw_section in sections:
        name = str(raw_section.get("name", "line_less_gps_section"))
        entry_label = str(raw_section.get("entry_waypoint", ""))
        exit_label = str(raw_section.get("exit_waypoint", ""))
        entry = waypoints.get(entry_label)
        exit_wp = waypoints.get(exit_label)
        if entry is None or exit_wp is None:
            problems.append(
                f"{name} references missing waypoint(s) "
                f"{entry_label}/{exit_label}")
            continue
        separation = math.hypot(
            float(exit_wp["x_m"]) - float(entry["x_m"]),
            float(exit_wp["y_m"]) - float(entry["y_m"]),
        )
        minimum = float(raw_section.get("minimum_waypoint_separation_m",
                                        5.0))
        if separation + 1e-6 < minimum:
            problems.append(
                f"{name} waypoint separation {separation:.2f}m below "
                f"required {minimum:.2f}m")
        for segment_index in raw_section.get("required_broken_segments", []):
            idx = int(segment_index)
            for boundary in ("left", "right"):
                coverage = _break_coverage(data, idx, boundary)
                if coverage < 0.60:
                    problems.append(
                        f"{name} {boundary} boundary segment {idx} has only "
                        f"{coverage * 100:.0f}% tape removed; expected a "
                        "line-less GPS field")
    return problems


def slalom_gate_problems(course_path: Path) -> list[str]:
    data = load_yaml(course_path)
    gates = data.get("slalom_gates", [])
    if not gates:
        return []
    obstacles = {
        str(raw.get("name", "")): raw
        for raw in data.get("obstacles", [])
    }
    problems: list[str] = []
    for raw_gate in gates:
        gate_name = str(raw_gate.get("name", "slalom_gate"))
        lower_name = str(raw_gate.get("lower", ""))
        upper_name = str(raw_gate.get("upper", ""))
        lower = obstacles.get(lower_name)
        upper = obstacles.get(upper_name)
        if lower is None or upper is None:
            problems.append(
                f"{gate_name} references missing cone pair "
                f"{lower_name}/{upper_name}")
            continue
        lower_kind = str(lower.get("type", lower.get("kind", "")))
        upper_kind = str(upper.get("type", upper.get("kind", "")))
        if lower_kind != "cone" or upper_kind != "cone":
            problems.append(f"{gate_name} must reference cone obstacles")
        dx = float(upper["x_m"]) - float(lower["x_m"])
        dy = float(upper["y_m"]) - float(lower["y_m"])
        center_distance = math.hypot(dx, dy)
        clear = center_distance - float(lower["radius_m"]) - float(
            upper["radius_m"])
        required = float(raw_gate.get("minimum_clear_m", 6.0 * FT_TO_M))
        if clear + 1e-6 < required:
            problems.append(
                f"{gate_name} clear opening {clear / FT_TO_M:.2f}ft below "
                f"required {required / FT_TO_M:.2f}ft")
    return problems


def legal_passage_problems(course_path: Path) -> list[str]:
    data = load_yaml(course_path)
    if _uses_explicit_geometry(data):
        return []
    centerline = _centerline(data)
    if len(centerline) < 2:
        return []
    problems = []
    for raw in data.get("obstacles", []):
        x = float(raw["x_m"])
        y = float(raw["y_m"])
        radius_m = float(raw["radius_m"])
        best_gap, left_gap, right_gap = _nearest_centerline_gap(
            centerline, x, y, radius_m)
        if min(left_gap, right_gap) < -1e-6:
            problems.append(
                f"{raw.get('name', 'obstacle')} overlaps or sits outside a "
                f"nearest lane boundary (left={left_gap:.2f}m "
                f"right={right_gap:.2f}m)")
        if best_gap + 1e-6 < MIN_LEGAL_PASSAGE_M:
            problems.append(
                f"{raw.get('name', 'obstacle')} leaves no 5 ft passage "
                f"(best={best_gap:.2f}m left={left_gap:.2f}m "
                f"right={right_gap:.2f}m)")
    return problems


def official_rules_profile_problems(course_path: Path) -> list[str]:
    data = load_yaml(course_path)
    if str(data.get("rules_profile", "")) != OFFICIAL_RULES_PROFILE:
        return []
    course = load_course(course_path)
    centerline = _centerline(data)
    problems: list[str] = []

    length = _centerline_length(centerline)
    if not (OFFICIAL_MIN_LENGTH_M <= length <= OFFICIAL_MAX_LENGTH_M):
        problems.append(
            f"official course length {length / FT_TO_M:.0f}ft outside "
            f"{OFFICIAL_MIN_LENGTH_M / FT_TO_M:.0f}-"
            f"{OFFICIAL_MAX_LENGTH_M / FT_TO_M:.0f}ft acceptance band")

    min_x, min_y, max_x, max_y = _exact_bounds(course)
    spans = sorted((max_x - min_x, max_y - min_y), reverse=True)
    if spans[0] > OFFICIAL_AREA_LONG_M + OFFICIAL_AREA_TOL_M or (
            spans[1] > OFFICIAL_AREA_SHORT_M + OFFICIAL_AREA_TOL_M):
        problems.append(
            f"official course footprint {spans[0] / FT_TO_M:.0f}ft x "
            f"{spans[1] / FT_TO_M:.0f}ft exceeds 120ft x 100ft envelope")

    for idx, (_, _, width) in enumerate(centerline):
        if width < OFFICIAL_TRACK_MIN_M - 1e-6 or (
                width > OFFICIAL_TRACK_MAX_M + 1e-6):
            problems.append(
                f"centerline station {idx} width {width / FT_TO_M:.1f}ft "
                "outside 10-20ft rule range")

    for idx in range(1, len(centerline) - 1):
        radius = _turn_radius(centerline[idx - 1], centerline[idx],
                              centerline[idx + 1])
        if radius + 1e-6 < OFFICIAL_TURN_RADIUS_MIN_M:
            problems.append(
                f"turn at centerline station {idx} radius "
                f"{radius / FT_TO_M:.1f}ft below 5ft minimum")

    tape_width = float(data.get("tape_width_m", 0.0))
    if abs(tape_width - OFFICIAL_TAPE_WIDTH_M) > OFFICIAL_TAPE_TOL_M:
        problems.append(
            f"tape width {tape_width / IN_TO_M:.2f}in outside "
            "3in +/-0.25in rule band")

    if len(course.mission_waypoints) != 4:
        problems.append(
            f"official course must expose exactly 4 mission waypoints, got "
            f"{len(course.mission_waypoints)}")
    for waypoint in course.mission_waypoints:
        if waypoint.kind != "gps":
            problems.append(
                f"official waypoint {waypoint.label} is {waypoint.kind}, "
                "expected gps")

    obstacle_kinds = {obstacle.kind for obstacle in course.obstacles}
    if "cone" not in obstacle_kinds:
        problems.append("official course must include cone obstacles")
    if "barrel" not in obstacle_kinds:
        problems.append("official course must include barrel obstacles")
    if not course.ramps:
        problems.append("official course must include a ramp")
    if not data.get("line_less_gps_sections", []):
        problems.append(
            "official course must include a line-less GPS waypoint section")

    for ramp in course.ramps:
        half_run = 0.5 * (ramp.end_x_m - ramp.start_x_m)
        if half_run <= 1e-6:
            problems.append(f"ramp {ramp.name} has non-positive half-run")
            continue
        if ramp.width_m < OFFICIAL_TRACK_MIN_M - 1e-6:
            problems.append(
                f"ramp {ramp.name} width {ramp.width_m / FT_TO_M:.1f}ft "
                "below the 10ft lane width; real ramp lanes have white lines")
        grade = ramp.rise_m / half_run
        if grade > OFFICIAL_MAX_RAMP_GRADE + 1e-6:
            problems.append(
                f"ramp {ramp.name} grade {grade * 100:.1f}% exceeds 15%")

    speed = course.speed_check
    if abs(speed.end_distance_m - OFFICIAL_SPEED_DISTANCE_M) > 0.25:
        problems.append(
            f"speed check distance {speed.end_distance_m / FT_TO_M:.1f}ft "
            "is not the official first-44-ft gate")
    if speed.minimum_average_mps < OFFICIAL_MIN_SPEED_MPS - 1e-6:
        problems.append("minimum speed gate is below 1 mph")
    if speed.maximum_speed_mps > OFFICIAL_MAX_SPEED_MPS + 1e-6:
        problems.append("maximum speed gate exceeds 5 mph")

    return problems


def generated_world_problems(course_path: Path) -> list[str]:
    if course_path.parent.name != "courses":
        return []
    world_path = course_path.parent / "worlds" / f"{course_path.stem}.sdf"
    if not world_path.is_file():
        return [f"generated world missing: {world_path}"]
    try:
        root = ET.parse(world_path).getroot()
    except ET.ParseError as exc:
        return [f"generated world XML parse failed: {exc}"]

    course = load_course(course_path)
    model_names = {
        model.attrib.get("name", "")
        for model in root.findall(".//model")
    }

    shogi = None
    for model in root.findall(".//model"):
        if model.attrib.get("name") == "shogi":
            shogi = model
            break
    if shogi is None:
        return ["generated world missing shogi model"]

    problems: list[str] = []

    def _pose_xyz(element: ET.Element | None) -> tuple[float, float, float]:
        if element is None or element.text is None:
            return 0.0, 0.0, 0.0
        parts = [float(part) for part in element.text.split()[:3]]
        while len(parts) < 3:
            parts.append(0.0)
        return parts[0], parts[1], parts[2]

    def _link_cg(link_name: str) -> tuple[float, float, float, float] | None:
        link = shogi.find(f"./link[@name='{link_name}']")
        if link is None:
            return None
        mass_node = link.find("./inertial/mass")
        if mass_node is None or mass_node.text is None:
            return None
        mass = float(mass_node.text)
        lx, ly, lz = _pose_xyz(link.find("./pose"))
        ix, iy, iz = _pose_xyz(link.find("./inertial/pose"))
        return mass, lx + ix, ly + iy, lz + iz

    base_link = shogi.find("./link[@name='base_link']")
    caster_link = shogi.find("./link[@name='Caster_link']")
    if base_link is None:
        problems.append("generated world missing base_link")
    else:
        base_mesh_uri = base_link.find(
            "./visual[@name='base_link_mesh']/geometry/mesh/uri")
        if base_mesh_uri is None or (
                base_mesh_uri.text or "").strip() != (
                    "model://bringup/description/meshes/base_link.STL"):
            problems.append(
                "generated world base visual is not derived from "
                "bringup/description/meshes/base_link.STL")
    if caster_link is None:
        problems.append(
            "generated world missing Caster_link Chaplygin-sleigh support")
    else:
        caster_mesh_uri = caster_link.find(
            "./visual[@name='caster_mesh']/geometry/mesh/uri")
        if caster_mesh_uri is not None:
            problems.append(
                "generated world still uses fixed Caster_link.STL; the "
                "canonical Gazebo model should use a passive swivel + rolling "
                "caster wheel because the single caster mesh cannot turn")
    caster_swivel = shogi.find("./joint[@name='Caster_Swivel']")
    if caster_swivel is None or caster_swivel.attrib.get("type") != "revolute":
        problems.append("generated world missing passive Caster_Swivel joint")
    else:
        child = caster_swivel.find("./child")
        if child is None or (child.text or "").strip() != "Caster_link":
            problems.append("Caster_Swivel should drive Caster_link directly")
        axis = caster_swivel.find("./axis/xyz")
        xyz = " ".join((axis.text or "").split()) if axis is not None else ""
        if xyz != "0 0 1":
            problems.append(
                f"Caster_Swivel axis is {xyz or 'missing'}, expected 0 0 1")
    caster_wheel = shogi.find("./link[@name='caster_wheel_link']")
    if caster_wheel is None:
        problems.append("generated world missing caster_wheel_link")
    else:
        caster_wheel_collision = caster_wheel.find(
            "./collision[@name='caster_wheel_collision']")
        if caster_wheel_collision is None:
            problems.append(
                "generated world missing rolling caster wheel collision")
    caster_roll = shogi.find("./joint[@name='Caster_Wheel_Roll']")
    if caster_roll is None or caster_roll.attrib.get("type") != "revolute":
        problems.append("generated world missing Caster_Wheel_Roll joint")
    else:
        child = caster_roll.find("./child")
        if child is None or (child.text or "").strip() != "caster_wheel_link":
            problems.append(
                "Caster_Wheel_Roll should drive caster_wheel_link directly")
        axis = caster_roll.find("./axis/xyz")
        xyz = " ".join((axis.text or "").split()) if axis is not None else ""
        if xyz != "0 1 0":
            problems.append(
                f"Caster_Wheel_Roll axis is {xyz or 'missing'}, expected "
                "0 1 0 for the lateral caster axle")
    for link_name, visual_name, mesh_name in (
            ("left_wheel_link", "left_wheel_mesh", "Left_Wheel_Link.STL"),
            ("right_wheel_link", "right_wheel_mesh", "Right_Wheel_Link.STL"),
    ):
        wheel_link = shogi.find(f"./link[@name='{link_name}']")
        if wheel_link is None:
            problems.append(f"generated world missing {link_name}")
            continue
        wheel_mesh_uri = wheel_link.find(
            f"./visual[@name='{visual_name}']/geometry/mesh/uri")
        if wheel_mesh_uri is None or (
                wheel_mesh_uri.text or "").strip() != (
                    f"model://bringup/description/meshes/{mesh_name}"):
            problems.append(
                f"generated world {link_name} visual is not derived from "
                f"bringup/description/meshes/{mesh_name}")

    for ramp in course.ramps:
        expected = (
            f"{ramp.name}_up",
            f"{ramp.name}_down",
            f"{ramp.name}_up_left_white_line",
            f"{ramp.name}_up_right_white_line",
            f"{ramp.name}_down_left_white_line",
            f"{ramp.name}_down_right_white_line",
        )
        for model_name in expected:
            if model_name not in model_names:
                problems.append(f"generated world missing {model_name}")

    for joint_name in ("Left_Wheel", "Right_Wheel"):
        joint = shogi.find(f"./joint[@name='{joint_name}']")
        if joint is None:
            problems.append(f"generated world missing {joint_name} joint")
            continue
        axis = joint.find("./axis/xyz")
        xyz = " ".join((axis.text or "").split()) if axis is not None else ""
        if xyz != "0 0 -1":
            problems.append(
                f"{joint_name} axis is {xyz or 'missing'}, expected 0 0 -1 "
                "because wheel links are rolled 90deg; 0 1 0 makes the "
                "rendered wheels spin about the vertical axis")

    cg_terms = [
        _link_cg("base_link"),
        _link_cg("left_wheel_link"),
        _link_cg("right_wheel_link"),
        _link_cg("Caster_link"),
        _link_cg("caster_wheel_link"),
    ]
    if any(term is None for term in cg_terms):
        problems.append("generated world cannot verify measured robot CG")
    else:
        mass = sum(term[0] for term in cg_terms if term is not None)
        cg_x = sum(term[0] * term[1] for term in cg_terms
                   if term is not None) / mass
        cg_y = sum(term[0] * term[2] for term in cg_terms
                   if term is not None) / mass
        cg_z = sum(term[0] * term[3] for term in cg_terms
                   if term is not None) / mass
        expected_x = ROBOT_CG_FORWARD_OF_DRIVE_AXLE_M
        expected_y = 0.0
        expected_z = ROBOT_CG_HEIGHT_M - course.robot.wheel_radius_m
        if abs(mass - ROBOT_TOTAL_MASS_KG) > 0.05:
            problems.append(
                f"generated world mass {mass:.2f}kg does not match "
                "measured 117 lb robot")
        if math.hypot(cg_x - expected_x, cg_y - expected_y) > 0.01 or (
                abs(cg_z - expected_z) > 0.01):
            problems.append(
                f"generated world CG ({cg_x:.3f},{cg_y:.3f},{cg_z:.3f}) "
                f"relative to drive axle does not match measured "
                f"({expected_x:.3f},{expected_y:.3f},{expected_z:.3f})")
    return problems


def validate(course_path: Path) -> tuple[bool, str]:
    c = load_course(course_path)
    min_x, min_y, max_x, max_y = course_bounds(c, margin_m=2.0)
    nx = int(math.ceil((max_x - min_x) / RES))
    ny = int(math.ceil((max_y - min_y) / RES))
    lethal = np.zeros((ny, nx), dtype=bool)

    def w2c(x: float, y: float) -> tuple[int, int]:
        return (int((x - min_x) / RES), int((y - min_y) / RES))

    def stamp(x: float, y: float, radius_m: float) -> None:
        cx, cy = w2c(x, y)
        r = max(1, int(round(radius_m / RES)))
        x0, x1 = max(0, cx - r), min(nx, cx + r + 1)
        y0, y1 = max(0, cy - r), min(ny, cy + r + 1)
        if x0 >= x1 or y0 >= y1:
            return
        ys, xs = np.ogrid[y0:y1, x0:x1]
        lethal[y0:y1, x0:x1] |= ((xs - cx) ** 2 + (ys - cy) ** 2) <= r * r

    # tapes (sample along each segment)
    for t in c.tapes:
        (ax, ay), (bx, by) = t.start, t.end
        length = math.hypot(bx - ax, by - ay)
        n = max(1, int(length / (RES * 0.5)))
        for i in range(n + 1):
            s = i / n
            stamp(ax + (bx - ax) * s, ay + (by - ay) * s, t.width_m * 0.5)
    for o in c.obstacles:
        stamp(o.center[0], o.center[1], o.radius_m)

    # inflate by inscribed radius (half-width + padding); the robot cannot be
    # closer than this to any lethal cell without a footprint violation.
    r_in = c.robot.physical_half_width_m + c.robot.footprint_padding_m
    inflated = binary_dilation(lethal, structure=_disk(round(r_in / RES)))
    free = ~inflated

    labels, _ = label(free)

    def cell_ok(x: float, y: float, name: str) -> tuple[bool, int, str]:
        cx, cy = w2c(x, y)
        if not (0 <= cx < nx and 0 <= cy < ny):
            return False, -1, f"{name} out of bounds"
        if inflated[cy, cx]:
            return False, -1, f"{name} ({x:.2f},{y:.2f}) inside lethal/inflated"
        return True, labels[cy, cx], ""

    ok_s, start_lab, msg_s = cell_ok(c.start.x, c.start.y, "start")
    if not ok_s:
        return False, msg_s
    problems = []
    targets = [(wp.label, wp.x_m, wp.y_m) for wp in c.mission_waypoints]
    targets.append(("finish", c.finish[0], c.finish[1]))
    for name, x, y in targets:
        ok, lab, msg = cell_ok(x, y, name)
        if not ok:
            problems.append(msg)
        elif lab != start_lab:
            problems.append(f"{name} ({x:.2f},{y:.2f}) not connected to start")
    problems.extend(legal_passage_problems(course_path))
    problems.extend(generated_boundary_width_problems(course_path))
    problems.extend(hairpin_connector_problems(course_path))
    problems.extend(line_less_gps_section_problems(course_path))
    problems.extend(slalom_gate_problems(course_path))
    problems.extend(ramp_geometry_problems(course_path))
    problems.extend(official_rules_profile_problems(course_path))
    problems.extend(generated_world_problems(course_path))
    free_frac = float(free.mean())
    detail = (f"grid={nx}x{ny} r_in={r_in:.2f}m free={free_frac*100:.0f}% "
              f"wp={len(c.mission_waypoints)}")
    if problems:
        return False, detail + " | " + "; ".join(problems)
    return True, detail + " | all waypoints + finish reachable from start"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("courses", nargs="+")
    args = ap.parse_args()
    all_ok = True
    print("=== COURSE FEASIBILITY VALIDATION (padded-robot connectivity) ===")
    for cp in args.courses:
        try:
            ok, msg = validate(Path(cp))
        except Exception as exc:  # noqa: BLE001
            ok, msg = False, f"EXCEPTION: {exc}"
        all_ok &= ok
        print(f"[{'PASS' if ok else 'FAIL'}] {Path(cp).stem}: {msg}")
    print("RESULT: " + ("ALL PASS" if all_ok else "SOME FAILED"))
    return 0 if all_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())

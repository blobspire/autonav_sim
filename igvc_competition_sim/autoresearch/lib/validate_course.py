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


def legal_passage_problems(course_path: Path) -> list[str]:
    data = load_yaml(course_path)
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

    for ramp in course.ramps:
        run = ramp.end_x_m - ramp.start_x_m
        if run <= 1e-6:
            problems.append(f"ramp {ramp.name} has non-positive run")
            continue
        if ramp.width_m < OFFICIAL_TRACK_MIN_M - 1e-6:
            problems.append(
                f"ramp {ramp.name} width {ramp.width_m / FT_TO_M:.1f}ft "
                "below the 10ft lane width; real ramp lanes have white lines")
        grade = ramp.rise_m / run
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
    problems.extend(official_rules_profile_problems(course_path))
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

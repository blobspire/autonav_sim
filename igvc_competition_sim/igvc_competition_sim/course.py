from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, Iterable

from .robot_profile import RobotSpec


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
EARTH_RADIUS_M = 6378137.0


def _default_course_config() -> Path:
    source_tree = PACKAGE_ROOT / "config" / "igvc_competition_compact.yaml"
    if source_tree.is_file():
        return source_tree
    try:
        from ament_index_python.packages import get_package_share_directory
    except ImportError:
        return source_tree
    share_tree = (
        Path(get_package_share_directory("igvc_competition_sim"))
        / "config"
        / "igvc_competition_compact.yaml"
    )
    return share_tree


DEFAULT_COURSE_CONFIG = _default_course_config()


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float = 0.0


@dataclass(frozen=True)
class TapeSegment:
    name: str
    start: tuple[float, float]
    end: tuple[float, float]
    width_m: float


@dataclass(frozen=True)
class Obstacle:
    name: str
    kind: str
    center: tuple[float, float]
    radius_m: float
    height_m: float


@dataclass(frozen=True)
class Ramp:
    name: str
    start_x_m: float
    end_x_m: float
    center_y_m: float
    width_m: float
    rise_m: float
    center_x_m: float | None = None
    yaw_rad: float = 0.0
    run_length_m: float | None = None


@dataclass(frozen=True)
class MissionWaypoint:
    label: str
    kind: str
    x_m: float
    y_m: float
    radius_m: float


@dataclass(frozen=True)
class AnalysisStation:
    label: str
    x_m: float
    y_min_m: float
    y_max_m: float


@dataclass(frozen=True)
class SpeedCheck:
    start_x_m: float
    end_distance_m: float
    minimum_average_mps: float
    maximum_speed_mps: float
    blocking_stop_s: float
    blocking_speed_mps: float = 0.02
    blocking_progress_radius_m: float = 0.25


@dataclass(frozen=True)
class Course:
    course_id: str
    description: str
    config_path: Path
    datum_latitude_deg: float
    datum_longitude_deg: float
    datum_altitude_m: float
    start: Pose2D
    finish: tuple[float, float, float]
    robot: RobotSpec
    tapes: tuple[TapeSegment, ...]
    obstacles: tuple[Obstacle, ...]
    ramps: tuple[Ramp, ...]
    mission_waypoints: tuple[MissionWaypoint, ...]
    analysis_stations: tuple[AnalysisStation, ...]
    speed_check: SpeedCheck


def load_yaml(path: Path) -> dict[str, Any]:
    try:
        import yaml  # type: ignore
    except ImportError:
        return _load_limited_yaml(path)
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"Course config {path} did not load as a mapping")
    return data


def _strip_comment(line: str) -> str:
    in_single = False
    in_double = False
    for idx, ch in enumerate(line):
        if ch == "'" and not in_double:
            in_single = not in_single
        elif ch == '"' and not in_single:
            in_double = not in_double
        elif ch == "#" and not in_single and not in_double:
            return line[:idx]
    return line


def _limited_yaml_lines(path: Path) -> list[tuple[int, str]]:
    lines: list[tuple[int, str]] = []
    for raw in path.read_text(encoding="utf-8").splitlines():
        without_comment = _strip_comment(raw).rstrip()
        if not without_comment.strip():
            continue
        indent = len(without_comment) - len(without_comment.lstrip(" "))
        lines.append((indent, without_comment.strip()))
    return lines


def _split_top_level(raw: str, delimiter: str) -> list[str]:
    parts: list[str] = []
    start = 0
    depth = 0
    in_single = False
    in_double = False
    for idx, ch in enumerate(raw):
        if ch == "'" and not in_double:
            in_single = not in_single
        elif ch == '"' and not in_single:
            in_double = not in_double
        elif not in_single and not in_double:
            if ch in "[{(":
                depth += 1
            elif ch in "]})":
                depth -= 1
            elif ch == delimiter and depth == 0:
                parts.append(raw[start:idx].strip())
                start = idx + 1
    parts.append(raw[start:].strip())
    return [part for part in parts if part]


def _parse_scalar(raw: str) -> Any:
    value = raw.strip()
    if value == "":
        return ""
    if value.startswith("{") and value.endswith("}"):
        result: dict[str, Any] = {}
        inner = value[1:-1].strip()
        if not inner:
            return result
        for item in _split_top_level(inner, ","):
            key, val = item.split(":", 1)
            result[key.strip()] = _parse_scalar(val)
        return result
    if value.startswith("[") and value.endswith("]"):
        inner = value[1:-1].strip()
        return [_parse_scalar(item) for item in _split_top_level(inner, ",")]
    if (value.startswith('"') and value.endswith('"')) or (
            value.startswith("'") and value.endswith("'")):
        return value[1:-1]
    lower = value.lower()
    if lower == "true":
        return True
    if lower == "false":
        return False
    try:
        if any(ch in value for ch in (".", "e", "E")):
            return float(value)
        return int(value)
    except ValueError:
        return value


def _parse_mapping(lines: list[tuple[int, str]],
                   idx: int,
                   indent: int) -> tuple[dict[str, Any], int]:
    out: dict[str, Any] = {}
    while idx < len(lines):
        line_indent, text = lines[idx]
        if line_indent < indent:
            break
        if line_indent > indent:
            raise ValueError(f"Unexpected indentation near: {text}")
        if text.startswith("- "):
            break
        if ":" not in text:
            raise ValueError(f"Expected mapping entry near: {text}")
        key, raw_value = text.split(":", 1)
        key = key.strip()
        raw_value = raw_value.strip()
        idx += 1
        if raw_value:
            out[key] = _parse_scalar(raw_value)
            continue
        if idx >= len(lines) or lines[idx][0] <= indent:
            out[key] = {}
            continue
        child_indent = lines[idx][0]
        if lines[idx][1].startswith("- "):
            out[key], idx = _parse_list(lines, idx, child_indent)
        else:
            out[key], idx = _parse_mapping(lines, idx, child_indent)
    return out, idx


def _parse_list(lines: list[tuple[int, str]],
                idx: int,
                indent: int) -> tuple[list[Any], int]:
    out: list[Any] = []
    while idx < len(lines):
        line_indent, text = lines[idx]
        if line_indent < indent:
            break
        if line_indent != indent or not text.startswith("- "):
            break
        item_text = text[2:].strip()
        idx += 1
        if item_text.startswith("{"):
            out.append(_parse_scalar(item_text))
            continue
        if item_text and ":" in item_text:
            key, raw_value = item_text.split(":", 1)
            item: dict[str, Any] = {
                key.strip(): _parse_scalar(raw_value.strip())
            }
            if idx < len(lines) and lines[idx][0] > indent:
                more, idx = _parse_mapping(lines, idx, lines[idx][0])
                item.update(more)
            out.append(item)
            continue
        if idx < len(lines) and lines[idx][0] > indent:
            if lines[idx][1].startswith("- "):
                value, idx = _parse_list(lines, idx, lines[idx][0])
            else:
                value, idx = _parse_mapping(lines, idx, lines[idx][0])
            out.append(value)
        else:
            out.append(_parse_scalar(item_text))
    return out, idx


def _load_limited_yaml(path: Path) -> dict[str, Any]:
    lines = _limited_yaml_lines(path)
    if not lines:
        return {}
    data, idx = _parse_mapping(lines, 0, lines[0][0])
    if idx != len(lines):
        raise ValueError(f"Could not parse all of {path}")
    return data


def _as_path(path: str | Path | None) -> Path:
    if path is None or str(path).strip() in ("", "__auto__"):
        return DEFAULT_COURSE_CONFIG
    return Path(path).expanduser().resolve()


def _point(raw: dict[str, Any]) -> tuple[float, float]:
    return float(raw["x_m"]), float(raw["y_m"])


def _segment_normal(start: tuple[float, float, float],
                    end: tuple[float, float, float]) -> tuple[float, float]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length = math.hypot(dx, dy)
    if length <= 1e-9:
        return 0.0, 1.0
    return -dy / length, dx / length


def _corner_cross(points: list[tuple[float, float, float]], idx: int) -> float:
    prev = points[idx - 1]
    current = points[idx]
    next_point = points[idx + 1]
    prev_dx = current[0] - prev[0]
    prev_dy = current[1] - prev[1]
    next_dx = next_point[0] - current[0]
    next_dy = next_point[1] - current[1]
    return prev_dx * next_dy - prev_dy * next_dx


def _is_corner(points: list[tuple[float, float, float]], idx: int) -> bool:
    if idx <= 0 or idx >= len(points) - 1:
        return False
    prev = points[idx - 1]
    current = points[idx]
    next_point = points[idx + 1]
    prev_dx = current[0] - prev[0]
    prev_dy = current[1] - prev[1]
    next_dx = next_point[0] - current[0]
    next_dy = next_point[1] - current[1]
    prev_len = math.hypot(prev_dx, prev_dy)
    next_len = math.hypot(next_dx, next_dy)
    if prev_len <= 1e-9 or next_len <= 1e-9:
        return False
    cross = prev_dx * next_dy - prev_dy * next_dx
    return abs(cross / (prev_len * next_len)) > 1e-3


def _hairpin_inside_side(points: list[tuple[float, float, float]],
                         idx: int) -> str | None:
    if idx <= 0 or idx >= len(points) - 2:
        return None
    previous = points[idx - 1]
    start = points[idx]
    end = points[idx + 1]
    next_point = points[idx + 2]

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


def _line_breaks_for(data: dict[str, Any],
                     idx: int,
                     side: str) -> list[tuple[float, float]]:
    breaks: list[tuple[float, float]] = []
    for raw in data.get("line_breaks", []):
        if int(raw.get("segment_index", -1)) != idx:
            continue
        boundary = str(raw.get("boundary", raw.get("side", "both"))).lower()
        if boundary not in ("both", side):
            continue
        start_fraction = max(0.0, min(1.0, float(
            raw.get("start_fraction", 0.0))))
        end_fraction = max(0.0, min(1.0, float(raw.get("end_fraction", 1.0))))
        if end_fraction > start_fraction + 1e-9:
            breaks.append((start_fraction, end_fraction))
    if not breaks:
        return []
    breaks.sort()
    merged: list[tuple[float, float]] = []
    for start, end in breaks:
        if not merged or start > merged[-1][1]:
            merged.append((start, end))
        else:
            merged[-1] = (merged[-1][0], max(merged[-1][1], end))
    return merged


def _append_tape_with_breaks(tapes: list[TapeSegment],
                             *,
                             name: str,
                             side: str,
                             idx: int,
                             start: tuple[float, float],
                             end: tuple[float, float],
                             width_m: float,
                             data: dict[str, Any]) -> bool:
    breaks = _line_breaks_for(data, idx, side)
    if not breaks:
        tapes.append(TapeSegment(
            name=name,
            start=start,
            end=end,
            width_m=width_m,
        ))
        return True

    dx = end[0] - start[0]
    dy = end[1] - start[1]
    cursor = 0.0
    emitted = False
    emitted_count = 0
    for break_start, break_end in breaks:
        if break_start > cursor + 1e-9:
            seg_start = (start[0] + dx * cursor, start[1] + dy * cursor)
            seg_end = (start[0] + dx * break_start,
                       start[1] + dy * break_start)
            tapes.append(TapeSegment(
                name=name if emitted_count == 0 else f"{name}_{emitted_count}",
                start=seg_start,
                end=seg_end,
                width_m=width_m,
            ))
            emitted = True
            emitted_count += 1
        cursor = max(cursor, break_end)
    if cursor < 1.0 - 1e-9:
        seg_start = (start[0] + dx * cursor, start[1] + dy * cursor)
        tapes.append(TapeSegment(
            name=name if emitted_count == 0 else f"{name}_{emitted_count}",
            start=seg_start,
            end=end,
            width_m=width_m,
        ))
        emitted = True
    return emitted


def _shortest_angle_delta(start: float, end: float) -> float:
    delta = (end - start + math.pi) % (2.0 * math.pi) - math.pi
    if abs(delta + math.pi) <= 1e-9:
        return math.pi
    return delta


def _directed_angle_delta(start: float, end: float, direction: int) -> float:
    if direction >= 0:
        delta = (end - start) % (2.0 * math.pi)
        return 0.0 if delta <= 1e-9 else delta
    delta = (start - end) % (2.0 * math.pi)
    return 0.0 if delta <= 1e-9 else -delta


def _arc_points(center: tuple[float, float],
                start: tuple[float, float],
                end: tuple[float, float],
                count: int,
                direction: int | None = None) -> list[tuple[float, float]]:
    cx, cy = center
    radius = math.hypot(start[0] - cx, start[1] - cy)
    start_angle = math.atan2(start[1] - cy, start[0] - cx)
    end_angle = math.atan2(end[1] - cy, end[0] - cx)
    delta = (
        _shortest_angle_delta(start_angle, end_angle)
        if direction is None
        else _directed_angle_delta(start_angle, end_angle, direction)
    )
    if radius <= 1e-9 or abs(delta) <= 1e-9:
        return [start, end]
    return [
        (
            cx + radius * math.cos(start_angle + delta * idx / count),
            cy + radius * math.sin(start_angle + delta * idx / count),
        )
        for idx in range(count + 1)
    ]


def _boundary_tapes(data: dict[str, Any],
                    tape_width_m: float) -> list[TapeSegment]:
    raw_centerline = data.get("centerline", [])
    points = [
        (float(p["x_m"]), float(p["y_m"]), float(p["width_m"]))
        for p in raw_centerline
    ]
    if len(points) < 2:
        raise ValueError("centerline must contain at least two stations")

    left_segments: list[tuple[tuple[float, float], tuple[float, float]]] = []
    right_segments: list[tuple[tuple[float, float], tuple[float, float]]] = []
    for idx in range(len(points) - 1):
        start = points[idx]
        end = points[idx + 1]
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length = math.hypot(dx, dy)
        if length <= 1e-9:
            continue
        ux = dx / length
        uy = dy / length
        nx, ny = -uy, ux
        start_trim = 0.5 * start[2] if _is_corner(points, idx) else 0.0
        end_trim = 0.5 * end[2] if _is_corner(points, idx + 1) else 0.0
        if start_trim + end_trim > length * 0.80:
            scale = length * 0.80 / (start_trim + end_trim)
            start_trim *= scale
            end_trim *= scale
        start_center = (start[0] + ux * start_trim,
                        start[1] + uy * start_trim)
        end_center = (end[0] - ux * end_trim,
                      end[1] - uy * end_trim)
        start_half = 0.5 * start[2]
        end_half = 0.5 * end[2]
        left_start = (start_center[0] + nx * start_half,
                      start_center[1] + ny * start_half)
        left_end = (end_center[0] + nx * end_half,
                    end_center[1] + ny * end_half)
        right_start = (start_center[0] - nx * start_half,
                       start_center[1] - ny * start_half)
        right_end = (end_center[0] - nx * end_half,
                     end_center[1] - ny * end_half)
        left_segments.append((left_start, left_end))
        right_segments.append((right_start, right_end))

    draw_left_segments: list[bool] = [True] * (len(points) - 1)
    draw_right_segments: list[bool] = [True] * (len(points) - 1)

    tapes: list[TapeSegment] = []
    for idx in range(len(points) - 1):
        if draw_left_segments[idx]:
            draw_left_segments[idx] = _append_tape_with_breaks(
                tapes,
                name=f"left_boundary_{idx}",
                side="left",
                idx=idx,
                start=left_segments[idx][0],
                end=left_segments[idx][1],
                width_m=tape_width_m,
                data=data,
            )
        if draw_right_segments[idx]:
            draw_right_segments[idx] = _append_tape_with_breaks(
                tapes,
                name=f"right_boundary_{idx}",
                side="right",
                idx=idx,
                start=right_segments[idx][0],
                end=right_segments[idx][1],
                width_m=tape_width_m,
                data=data,
            )
    for idx in range(1, len(points) - 1):
        center = points[idx][0], points[idx][1]
        turn_cross = _corner_cross(points, idx)
        left_start = left_segments[idx - 1][1]
        left_end = left_segments[idx][0]
        right_start = right_segments[idx - 1][1]
        right_end = right_segments[idx][0]
        left_start_angle = math.atan2(left_start[1] - center[1],
                                      left_start[0] - center[0])
        left_end_angle = math.atan2(left_end[1] - center[1],
                                    left_end[0] - center[0])
        right_start_angle = math.atan2(right_start[1] - center[1],
                                       right_start[0] - center[0])
        right_end_angle = math.atan2(right_end[1] - center[1],
                                     right_end[0] - center[0])
        left_direction = -1 if turn_cross < -1e-9 else None
        right_direction = 1 if turn_cross > 1e-9 else None
        left_delta = abs(
            _directed_angle_delta(left_start_angle, left_end_angle,
                                  left_direction)
            if left_direction is not None else
            _shortest_angle_delta(left_start_angle, left_end_angle))
        right_delta = abs(
            _directed_angle_delta(right_start_angle, right_end_angle,
                                  right_direction)
            if right_direction is not None else
            _shortest_angle_delta(right_start_angle, right_end_angle))
        delta = max(left_delta, right_delta)
        if delta <= 1e-6:
            continue
        segment_count = max(2, int(math.ceil(delta / (math.pi / 36.0))))
        left_arc = _arc_points(center, left_start, left_end, segment_count,
                               left_direction)
        right_arc = _arc_points(center, right_start, right_end, segment_count,
                                right_direction)
        # At a sharp centerline corner, the inside offset arc becomes a cross
        # tape through the drivable lane. Real IGVC courses have finite-radius
        # turns; for this polyline generator, keep only the outside join so
        # the generated tape never narrows the legal passage at switchbacks.
        draw_left_join = (
            turn_cross < -1e-9
            and draw_left_segments[idx - 1]
            and draw_left_segments[idx]
        )
        draw_right_join = (
            turn_cross > 1e-9
            and draw_right_segments[idx - 1]
            and draw_right_segments[idx]
        )
        for arc_idx in range(segment_count):
            if draw_left_join:
                tapes.append(TapeSegment(
                    name=f"left_boundary_join_{idx}_{arc_idx}",
                    start=left_arc[arc_idx],
                    end=left_arc[arc_idx + 1],
                    width_m=tape_width_m,
                ))
            if draw_right_join:
                tapes.append(TapeSegment(
                    name=f"right_boundary_join_{idx}_{arc_idx}",
                    start=right_arc[arc_idx],
                    end=right_arc[arc_idx + 1],
                    width_m=tape_width_m,
                ))
    return tapes


def _dashed_segments(raw: dict[str, Any],
                     default_width_m: float) -> list[TapeSegment]:
    start = _point(raw["start"])
    end = _point(raw["end"])
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length = math.hypot(dx, dy)
    if length <= 1e-9:
        return []
    ux = dx / length
    uy = dy / length
    dash = float(raw.get("dash_length_m", 0.75))
    gap = float(raw.get("gap_length_m", 0.45))
    width = float(raw.get("width_m", default_width_m))
    out: list[TapeSegment] = []
    offset = 0.0
    idx = 0
    while offset < length:
        seg_len = min(dash, length - offset)
        if seg_len > 1e-6:
            out.append(TapeSegment(
                name=f"{raw.get('name', 'dash')}_{idx}",
                start=(start[0] + ux * offset, start[1] + uy * offset),
                end=(start[0] + ux * (offset + seg_len),
                     start[1] + uy * (offset + seg_len)),
                width_m=width,
            ))
            idx += 1
        offset += dash + gap
    return out


def load_course(path: str | Path | None = None) -> Course:
    config_path = _as_path(path)
    data = load_yaml(config_path)
    tape_width = float(data.get("tape_width_m", 0.12))
    tapes = _boundary_tapes(data, tape_width)
    for raw in data.get("internal_lines", []):
        tapes.append(TapeSegment(
            name=str(raw.get("name", "internal_line")),
            start=_point(raw["start"]),
            end=_point(raw["end"]),
            width_m=float(raw.get("width_m", tape_width)),
        ))
    for raw in data.get("dashed_lines", []):
        tapes.extend(_dashed_segments(raw, tape_width))

    datum = data["datum"]
    start = data["start"]
    finish = data["finish"]
    robot = data["robot"]
    speed_check = data["speed_check"]

    return Course(
        course_id=str(data.get("course_id", config_path.stem)),
        description=str(data.get("description", "")),
        config_path=config_path,
        datum_latitude_deg=float(datum["latitude_deg"]),
        datum_longitude_deg=float(datum["longitude_deg"]),
        datum_altitude_m=float(datum.get("altitude_m", 0.0)),
        start=Pose2D(
            x=float(start.get("x_m", 0.0)),
            y=float(start.get("y_m", 0.0)),
            yaw=float(start.get("yaw_rad", 0.0)),
        ),
        finish=(
            float(finish["x_m"]),
            float(finish["y_m"]),
            float(finish.get("radius_m", 1.0)),
        ),
        robot=RobotSpec(**{key: float(value) for key, value in robot.items()}),
        tapes=tuple(tapes),
        obstacles=tuple(
            Obstacle(
                name=str(raw["name"]),
                kind=str(raw.get("type", "barrel")),
                center=(float(raw["x_m"]), float(raw["y_m"])),
                radius_m=float(raw["radius_m"]),
                height_m=float(raw["height_m"]),
            )
            for raw in data.get("obstacles", [])
        ),
        ramps=tuple(
            Ramp(
                name=str(raw["name"]),
                start_x_m=float(raw["start_x_m"]),
                end_x_m=float(raw["end_x_m"]),
                center_y_m=float(raw["center_y_m"]),
                width_m=float(raw["width_m"]),
                rise_m=float(raw["rise_m"]),
                center_x_m=float(raw["center_x_m"])
                if "center_x_m" in raw else None,
                yaw_rad=float(raw.get("yaw_rad", 0.0)),
                run_length_m=float(raw["run_length_m"])
                if "run_length_m" in raw else None,
            )
            for raw in data.get("ramps", [])
        ),
        mission_waypoints=tuple(
            MissionWaypoint(
                label=str(raw["label"]),
                kind=str(raw["type"]),
                x_m=float(raw["x_m"]),
                y_m=float(raw["y_m"]),
                radius_m=float(raw.get("radius_m", 1.0)),
            )
            for raw in data.get("mission_waypoints", [])
        ),
        analysis_stations=tuple(
            AnalysisStation(
                label=str(raw["label"]),
                x_m=float(raw["x_m"]),
                y_min_m=float(raw["y_min_m"]),
                y_max_m=float(raw["y_max_m"]),
            )
            for raw in data.get("analysis_stations", [])
        ),
        speed_check=SpeedCheck(
            start_x_m=float(speed_check["start_x_m"]),
            end_distance_m=float(speed_check["end_distance_m"]),
            minimum_average_mps=float(speed_check["minimum_average_mps"]),
            maximum_speed_mps=float(speed_check["maximum_speed_mps"]),
            blocking_stop_s=float(speed_check["blocking_stop_s"]),
            blocking_speed_mps=float(speed_check.get("blocking_speed_mps", 0.02)),
            blocking_progress_radius_m=float(
                speed_check.get("blocking_progress_radius_m", 0.25)),
        ),
    )


def local_to_latlon(x_m: float,
                    y_m: float,
                    datum_lat_deg: float,
                    datum_lon_deg: float) -> tuple[float, float]:
    datum_lat_rad = math.radians(datum_lat_deg)
    lat = datum_lat_deg + math.degrees(y_m / EARTH_RADIUS_M)
    lon = datum_lon_deg + math.degrees(
        x_m / (EARTH_RADIUS_M * math.cos(datum_lat_rad)))
    return lat, lon


def latlon_to_local(lat_deg: float,
                    lon_deg: float,
                    datum_lat_deg: float,
                    datum_lon_deg: float) -> tuple[float, float]:
    datum_lat_rad = math.radians(datum_lat_deg)
    x = math.radians(lon_deg - datum_lon_deg) * EARTH_RADIUS_M * math.cos(
        datum_lat_rad)
    y = math.radians(lat_deg - datum_lat_deg) * EARTH_RADIUS_M
    return x, y


def iter_course_points(course: Course) -> Iterable[tuple[float, float]]:
    yield course.start.x, course.start.y
    yield course.finish[0], course.finish[1]
    for tape in course.tapes:
        yield tape.start
        yield tape.end
    for obstacle in course.obstacles:
        yield obstacle.center
    for ramp in course.ramps:
        center_x = (
            ramp.center_x_m
            if ramp.center_x_m is not None
            else 0.5 * (ramp.start_x_m + ramp.end_x_m)
        )
        run_length = (
            ramp.run_length_m
            if ramp.run_length_m is not None
            else ramp.end_x_m - ramp.start_x_m
        )
        cos_yaw = math.cos(ramp.yaw_rad)
        sin_yaw = math.sin(ramp.yaw_rad)
        for local_x in (-0.5 * run_length, 0.5 * run_length):
            for local_y in (-0.5 * ramp.width_m, 0.5 * ramp.width_m):
                yield (
                    center_x + cos_yaw * local_x - sin_yaw * local_y,
                    ramp.center_y_m + sin_yaw * local_x + cos_yaw * local_y,
                )
    for waypoint in course.mission_waypoints:
        yield waypoint.x_m, waypoint.y_m


def course_bounds(course: Course, margin_m: float = 4.0
                  ) -> tuple[float, float, float, float]:
    points = tuple(iter_course_points(course))
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return (
        math.floor(min(xs) - margin_m),
        math.floor(min(ys) - margin_m),
        math.ceil(max(xs) + margin_m),
        math.ceil(max(ys) + margin_m),
    )

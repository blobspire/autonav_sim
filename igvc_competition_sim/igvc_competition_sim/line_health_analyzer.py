#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

from .course import DEFAULT_COURSE_CONFIG, Course, load_course


def _line_samples(course: Course, spacing_m: float = 0.10
                  ) -> list[tuple[float, float]]:
    samples: list[tuple[float, float]] = []
    for tape in course.tapes:
        ax, ay = tape.start
        bx, by = tape.end
        dx = bx - ax
        dy = by - ay
        length = math.hypot(dx, dy)
        if length <= 1e-6:
            continue
        steps = max(1, int(math.ceil(length / spacing_m)))
        for idx in range(steps + 1):
            t = idx / float(steps)
            samples.append((ax + dx * t, ay + dy * t))
    return samples


def _nearest_distance(point: tuple[float, float],
                      samples: list[tuple[float, float]]) -> float:
    return min(
        math.hypot(point[0] - sample[0], point[1] - sample[1])
        for sample in samples
    )


def _cost_counts(msg: Any) -> dict[str, int]:
    values = [int(v) for v in msg.data]
    return {
        "max": max(values, default=0),
        "ge_1": sum(1 for v in values if v >= 1),
        "ge_100": sum(1 for v in values if v >= 100),
        "ge_254": sum(1 for v in values if v >= 254),
    }


def analyze_line_health(bag_dir: Path,
                        course_config: Path = DEFAULT_COURSE_CONFIG
                        ) -> dict[str, Any]:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    course = load_course(course_config)
    truth_samples = _line_samples(course)
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    wanted = {"/line_points", "/line_costmap"}
    msg_types = {
        topic: get_message(topic_types[topic])
        for topic in wanted
        if topic in topic_types
    }

    first_ts: int | None = None
    line_messages = 0
    nonempty_messages = 0
    max_points = 0
    first_nonempty_rel_s: float | None = None
    nearest_distances: list[float] = []
    costmap_samples = 0
    max_cost_ge_1 = 0
    max_cost_ge_254 = 0

    while reader.has_next():
        topic, data, ts = reader.read_next()
        if topic not in msg_types:
            continue
        if first_ts is None:
            first_ts = ts
        rel_s = (ts - first_ts) / 1e9
        msg = deserialize_message(data, msg_types[topic])
        if topic == "/line_points":
            line_messages += 1
            points = [(float(p.x), float(p.y)) for p in msg.points]
            max_points = max(max_points, len(points))
            if points:
                nonempty_messages += 1
                if first_nonempty_rel_s is None:
                    first_nonempty_rel_s = rel_s
                sample_step = max(1, len(points) // 250)
                for point in points[::sample_step]:
                    nearest_distances.append(
                        _nearest_distance(point, truth_samples))
        elif topic == "/line_costmap":
            costmap_samples += 1
            counts = _cost_counts(msg)
            max_cost_ge_1 = max(max_cost_ge_1, counts["ge_1"])
            max_cost_ge_254 = max(max_cost_ge_254, counts["ge_254"])

    mean_distance = None
    p90_distance = None
    fraction_within_25cm = None
    if nearest_distances:
        ordered = sorted(nearest_distances)
        mean_distance = sum(ordered) / len(ordered)
        p90_distance = ordered[min(len(ordered) - 1, int(0.90 * len(ordered)))]
        fraction_within_25cm = (
            sum(1 for value in ordered if value <= 0.25) / len(ordered))

    return {
        "bag": str(bag_dir),
        "course_config": str(course_config),
        "truth_samples": len(truth_samples),
        "line_messages": line_messages,
        "nonempty_line_messages": nonempty_messages,
        "max_line_points": max_points,
        "first_nonempty_line_rel_s": (
            None if first_nonempty_rel_s is None
            else round(first_nonempty_rel_s, 3)),
        "mean_distance_to_truth_m": (
            None if mean_distance is None else round(mean_distance, 3)),
        "p90_distance_to_truth_m": (
            None if p90_distance is None else round(p90_distance, 3)),
        "fraction_within_25cm": (
            None if fraction_within_25cm is None
            else round(fraction_within_25cm, 3)),
        "line_costmap_samples": costmap_samples,
        "line_costmap_nonempty": max_cost_ge_1 > 0,
        "max_line_costmap_ge_1": max_cost_ge_1,
        "max_line_costmap_ge_254": max_cost_ge_254,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_or_bag", help="Run directory or rosbag directory")
    parser.add_argument("--course-config", default=str(DEFAULT_COURSE_CONFIG))
    parser.add_argument("--json-out", default="")
    args = parser.parse_args(argv)

    run_or_bag = Path(args.run_or_bag).expanduser()
    bag_dir = run_or_bag / "bag" if (run_or_bag / "bag").exists() else run_or_bag
    summary = analyze_line_health(
        bag_dir,
        Path(args.course_config).expanduser(),
    )
    print(json.dumps(summary, indent=2, sort_keys=True))
    if args.json_out:
        Path(args.json_out).expanduser().write_text(
            json.dumps(summary, indent=2, sort_keys=True),
            encoding="utf-8",
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

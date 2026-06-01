#!/usr/bin/env python3
"""Self-test for metrics.py + fitness.py using a synthetic rosbag2 (no sim).

Writes a small bag with standard message types exercising every metric path,
runs compute_metrics() + fitness.evaluate_candidate(), and asserts the results.
Run in a sourced ROS 2 Humble env: python3 selftest_metrics.py
Exit 0 = all asserts pass.
"""
from __future__ import annotations

import hashlib
import json
import shutil
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import metrics as M          # noqa: E402
import fitness as F          # noqa: E402

import rosbag2_py            # noqa: E402
from rclpy.serialization import serialize_message  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402
from action_msgs.msg import GoalStatusArray, GoalStatus, GoalInfo  # noqa: E402
from unique_identifier_msgs.msg import UUID  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from rcl_interfaces.msg import Log  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from builtin_interfaces.msg import Time  # noqa: E402

COURSE = str(HERE.parent / "courses" / "compact_baseline.yaml")
COURSE_SHA256 = hashlib.sha256(Path(COURSE).read_bytes()).hexdigest()
RUN_ID = "selftest-run"


def wall(sim_s: float) -> int:
    return int((sim_s + 1.0) * 1e9)   # wall = sim + 1.0 s (recoverable by mapper)


def time_msg(sim_s: float) -> Time:
    return Time(sec=int(sim_s), nanosec=int((sim_s % 1) * 1e9))


def make_status(status: int, sim_s: float, gid: int) -> GoalStatusArray:
    arr = GoalStatusArray()
    gs = GoalStatus()
    gi = GoalInfo()
    u = UUID()
    u.uuid = [gid % 256] * 16
    gi.goal_id = u
    gi.stamp = time_msg(sim_s)
    gs.goal_info = gi
    gs.status = status
    arr.status_list = [gs]
    return arr


def write_bag(run_dir: Path, *, fail: bool = False) -> None:
    bag = run_dir / "bag"
    if bag.exists():
        shutil.rmtree(bag)
    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
           rosbag2_py.ConverterOptions("cdr", "cdr"))
    topics = {
        "/clock": "rosgraph_msgs/msg/Clock",
        "/navigate_to_pose/_action/status": "action_msgs/msg/GoalStatusArray",
        "/cmd_vel": "geometry_msgs/msg/Twist",
        "/rosout": "rcl_interfaces/msg/Log",
        "/odom": "nav_msgs/msg/Odometry",
        "/igvc_sim/ground_truth_odom": "nav_msgs/msg/Odometry",
        "/igvc_sim/score": "std_msgs/msg/String",
        "/scan_pca_filtered": "sensor_msgs/msg/LaserScan",
        "/back_up/_action/status": "action_msgs/msg/GoalStatusArray",
    }
    for name, ty in topics.items():
        w.create_topic(rosbag2_py.TopicMetadata(
            name=name, type=ty, serialization_format="cdr"))

    def put(topic, msg, sim_s):
        w.write(topic, serialize_message(msg), wall(sim_s))

    # clock every 0.5 s over 0..20 s
    s = 0.0
    while s <= 20.0:
        c = Clock()
        c.clock = time_msg(s)
        put("/clock", c, s)
        s += 0.5

    # nav action: EXECUTING @2s, SUCCEEDED @18s
    put("/navigate_to_pose/_action/status", make_status(2, 2.0, 1), 2.0)
    put("/navigate_to_pose/_action/status", make_status(4, 18.0, 1), 18.0)

    # one backup activation
    put("/back_up/_action/status", make_status(1, 9.0, 7), 9.0)

    # cmd_vel: forward with angular sign reversals, plus a slow gap 10..12s
    t = 2.0
    ang = 0.3
    while t <= 18.0:
        tw = Twist()
        if 10.0 <= t < 12.0:
            tw.linear.x = 0.0     # stuck gap (>1s) while not at goal
            tw.angular.z = 0.0
        else:
            tw.linear.x = 0.25
            ang = -ang            # sign flip every sample
            tw.angular.z = ang
        put("/cmd_vel", tw, t)
        t += 0.25

    # rosout: one each breadcrumb-reverse, gradient-start, PFS reject
    for sim_s, txt in [
        (5.0, "BreadcrumbReverse: reversing to breadcrumb (1.00, 2.00) in frame 'odom'"),
        (6.0, "GradientEscape: starting (threshold=200, speed=0.10, timeout=15.0s)"),
        (7.0, "PathFootprintSafe: rejecting path at pose 3/40 (1.20, 0.30); footprint overlaps"),
    ]:
        lg = Log()
        lg.stamp = time_msg(sim_s)
        lg.msg = txt
        put("/rosout", lg, sim_s)

    # odom: drive forward along lane center from (0,0); stays clear of tape
    s = 2.0
    x = 0.0
    while s <= 18.0:
        od = Odometry()
        od.pose.pose.position.x = x
        od.pose.pose.position.y = 0.0
        od.pose.pose.orientation.w = 1.0
        put("/odom", od, s)
        put("/igvc_sim/ground_truth_odom", od, s)
        x += 0.1
        s += 0.5

    # pca scan present at 3s
    put("/scan_pca_filtered", LaserScan(), 3.0)

    # score
    score = {
        "score_schema_version": 4,
        "scoring_mode": "ground_truth_odom_swept_footprint_v4",
        "run_id": RUN_ID,
        "course_id": "compact_baseline",
        "course_config_sha256": COURSE_SHA256,
        "failed": fail,
        "failures": (["tape_crossing:left_boundary_3"] if fail else []),
        "distance_m": 12.3,
        "finish_armed": True,
        "max_speed_mps": 0.49,
        "odom_source": "primary",
        "odom_topic": "/igvc_sim/ground_truth_odom",
        "odom_sample_count": 33,
        "primary_odom_sample_count": 33,
        "fallback_odom_sample_count": 0,
        "used_fallback_before_primary": False,
        "last_odom_age_s": 0.1,
        "waypoints_total": 1,
        "waypoints_reached_count": 1,
        "all_waypoints_reached": True,
        "finish_reached": (not fail),
        "speed_check_complete": True,
    }
    sc = String()
    sc.data = json.dumps(score, sort_keys=True)
    put("/igvc_sim/score", sc, 19.0)
    del w  # close/flush

    (run_dir / "final_score.txt").write_text(
        'data: "%s"\n' % json.dumps(score, sort_keys=True).replace('"', '\\"'),
        encoding="utf-8")
    (run_dir / "mission.log").write_text(
        ("mission aborted at waypoint x\n" if fail else "mission complete\n"),
        encoding="utf-8")
    (run_dir / "mission_status.txt").write_text("0\n", encoding="utf-8")
    (run_dir / "run_one_status.txt").write_text("0\n", encoding="utf-8")
    (run_dir / "run_id.txt").write_text(RUN_ID + "\n", encoding="utf-8")
    (run_dir / "course_config_sha256.txt").write_text(
        COURSE_SHA256 + "\n", encoding="utf-8")


def write_score_file(run_dir: Path, score: dict) -> None:
    (run_dir / "final_score.txt").write_text(
        'data: "%s"\n' % json.dumps(score, sort_keys=True).replace('"', '\\"'),
        encoding="utf-8")


def check(name: str, cond: bool) -> bool:
    print(("  OK  " if cond else "  FAIL") + " " + name)
    return cond


def main() -> int:
    tmp = Path(tempfile.mkdtemp(prefix="ar_selftest_"))
    ok = True
    try:
        # clean run
        clean_dir = tmp / "clean"
        clean_dir.mkdir(parents=True)
        write_bag(clean_dir, fail=False)
        m = M.compute_metrics(clean_dir, COURSE)
        print("clean metrics:", json.dumps(
            {k: m[k] for k in ("traversal_time", "breadcrumb", "gradient",
             "pathfootprint_rejects", "backup", "ang_reversals", "stuck_events",
             "time_below_speed", "finish_reached", "violations", "mission_completed",
             "min_course_clear", "pca_first_s", "score_trustworthy",
             "all_waypoints_reached", "score_source")}, default=str))
        ok &= check("score parsed + clean", m["score_loaded"] and not m["failed"]
                    and m["finish_reached"] and not m["violations"])
        ok &= check("score is trustworthy", m["score_trustworthy"] is True)
        ok &= check("score source is final_score", m["score_source"] == "final_score")
        ok &= check("mission_completed True", m["mission_completed"] is True)
        ok &= check("traversal_time ~16s",
                    m["traversal_time"] is not None and 15.0 <= m["traversal_time"] <= 17.0)
        ok &= check("breadcrumb==1", m["breadcrumb"] == 1)
        ok &= check("gradient==1", m["gradient"] == 1)
        ok &= check("pathfootprint_rejects==1", m["pathfootprint_rejects"] == 1)
        ok &= check("backup==1", m["backup"] == 1)
        ok &= check("ang_reversals>0", (m["ang_reversals"] or 0) > 0)
        ok &= check("stuck_events>=1 (10-12s gap)", (m["stuck_events"] or 0) >= 1)
        ok &= check("time_below_speed>=1.5", (m["time_below_speed"] or 0) >= 1.5)
        ok &= check("min_course_clear computed +", m["min_course_clear"] is not None
                    and m["min_course_clear"] > 0)
        ok &= check("pca_first_s computed", m["pca_first_s"] is not None)

        # fitness on a 3x clean candidate
        res = F.evaluate_candidate([m, m, m], course="compact_baseline", tier=2,
                                   commit="deadbeef", best_fitness=None)
        print(F.report_card(res, [m, m, m]))
        print(F.result_line(res))
        ok &= check("gate PASS on 3x clean", res["gate"] == "PASS")
        ok &= check("decision KEEP vs no best", res["decision"] == "KEEP")
        ok &= check("fitness is finite", res["fitness"] is not None)

        # failing run -> gate FAIL
        fail_dir = tmp / "fail"
        fail_dir.mkdir(parents=True)
        write_bag(fail_dir, fail=True)
        mf = M.compute_metrics(fail_dir, COURSE)
        ok &= check("fail run detected (violations)", bool(mf["violations"]))
        resf = F.evaluate_candidate([m, mf, m], course="compact_baseline", tier=2)
        ok &= check("gate FAIL on 1 bad run", resf["gate"] == "FAIL"
                    and resf["decision"] == "DISCARD")
        ok &= check("failed candidate has progress score",
                    resf.get("progress_fitness") is not None
                    and resf.get("distance_mean") is not None)
        mb = dict(mf)
        mb["violations"] = ["blocking_traffic_over_60s"]
        resb = F.evaluate_candidate([mb], course="compact_baseline", tier=1)
        ok &= check("blocking traffic violation -> gate FAIL",
                    F.run_clean(mb) is False
                    and resb["gate"] == "FAIL"
                    and "blocking_traffic_over_60s" in resb.get("notes", ""))

        # mission runner abort with a superficially clean monitor score must
        # still fail the reliability gate.
        abort_dir = tmp / "abort"
        abort_dir.mkdir(parents=True)
        write_bag(abort_dir, fail=False)
        (abort_dir / "mission_status.txt").write_text("1\n", encoding="utf-8")
        (abort_dir / "mission.log").write_text(
            "[ros2run]: Process exited with failure 1\n", encoding="utf-8")
        ma = M.compute_metrics(abort_dir, COURSE)
        resa = F.evaluate_candidate([ma], course="compact_baseline", tier=1)
        ok &= check("mission_status nonzero -> gate FAIL",
                    F.run_clean(ma) is False
                    and resa["gate"] == "FAIL"
                    and "mission_status=1" in resa.get("notes", ""))

        # Old/stale score schemas or fallback-odom scores must not be trusted
        # as clean AutoResearch evidence.
        stale_dir = tmp / "stale_score"
        stale_dir.mkdir(parents=True)
        write_bag(stale_dir, fail=False)
        stale_score = {
            "course_id": "compact_baseline",
            "failed": False,
            "failures": [],
            "distance_m": 12.3,
            "max_speed_mps": 0.49,
            "finish_reached": True,
            "speed_check_complete": True,
        }
        (stale_dir / "final_score.txt").write_text(
            'data: "%s"\n'
            % json.dumps(stale_score, sort_keys=True).replace('"', '\\"'),
            encoding="utf-8")
        ms = M.compute_metrics(stale_dir, COURSE)
        ok &= check("stale score schema -> INCOMPLETE",
                    ms["score_loaded"]
                    and ms["score_trustworthy"] is False
                    and F.run_clean(ms) is None)

        fallback_dir = tmp / "fallback_score"
        fallback_dir.mkdir(parents=True)
        write_bag(fallback_dir, fail=False)
        fallback_score = dict(stale_score)
        fallback_score.update({
            "score_schema_version": 4,
            "scoring_mode": "ground_truth_odom_swept_footprint_v4",
            "run_id": RUN_ID,
            "course_config_sha256": COURSE_SHA256,
            "odom_source": "fallback",
            "odom_topic": "/igvc_sim/ground_truth_odom",
            "odom_sample_count": 33,
            "primary_odom_sample_count": 0,
            "fallback_odom_sample_count": 33,
            "used_fallback_before_primary": True,
            "last_odom_age_s": 0.1,
            "waypoints_total": 1,
            "waypoints_reached_count": 1,
            "all_waypoints_reached": True,
        })
        (fallback_dir / "final_score.txt").write_text(
            'data: "%s"\n'
            % json.dumps(fallback_score, sort_keys=True).replace('"', '\\"'),
            encoding="utf-8")
        mfallback = M.compute_metrics(fallback_dir, COURSE)
        ok &= check("fallback odom score -> INCOMPLETE",
                    mfallback["score_loaded"]
                    and mfallback["score_trustworthy"] is False
                    and F.run_clean(mfallback) is None)

        stale_odom_dir = tmp / "stale_odom_score"
        stale_odom_dir.mkdir(parents=True)
        write_bag(stale_odom_dir, fail=False)
        stale_odom_score = dict(fallback_score)
        stale_odom_score.update({
            "odom_source": "primary",
            "primary_odom_sample_count": 33,
            "fallback_odom_sample_count": 0,
            "used_fallback_before_primary": False,
            "last_odom_age_s": 10.0,
        })
        (stale_odom_dir / "final_score.txt").write_text(
            'data: "%s"\n'
            % json.dumps(stale_odom_score, sort_keys=True).replace('"', '\\"'),
            encoding="utf-8")
        mstale_odom = M.compute_metrics(stale_odom_dir, COURSE)
        ok &= check("stale odom score -> INCOMPLETE",
                    mstale_odom["score_loaded"]
                    and mstale_odom["score_trustworthy"] is False
                    and F.run_clean(mstale_odom) is None)

        missed_wp_dir = tmp / "missed_waypoint_score"
        missed_wp_dir.mkdir(parents=True)
        write_bag(missed_wp_dir, fail=False)
        missed_wp_score = dict(fallback_score)
        missed_wp_score.update({
            "odom_source": "primary",
            "primary_odom_sample_count": 33,
            "fallback_odom_sample_count": 0,
            "used_fallback_before_primary": False,
            "waypoints_total": 2,
            "waypoints_reached_count": 1,
            "all_waypoints_reached": False,
        })
        (missed_wp_dir / "final_score.txt").write_text(
            'data: "%s"\n'
            % json.dumps(missed_wp_score, sort_keys=True).replace('"', '\\"'),
            encoding="utf-8")
        mmissed_wp = M.compute_metrics(missed_wp_dir, COURSE)
        ok &= check("missed ground-truth waypoint -> gate FAIL",
                    mmissed_wp["score_trustworthy"] is True
                    and F.run_clean(mmissed_wp) is False)

        bag_only_dir = tmp / "bag_only_score"
        bag_only_dir.mkdir(parents=True)
        write_bag(bag_only_dir, fail=False)
        (bag_only_dir / "final_score.txt").unlink()
        mbag_only = M.compute_metrics(bag_only_dir, COURSE)
        ok &= check("bag-only score is diagnostic, not clean evidence",
                    mbag_only["score_loaded"]
                    and mbag_only["score_source"] == "bag"
                    and mbag_only["score_trustworthy"] is False
                    and F.run_clean(mbag_only) is None)

        wrong_run_dir = tmp / "wrong_run_id"
        wrong_run_dir.mkdir(parents=True)
        write_bag(wrong_run_dir, fail=False)
        wrong_run_score = {
            "score_schema_version": 4,
            "scoring_mode": "ground_truth_odom_swept_footprint_v4",
            "run_id": "different-run",
            "course_id": "compact_baseline",
            "course_config_sha256": COURSE_SHA256,
            "failed": False,
            "failures": [],
            "distance_m": 12.3,
            "finish_armed": True,
            "max_speed_mps": 0.49,
            "odom_source": "primary",
            "odom_topic": "/igvc_sim/ground_truth_odom",
            "odom_sample_count": 33,
            "primary_odom_sample_count": 33,
            "fallback_odom_sample_count": 0,
            "used_fallback_before_primary": False,
            "last_odom_age_s": 0.1,
            "waypoints_total": 1,
            "waypoints_reached_count": 1,
            "all_waypoints_reached": True,
            "finish_reached": True,
            "speed_check_complete": True,
        }
        write_score_file(wrong_run_dir, wrong_run_score)
        mwrong_run = M.compute_metrics(wrong_run_dir, COURSE)
        ok &= check("wrong run_id score -> INCOMPLETE",
                    mwrong_run["score_loaded"]
                    and mwrong_run["score_trustworthy"] is False
                    and F.run_clean(mwrong_run) is None)

        wrong_course_dir = tmp / "wrong_course_hash"
        wrong_course_dir.mkdir(parents=True)
        write_bag(wrong_course_dir, fail=False)
        wrong_course_score = dict(wrong_run_score)
        wrong_course_score.update({
            "run_id": RUN_ID,
            "course_config_sha256": "0" * 64,
        })
        write_score_file(wrong_course_dir, wrong_course_score)
        mwrong_course = M.compute_metrics(wrong_course_dir, COURSE)
        ok &= check("wrong course hash score -> INCOMPLETE",
                    mwrong_course["score_loaded"]
                    and mwrong_course["score_trustworthy"] is False
                    and F.run_clean(mwrong_course) is None)

        rejected_dir = tmp / "run_one_rejected_clean_score"
        rejected_dir.mkdir(parents=True)
        write_bag(rejected_dir, fail=False)
        (rejected_dir / "run_one_status.txt").write_text("1\n", encoding="utf-8")
        mrejected = M.compute_metrics(rejected_dir, COURSE)
        ok &= check("nonzero run_one_status with clean-looking score -> INCOMPLETE",
                    mrejected["score_trustworthy"] is True
                    and F.run_clean(mrejected) is None)

        # incomplete run (no score) -> not clean
        inc_dir = tmp / "inc"
        (inc_dir / "bag").mkdir(parents=True)
        mi = M.compute_metrics(inc_dir, COURSE)
        ok &= check("missing bag/score -> INCOMPLETE (run_clean None)",
                    F.run_clean(mi) is None)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    print("RESULT: " + ("ALL PASS" if ok else "SOME FAILED"))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import sys

from .course import Course, load_course

SCORE_SCHEMA_VERSION = 2

try:
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool, String
except ImportError as exc:  # pragma: no cover - ROS runtime only.
    raise SystemExit(
        "igvc_course_monitor must run in a sourced ROS 2 Humble environment"
    ) from exc


def _stamp_s(node: Node) -> float:
    stamp = node.get_clock().now().to_msg()
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


def _point_segment_distance(x: float,
                            y: float,
                            start: tuple[float, float],
                            end: tuple[float, float]) -> float:
    ax, ay = start
    bx, by = end
    abx = bx - ax
    aby = by - ay
    denom = abx * abx + aby * aby
    if denom <= 1e-12:
        return math.hypot(x - ax, y - ay)
    t = max(0.0, min(1.0, ((x - ax) * abx + (y - ay) * aby) / denom))
    qx = ax + t * abx
    qy = ay + t * aby
    return math.hypot(x - qx, y - qy)


def _shortest_angle_delta(start: float, end: float) -> float:
    return (end - start + math.pi) % (2.0 * math.pi) - math.pi


class IgvcCourseMonitor(Node):
    def __init__(self) -> None:
        super().__init__("igvc_course_monitor")
        self.declare_parameter("course_config", "")
        self.declare_parameter("sample_spacing_m", 0.05)
        self.declare_parameter("odom_topic", "/igvc_sim/ground_truth_odom")
        self.declare_parameter("fallback_odom_topic", "/odom")
        self.declare_parameter("finish_reentry_margin_m", 0.25)
        course_path = str(self.get_parameter("course_config").value).strip()
        self.course: Course = load_course(course_path or None)
        self.robot = self.course.robot
        self.sample_spacing_m = max(
            0.02, float(self.get_parameter("sample_spacing_m").value))
        self.odom_topic = str(self.get_parameter("odom_topic").value).strip()
        self.fallback_odom_topic = str(
            self.get_parameter("fallback_odom_topic").value).strip()
        self.finish_reentry_margin_m = max(
            0.0, float(self.get_parameter("finish_reentry_margin_m").value))
        self.ramp_monitor_lateral_margin_m = 1.0

        self.score_pub = self.create_publisher(String, "/igvc_sim/score", 10)
        self.fail_pub = self.create_publisher(Bool, "/igvc_sim/fail", 10)
        self.odom_source = ""
        self.primary_odom_seen = False
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic,
            lambda msg: self._odom_callback(msg, "primary"), 20)
        self.fallback_odom_sub = None
        if self.fallback_odom_topic and self.fallback_odom_topic != self.odom_topic:
            self.fallback_odom_sub = self.create_subscription(
                Odometry, self.fallback_odom_topic,
                lambda msg: self._odom_callback(msg, "fallback"), 20)

        self.last_pose: tuple[float, float, float] | None = None
        self.last_time_s: float | None = None
        self.distance_m = 0.0
        self.speed_check_start_s: float | None = None
        self.speed_check_start_distance_m: float = 0.0
        self.speed_check_end_s: float | None = None
        self.stop_started_s: float | None = None
        self.blocking_anchor_pose: tuple[float, float] | None = None
        self.blocking_anchor_time_s: float | None = None
        self.autonomous = True
        self.failures: list[str] = []
        self.failure_details: list[dict[str, object]] = []
        self.first_failure: dict[str, object] | None = None
        self.max_speed_mps = 0.0
        self.finish_reached = False
        fx, fy, radius = self.course.finish
        start_finish_distance = math.hypot(
            self.course.start.x - fx, self.course.start.y - fy)
        self.finish_armed = start_finish_distance > radius
        self.create_timer(1.0, self._publish_score)
        self.get_logger().info(
            "IGVC course monitor scoring with odom_topic=%s fallback=%s "
            "finish_armed=%s"
            % (self.odom_topic, self.fallback_odom_topic, self.finish_armed))

    def _odom_callback(self, msg: Odometry, source: str = "primary") -> None:
        if source == "fallback" and self.primary_odom_seen:
            return
        if source == "primary" and not self.primary_odom_seen:
            self.primary_odom_seen = True
            if self.odom_source == "fallback":
                # Ground-truth odom is the scoring authority. If fallback odom
                # arrived first during bringup, reset sampling state so a frame
                # switch cannot create a fake speed, distance, or contact.
                self.last_pose = None
                self.last_time_s = None
        self.odom_source = source
        now_s = _stamp_s(self)
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        yaw = _yaw_from_quaternion(q.x, q.y, q.z, q.w)
        pose = (x, y, yaw)
        reported_speed = abs(float(msg.twist.twist.linear.x))
        step_distance = 0.0
        derived_speed = 0.0
        previous_pose = self.last_pose
        if self.last_pose is not None:
            step_distance = math.hypot(x - self.last_pose[0],
                                       y - self.last_pose[1])
            if self.last_time_s is not None:
                dt = now_s - self.last_time_s
                if 0.02 <= dt <= 1.0:
                    derived_speed = step_distance / dt
        motion_speed = max(reported_speed, derived_speed)
        # Enforce max speed from odom twist when available. Position-derived
        # speed is useful for stop detection, but VM sim-clock jitter can make
        # small odom steps look like impossible 6-10 m/s bursts.
        limit_speed = reported_speed if reported_speed > 0.01 else derived_speed
        self.max_speed_mps = max(self.max_speed_mps, limit_speed)

        self.distance_m += step_distance
        if not self.finish_reached:
            self._check_course_contact_swept(previous_pose, pose)
            self._update_speed_checks(now_s, motion_speed, limit_speed, pose)
            self._check_finish(x, y)
            if self.finish_reached:
                self.stop_started_s = None

        self.last_pose = pose
        self.last_time_s = now_s

    def _update_speed_checks(self,
                             now_s: float,
                             motion_speed: float,
                             limit_speed: float,
                             pose: tuple[float, float, float]) -> None:
        if self.speed_check_start_s is None:
            # The sim stack can publish odom for many seconds before the
            # mission runner sends the first waypoint. The IGVC 44 ft speed
            # check starts when the robot actually begins the run, not while
            # it is parked during bringup.
            if motion_speed < 0.05 and self.distance_m < 0.05:
                return
            self.speed_check_start_s = now_s
            self.speed_check_start_distance_m = self.distance_m
        if (self.speed_check_end_s is None
                and self.distance_m - self.speed_check_start_distance_m
                >= self.course.speed_check.end_distance_m):
            self.speed_check_end_s = now_s
            elapsed = max(1e-6, now_s - self.speed_check_start_s)
            avg = self.course.speed_check.end_distance_m / elapsed
            if avg < self.course.speed_check.minimum_average_mps:
                self._fail(
                    "first_44ft_speed_below_1mph: %.3f m/s" % avg,
                    failure_type="speed_check",
                    pose=pose)
        if limit_speed > self.course.speed_check.maximum_speed_mps:
            self._fail("max_speed_exceeded: %.3f m/s" % limit_speed,
                       failure_type="speed_check",
                       pose=pose)
        if motion_speed < self.course.speed_check.blocking_speed_mps:
            if self.stop_started_s is None:
                self.stop_started_s = now_s
            elif now_s - self.stop_started_s > self.course.speed_check.blocking_stop_s:
                self._fail(
                    "blocking_traffic_over_60s",
                    failure_type="blocking_traffic",
                    pose=pose)
        else:
            self.stop_started_s = None
        self._update_blocking_progress(now_s, pose)

    def _update_blocking_progress(
            self,
            now_s: float,
            pose: tuple[float, float, float]) -> None:
        radius = self.course.speed_check.blocking_progress_radius_m
        if self.blocking_anchor_pose is None:
            self.blocking_anchor_pose = (pose[0], pose[1])
            self.blocking_anchor_time_s = now_s
            return
        moved = math.hypot(pose[0] - self.blocking_anchor_pose[0],
                           pose[1] - self.blocking_anchor_pose[1])
        if moved > radius:
            self.blocking_anchor_pose = (pose[0], pose[1])
            self.blocking_anchor_time_s = now_s
            return
        if (self.blocking_anchor_time_s is not None
                and now_s - self.blocking_anchor_time_s
                > self.course.speed_check.blocking_stop_s):
            self._fail(
                "blocking_traffic_no_progress_over_60s",
                failure_type="blocking_traffic",
                pose=pose)

    def _check_finish(self, x: float, y: float) -> None:
        fx, fy, radius = self.course.finish
        distance_to_finish = math.hypot(x - fx, y - fy)
        if not self.finish_armed:
            if distance_to_finish > radius + self.finish_reentry_margin_m:
                self.finish_armed = True
            return
        if distance_to_finish <= radius:
            self.finish_reached = True

    def _check_course_contact_swept(
            self,
            previous_pose: tuple[float, float, float] | None,
            current_pose: tuple[float, float, float]) -> None:
        if previous_pose is None:
            self._check_course_contact(*current_pose)
            return
        distance = math.hypot(current_pose[0] - previous_pose[0],
                              current_pose[1] - previous_pose[1])
        samples = max(1, int(math.ceil(distance / self.sample_spacing_m)))
        yaw_delta = _shortest_angle_delta(previous_pose[2], current_pose[2])
        for idx in range(samples + 1):
            t = idx / float(samples)
            x = previous_pose[0] + (current_pose[0] - previous_pose[0]) * t
            y = previous_pose[1] + (current_pose[1] - previous_pose[1]) * t
            yaw = previous_pose[2] + yaw_delta * t
            self._check_course_contact(x, y, yaw)

    def _check_course_contact(self, base_x: float, base_y: float,
                              yaw: float) -> None:
        nav_x = (
            base_x + self.robot.base_link_to_nav_center_m * math.cos(yaw))
        nav_y = (
            base_y + self.robot.base_link_to_nav_center_m * math.sin(yaw))
        hx = self.robot.physical_half_length_m + self.robot.footprint_padding_m
        hy = self.robot.physical_half_width_m + self.robot.footprint_padding_m

        for tape in self.course.tapes:
            if self._segment_hits_body(
                    tape.start, tape.end, tape.width_m * 0.5,
                    nav_x, nav_y, yaw, hx, hy):
                self._fail(
                    "tape_crossing:" + tape.name,
                    failure_type="tape_crossing",
                    hazard=tape.name,
                    pose=(base_x, base_y, yaw),
                )
        for obstacle in self.course.obstacles:
            if self._circle_hits_body(
                    obstacle.center, obstacle.radius_m, nav_x, nav_y,
                    yaw, hx, hy):
                self._fail(
                    "obstacle_contact:" + obstacle.name,
                    failure_type="obstacle_contact",
                    hazard=obstacle.name,
                    pose=(base_x, base_y, yaw),
                )
        for ramp in self.course.ramps:
            if nav_x + hx >= ramp.start_x_m and nav_x - hx <= ramp.end_x_m:
                lateral = abs(nav_y - ramp.center_y_m)
                edge_limit = ramp.width_m * 0.5 + hy
                distance_to_ramp_center = _point_segment_distance(
                    nav_x, nav_y,
                    (ramp.start_x_m, ramp.center_y_m),
                    (ramp.end_x_m, ramp.center_y_m),
                )
                # Long loop courses can revisit the same x range far away
                # from an x-aligned ramp. Only suppress the ramp-specific
                # check when the robot is clearly in a different lane; tape and
                # obstacle contact checks above still score normal course exits.
                if distance_to_ramp_center > (
                        edge_limit + self.ramp_monitor_lateral_margin_m):
                    continue
                if lateral > edge_limit:
                    self._fail(
                        "ramp_edge_departure:" + ramp.name,
                        failure_type="ramp_edge_departure",
                        hazard=ramp.name,
                        pose=(base_x, base_y, yaw),
                    )

    def _segment_hits_body(self,
                           start: tuple[float, float],
                           end: tuple[float, float],
                           half_width: float,
                           nav_x: float,
                           nav_y: float,
                           yaw: float,
                           hx: float,
                           hy: float) -> bool:
        length = max(0.0, math.hypot(end[0] - start[0], end[1] - start[1]))
        samples = max(1, int(math.ceil(length / self.sample_spacing_m)))
        for idx in range(samples + 1):
            t = idx / float(samples)
            x = start[0] + (end[0] - start[0]) * t
            y = start[1] + (end[1] - start[1]) * t
            local_x, local_y = self._world_to_nav(x, y, nav_x, nav_y, yaw)
            if abs(local_x) <= hx + half_width and abs(local_y) <= hy + half_width:
                return True
        return False

    def _circle_hits_body(self,
                          center: tuple[float, float],
                          radius_m: float,
                          nav_x: float,
                          nav_y: float,
                          yaw: float,
                          hx: float,
                          hy: float) -> bool:
        local_x, local_y = self._world_to_nav(
            center[0], center[1], nav_x, nav_y, yaw)
        dx = max(abs(local_x) - hx, 0.0)
        dy = max(abs(local_y) - hy, 0.0)
        return math.hypot(dx, dy) <= radius_m

    @staticmethod
    def _world_to_nav(x: float, y: float, nav_x: float, nav_y: float,
                      yaw: float) -> tuple[float, float]:
        dx = x - nav_x
        dy = y - nav_y
        c = math.cos(yaw)
        s = math.sin(yaw)
        return c * dx + s * dy, -s * dx + c * dy

    def _fail(self,
              reason: str,
              *,
              failure_type: str = "unknown",
              hazard: str = "",
              pose: tuple[float, float, float] | None = None) -> None:
        if reason not in self.failures:
            self.failures.append(reason)
            pose = pose if pose is not None else self.last_pose
            detail: dict[str, object] = {
                "reason": reason,
                "type": failure_type,
                "hazard": hazard,
                "sim_time_s": round(_stamp_s(self), 3),
            }
            if pose is not None:
                detail.update({
                    "base_x_m": round(pose[0], 3),
                    "base_y_m": round(pose[1], 3),
                    "yaw_rad": round(pose[2], 3),
                })
            self.failure_details.append(detail)
            if self.first_failure is None:
                self.first_failure = detail
            self.get_logger().error(
                "IGVC sim failure: %s detail=%s"
                % (reason, json.dumps(detail, sort_keys=True)))

    def _publish_score(self) -> None:
        score = {
            "score_schema_version": SCORE_SCHEMA_VERSION,
            "course_id": self.course.course_id,
            "failed": bool(self.failures),
            "failures": self.failures,
            "first_failure": self.first_failure,
            "failure_details": self.failure_details,
            "distance_m": round(self.distance_m, 3),
            "max_speed_mps": round(self.max_speed_mps, 3),
            "finish_reached": self.finish_reached,
            "finish_armed": self.finish_armed,
            "odom_topic": self.odom_topic,
            "odom_source": self.odom_source,
            "speed_check_complete": self.speed_check_end_s is not None,
        }
        self.score_pub.publish(String(data=json.dumps(score, sort_keys=True)))
        self.fail_pub.publish(Bool(data=bool(self.failures)))


def main(argv: list[str] | None = None) -> int:
    _ = argv
    rclpy.init(args=sys.argv)
    node = IgvcCourseMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

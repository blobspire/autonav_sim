#!/usr/bin/env python3
"""minimal_navigator — the bundled minimal example robot's autonomy.

A deliberately compact, readable controller that drives the minibot through the
IGVC course to the finish, CPU-only and with zero external deps:

  * Path-following (pure pursuit) — track the *line segments* between the
    course's mission waypoints, not just their endpoints. The waypoint-to-
    waypoint lines are laid out to stay in the lane, so following the line
    (instead of cutting the corner toward the next point) keeps the robot clear
    of the barrels the corners would clip.
  * Obstacle avoidance — from the simulated lidar (/scan_fullframe): when an
    obstacle falls inside a forward cone, steer smoothly away from it (stronger
    the closer it is) and slow down, arcing around anything the path does graze.

It closes the perception -> control loop using the sim's own lidar — it is the
worked "how a robot plugs in" example, NOT a scripted path. It is intentionally
simple (no Nav2, no costmaps); the AutoNav example is the full stack.

Localization: the minimal robot navigates on the simulator's ground-truth pose
(/igvc_sim/ground_truth_odom) for simplicity — a real robot brings its own
odometry/SLAM; this keeps the example focused on navigation + avoidance, not
state estimation. Contract: subscribes odom + LaserScan, publishes
geometry_msgs/Twist on /cmd_vel. Every behaviour is a tunable ROS parameter.
"""
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from igvc_competition_sim.course import load_course


def _yaw_from_quaternion(q) -> float:
    """Yaw (rotation about +Z) from a geometry_msgs quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _normalize_angle(angle: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class MinimalNavigator(Node):
    def __init__(self) -> None:
        super().__init__("minimal_navigator")

        # --- parameters (all tunable; defaults tuned for the compact course) ---
        self.declare_parameter("course_config", "")   # "" => default course
        self.declare_parameter("odom_topic", "/igvc_sim/ground_truth_odom")
        self.declare_parameter("scan_topic", "/scan_fullframe")
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("max_linear_mps", 0.8)
        self.declare_parameter("max_angular_rps", 1.6)
        self.declare_parameter("heading_gain", 2.2)
        self.declare_parameter("lookahead_m", 1.0)           # pure-pursuit lookahead
        self.declare_parameter("obstacle_influence_m", 2.5)  # start avoiding here
        self.declare_parameter("obstacle_repel_gain", 1.0)   # gentle: path-following
        #                                                      does the routing; this
        #                                                      only nudges off grazes
        self.declare_parameter("forward_cone_deg", 45.0)     # only avoid obstacles ahead
        self.declare_parameter("goal_tolerance_scale", 1.0)  # x waypoint radius

        # Load the course; the path is start -> each mission waypoint in order
        # (the last coincides with the finish, so reaching it finishes).
        course_config = str(self.get_parameter("course_config").value).strip()
        course = load_course(course_config or None)
        self.targets: list[tuple[float, float, float]] = [
            (wp.x_m, wp.y_m, wp.radius_m) for wp in course.mission_waypoints
        ]
        if not self.targets:
            self.targets = [tuple(course.finish)]  # type: ignore[list-item]
        self.finish = course.finish
        self.target_idx = 0
        self.prev_point = (course.start.x, course.start.y)  # segment start

        self.pose: tuple[float, float, float] | None = None  # (x, y, yaw)
        self.scan: LaserScan | None = None
        self.done = False
        # stuck-recovery state
        self._check_pos: tuple[float, float] | None = None
        self._check_t = 0.0
        self._recovery_until = 0.0
        self._recovery_turn = 0.0

        self.cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter("cmd_topic").value), 10)
        self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value),
            self._on_odom, 20)
        # The harness publishes the scan with best-effort sensor QoS — match it
        # (a reliable subscription silently receives nothing).
        self.create_subscription(
            LaserScan, str(self.get_parameter("scan_topic").value),
            self._on_scan, qos_profile_sensor_data)
        self.create_timer(0.05, self._control_step)  # 20 Hz control loop

        self.get_logger().info(
            f"minimal_navigator: {len(self.targets)} waypoints -> "
            f"finish ({self.finish[0]:.1f}, {self.finish[1]:.1f})")

    # --- sensor callbacks ---
    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y,
                     _yaw_from_quaternion(p.orientation))

    def _on_scan(self, msg: LaserScan) -> None:
        self.scan = msg

    def _nearest_obstacle_ahead(self, half_cone: float) -> tuple[float, float]:
        """(range, bearing) of the closest lidar return within +/- half_cone of
        straight ahead (0 = forward). (inf, 0.0) when the cone is clear. A narrow
        cone means we only react to things actually in our path, not beside us."""
        scan = self.scan
        if scan is None:
            return math.inf, 0.0
        best_range, best_bearing = math.inf, 0.0
        angle = scan.angle_min
        for rng in scan.ranges:
            if (-half_cone <= angle <= half_cone and math.isfinite(rng)
                    and rng >= scan.range_min and rng < best_range):
                best_range, best_bearing = rng, angle
            angle += scan.angle_increment
        return best_range, best_bearing

    def _lookahead_point(self, x: float, y: float, tx: float, ty: float,
                         lookahead: float) -> tuple[float, float]:
        """A point `lookahead` metres ahead of the robot's projection onto the
        current path segment (prev_point -> target). Following this tracks the
        line rather than cutting the corner toward the target."""
        px, py = self.prev_point
        seg_x, seg_y = tx - px, ty - py
        seg_len = math.hypot(seg_x, seg_y)
        if seg_len < 1e-6:
            return tx, ty
        ux, uy = seg_x / seg_len, seg_y / seg_len          # unit along segment
        along = (x - px) * ux + (y - py) * uy              # projection onto segment
        look = min(seg_len, max(0.0, along) + lookahead)
        return px + ux * look, py + uy * look

    # --- control loop ---
    def _control_step(self) -> None:
        if self.done or self.pose is None:
            return
        x, y, yaw = self.pose
        tx, ty, radius = self.targets[self.target_idx]
        max_lin = float(self.get_parameter("max_linear_mps").value)
        max_ang = float(self.get_parameter("max_angular_rps").value)

        # Stuck recovery: if we're commanding motion but not moving (wedged on an
        # obstacle too close for the lidar to see), back out and pivot toward the
        # target, then retry. The robot can pivot now, so this frees it.
        now = self.get_clock().now().nanoseconds * 1e-9
        if now < self._recovery_until:
            cmd = Twist()
            cmd.linear.x = -0.3 * max_lin
            cmd.angular.z = self._recovery_turn
            self.cmd_pub.publish(cmd)
            return
        if self._check_pos is None or now - self._check_t > 3.0:
            if (self._check_pos is not None and math.hypot(
                    x - self._check_pos[0], y - self._check_pos[1]) < 0.25):
                self._recovery_until = now + 2.5
                to_target = _normalize_angle(math.atan2(ty - y, tx - x) - yaw)
                self._recovery_turn = max_ang if to_target >= 0.0 else -max_ang
                self.get_logger().warn(
                    f"stuck at ({x:.1f},{y:.1f}); backing out")
            self._check_pos = (x, y)
            self._check_t = now

        # Waypoint reached? advance (finish when past the last).
        if math.hypot(tx - x, ty - y) <= radius * float(
                self.get_parameter("goal_tolerance_scale").value):
            self.prev_point = (tx, ty)
            self.target_idx += 1
            if self.target_idx >= len(self.targets):
                self._finish()
            else:
                self.get_logger().info(
                    f"reached waypoint {self.target_idx}/{len(self.targets)}")
            return

        max_lin = float(self.get_parameter("max_linear_mps").value)
        max_ang = float(self.get_parameter("max_angular_rps").value)
        gain = float(self.get_parameter("heading_gain").value)
        lookahead = float(self.get_parameter("lookahead_m").value)
        influence = float(self.get_parameter("obstacle_influence_m").value)
        repel = float(self.get_parameter("obstacle_repel_gain").value)
        cone = math.radians(float(self.get_parameter("forward_cone_deg").value))

        # Path-following: steer toward a lookahead point on the current segment.
        lax, lay = self._lookahead_point(x, y, tx, ty, lookahead)
        heading_error = _normalize_angle(math.atan2(lay - y, lax - x) - yaw)
        steer = gain * heading_error
        speed = max_lin * max(0.25, math.cos(heading_error))

        # Obstacle avoidance: a smooth steer-away from the nearest obstacle in the
        # forward cone, stronger the closer it is, plus a proportional slow-down.
        obstacle_range, obstacle_bearing = self._nearest_obstacle_ahead(cone)
        if obstacle_range < influence:
            urgency = (influence - obstacle_range) / influence  # 0..1
            if abs(obstacle_bearing) < 0.10:        # ~dead ahead: break the tie
                away = -1.0 if heading_error <= 0.0 else 1.0
            else:                                   # steer to the obstacle's far side
                away = -1.0 if obstacle_bearing > 0.0 else 1.0
            steer += repel * urgency * away
            # This robot turns by arcing and cannot pivot at a crawl, so hold a
            # steady moderate speed near obstacles (never stall) and let the
            # steer-away arc it clear.
            speed = 0.7 * max_lin

        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = max(-max_ang, min(max_ang, steer))
        self.cmd_pub.publish(cmd)

    def _finish(self) -> None:
        self.done = True
        self.cmd_pub.publish(Twist())  # full stop
        self.get_logger().info(
            "course complete — minibot reached the finish. Stopping.")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MinimalNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.cmd_pub.publish(Twist())  # leave the robot stopped
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

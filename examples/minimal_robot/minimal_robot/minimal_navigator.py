#!/usr/bin/env python3
"""minimal_navigator — the bundled minimal example robot's autonomy.

A deliberately compact, readable reactive controller that drives the minibot
through the IGVC course to the finish, CPU-only and with zero external deps:

  * Global guidance — pure-pursuit toward the course's mission waypoints (the
    same waypoints the course_monitor scores), read from the course config.
  * Local reaction — obstacle avoidance from the simulated lidar
    (/scan_fullframe): slow near obstacles, and steer toward the clearer side
    when the path ahead is blocked.

It closes the perception -> control loop using the sim's own sensors — it is the
worked "how a robot plugs in" example, NOT a dead-reckoned scripted path. It is
intentionally simple (no Nav2, no costmaps); the AutoNav example is the full
stack. Contract: subscribes odom + LaserScan, publishes geometry_msgs/Twist on
/cmd_vel. Every behaviour is a tunable ROS parameter.
"""
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
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
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("scan_topic", "/scan_fullframe")
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("max_linear_mps", 0.9)
        self.declare_parameter("max_angular_rps", 1.2)
        self.declare_parameter("heading_gain", 1.8)
        self.declare_parameter("obstacle_slow_m", 2.5)   # start slowing here
        self.declare_parameter("obstacle_stop_m", 1.0)   # turn away inside here
        self.declare_parameter("forward_arc_deg", 35.0)  # "ahead" half-angle
        self.declare_parameter("goal_tolerance_scale", 1.0)  # x waypoint radius

        # Load the course and build the ordered target list (mission waypoints;
        # the last one coincides with the finish, so reaching it finishes).
        course_config = str(self.get_parameter("course_config").value).strip()
        course = load_course(course_config or None)
        self.targets: list[tuple[float, float, float]] = [
            (wp.x_m, wp.y_m, wp.radius_m) for wp in course.mission_waypoints
        ]
        if not self.targets:
            self.targets = [tuple(course.finish)]  # type: ignore[list-item]
        self.finish = course.finish
        self.target_idx = 0

        self.pose: tuple[float, float, float] | None = None  # (x, y, yaw)
        self.scan: LaserScan | None = None
        self.done = False

        self.cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter("cmd_topic").value), 10)
        self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value),
            self._on_odom, 20)
        self.create_subscription(
            LaserScan, str(self.get_parameter("scan_topic").value),
            self._on_scan, 5)
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

    def _min_range_in_arc(self, lo_deg: float, hi_deg: float) -> float:
        """Smallest valid lidar range whose bearing is within [lo, hi] degrees
        (0 deg = straight ahead). inf if nothing is seen in that arc."""
        scan = self.scan
        if scan is None:
            return math.inf
        lo = math.radians(lo_deg)
        hi = math.radians(hi_deg)
        best = math.inf
        angle = scan.angle_min
        for rng in scan.ranges:
            if lo <= angle <= hi and math.isfinite(rng) and rng >= scan.range_min:
                if rng < best:
                    best = rng
            angle += scan.angle_increment
        return best

    # --- control loop ---
    def _control_step(self) -> None:
        if self.done or self.pose is None:
            return
        x, y, yaw = self.pose
        tx, ty, radius = self.targets[self.target_idx]

        # Waypoint reached? advance (finish when past the last).
        if math.hypot(tx - x, ty - y) <= radius * float(
                self.get_parameter("goal_tolerance_scale").value):
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
        slow_m = float(self.get_parameter("obstacle_slow_m").value)
        stop_m = float(self.get_parameter("obstacle_stop_m").value)
        arc = float(self.get_parameter("forward_arc_deg").value)

        # Global guidance: turn toward the target; ease off the throttle the more
        # we have to turn (so we pivot toward the goal instead of arcing wide).
        heading_error = _normalize_angle(math.atan2(ty - y, tx - x) - yaw)
        angular = max(-max_ang, min(max_ang, gain * heading_error))
        linear = max_lin * max(0.0, math.cos(heading_error))

        # Local reaction: obstacle avoidance from the lidar.
        ahead = self._min_range_in_arc(-arc, arc)
        if ahead < stop_m:
            # Blocked: crawl and turn toward whichever side has more room.
            left = self._min_range_in_arc(10.0, 80.0)
            right = self._min_range_in_arc(-80.0, -10.0)
            angular = max_ang if left >= right else -max_ang
            linear = 0.12 * max_lin
        elif ahead < slow_m:
            # Approaching: scale speed down with proximity.
            frac = (ahead - stop_m) / (slow_m - stop_m)
            linear = min(linear, max_lin * (0.2 + 0.8 * frac))

        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
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

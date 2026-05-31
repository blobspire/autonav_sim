#!/usr/bin/env python3
from __future__ import annotations

from copy import deepcopy
import sys

try:
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import TransformBroadcaster
except ImportError as exc:  # pragma: no cover - ROS runtime only.
    raise SystemExit(
        "igvc_odom_bridge must run in a sourced ROS 2 Humble environment"
    ) from exc

try:
    from rclpy.exceptions import RCLError
except ImportError:
    RCLError = Exception


class IgvcOdomBridge(Node):
    def __init__(self) -> None:
        super().__init__("igvc_odom_bridge")
        self.declare_parameter("input_odom_topic", "/model/shogi/odometry")
        self.declare_parameter("output_odom_topic", "/odom")
        self.declare_parameter("output_local_odom_topic", "/local_ekf/odom")
        self.declare_parameter(
            "ground_truth_odom_topic", "/igvc_sim/ground_truth_odom")
        self.declare_parameter("publish_ground_truth_odom", True)
        self.declare_parameter("publish_map_odom_tf", True)
        self.declare_parameter("max_relay_rate_hz", 30.0)

        self.publish_map_odom_tf = bool(
            self.get_parameter("publish_map_odom_tf").value)
        self.publish_ground_truth_odom = bool(
            self.get_parameter("publish_ground_truth_odom").value)
        self.max_relay_rate_hz = max(
            0.0, float(self.get_parameter("max_relay_rate_hz").value))
        self._min_publish_period_s = (
            0.0 if self.max_relay_rate_hz <= 0.0
            else 1.0 / self.max_relay_rate_hz
        )
        self._last_publish_stamp_s: float | None = None

        self.odom_pub = self.create_publisher(
            Odometry,
            str(self.get_parameter("output_odom_topic").value),
            50,
        )
        self.local_odom_pub = self.create_publisher(
            Odometry,
            str(self.get_parameter("output_local_odom_topic").value),
            50,
        )
        self.ground_truth_pub = (
            self.create_publisher(
                Odometry,
                str(self.get_parameter("ground_truth_odom_topic").value),
                50,
            )
            if self.publish_ground_truth_odom else None
        )
        self.tf_pub = TransformBroadcaster(self)
        self.create_subscription(
            Odometry,
            str(self.get_parameter("input_odom_topic").value),
            self._odom_callback,
            50,
        )
        self.get_logger().info(
            "Relaying Gazebo odom to /odom, /local_ekf/odom, and /tf "
            f"(max_relay_rate_hz={self.max_relay_rate_hz:.1f})")

    def _odom_callback(self, msg: Odometry) -> None:
        stamp = msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = self.get_clock().now().to_msg()
        stamp_s = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        if (
            self._last_publish_stamp_s is not None
            and stamp_s >= self._last_publish_stamp_s
            and self._min_publish_period_s > 0.0
            and stamp_s - self._last_publish_stamp_s
            < self._min_publish_period_s * 0.95
        ):
            return
        self._last_publish_stamp_s = stamp_s

        odom_msg = deepcopy(msg)
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        self.odom_pub.publish(odom_msg)
        self.local_odom_pub.publish(odom_msg)
        if self.ground_truth_pub is not None:
            self.ground_truth_pub.publish(odom_msg)

        transforms: list[TransformStamped] = []
        if self.publish_map_odom_tf:
            map_odom = TransformStamped()
            map_odom.header.stamp = stamp
            map_odom.header.frame_id = "map"
            map_odom.child_frame_id = "odom"
            map_odom.transform.rotation.w = 1.0
            transforms.append(map_odom)

        odom_base = TransformStamped()
        odom_base.header.stamp = stamp
        odom_base.header.frame_id = "odom"
        odom_base.child_frame_id = "base_link"
        odom_base.transform.translation.x = msg.pose.pose.position.x
        odom_base.transform.translation.y = msg.pose.pose.position.y
        odom_base.transform.translation.z = msg.pose.pose.position.z
        odom_base.transform.rotation = msg.pose.pose.orientation
        transforms.append(odom_base)
        self.tf_pub.sendTransform(transforms)


def main(argv: list[str] | None = None) -> int:
    rclpy.init(args=sys.argv if argv is None else argv)
    node = IgvcOdomBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

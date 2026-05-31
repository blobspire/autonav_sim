#!/usr/bin/env python3
from __future__ import annotations

from array import array
from copy import deepcopy
import math
import sys

try:
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import CameraInfo, Image
    from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
except ImportError as exc:  # pragma: no cover - ROS runtime only.
    raise SystemExit(
        "igvc_camera_bridge must run in a sourced ROS 2 Humble environment"
    ) from exc

# RCLError was added to rclpy after Humble; fall back when absent (matches the
# pattern already in dynamics_replay.py). auto_camera env-compat fix.
try:
    from rclpy.exceptions import RCLError
except ImportError:
    RCLError = Exception

try:
    import numpy as np
except ImportError:  # pragma: no cover - deployment environment issue.
    np = None


def _q_from_rpy(roll: float, pitch: float, yaw: float
                ) -> tuple[float, float, float, float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _stamp_to_float(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _float_to_stamp(value: float):
    sec = int(math.floor(max(0.0, value)))
    nanosec = int(round((max(0.0, value) - sec) * 1e9))
    if nanosec >= 1000000000:
        sec += 1
        nanosec -= 1000000000
    from builtin_interfaces.msg import Time
    return Time(sec=sec, nanosec=nanosec)


class IgvcCameraBridge(Node):
    def __init__(self) -> None:
        super().__init__("igvc_camera_bridge")

        self.declare_parameter("input_image_topic", "/igvc_sim/zed/image")
        self.declare_parameter("input_depth_topic", "/igvc_sim/zed/depth_image")
        self.declare_parameter(
            "input_camera_info_topic", "/igvc_sim/zed/camera_info")
        self.declare_parameter(
            "output_image_topic", "/zed/zed_node/rgb/color/rect/image")
        self.declare_parameter(
            "output_depth_topic", "/zed/zed_node/depth/depth_registered")
        self.declare_parameter(
            "output_camera_info_topic",
            "/zed/zed_node/rgb/color/rect/camera_info")
        self.declare_parameter(
            "output_depth_info_topic", "/zed/zed_node/depth/depth_info")
        self.declare_parameter("camera_link_frame_id", "zed_camera_link")
        self.declare_parameter("camera_center_frame_id", "zed_camera_center")
        self.declare_parameter("camera_frame_id", "zed_left_camera_frame")
        self.declare_parameter(
            "optical_frame_id", "zed_left_camera_frame_optical")
        self.declare_parameter("output_color_encoding", "bgra8")
        self.declare_parameter("fallback_width", 960)
        self.declare_parameter("fallback_height", 540)
        self.declare_parameter("fallback_horizontal_fov_rad", 1.453833)
        self.declare_parameter("fallback_fx", 539.702)
        self.declare_parameter("fallback_fy", 539.702)
        self.declare_parameter("fallback_cx", 472.965)
        self.declare_parameter("fallback_cy", 255.161)
        self.declare_parameter("override_inconsistent_camera_info", True)
        self.declare_parameter("sync_rgb_depth", True)
        self.declare_parameter("max_rgb_depth_pair_delta_ms", 120)
        self.declare_parameter("max_pair_buffer_age_ms", 1200)
        self.declare_parameter("wait_for_odom_before_publish", True)
        self.declare_parameter("odom_topic", "/local_ekf/odom")
        self.declare_parameter("odom_stamp_slop_ms", 25)

        self.optical_frame_id = str(
            self.get_parameter("optical_frame_id").value)
        self.camera_frame_id = str(self.get_parameter("camera_frame_id").value)
        self.camera_center_frame_id = str(
            self.get_parameter("camera_center_frame_id").value)
        self.camera_link_frame_id = str(
            self.get_parameter("camera_link_frame_id").value)
        self.output_color_encoding = str(
            self.get_parameter("output_color_encoding").value).lower()
        self.fallback_width = int(self.get_parameter("fallback_width").value)
        self.fallback_height = int(self.get_parameter("fallback_height").value)
        self.fallback_horizontal_fov_rad = float(
            self.get_parameter("fallback_horizontal_fov_rad").value)
        self.fallback_fx = float(self.get_parameter("fallback_fx").value)
        self.fallback_fy = float(self.get_parameter("fallback_fy").value)
        self.fallback_cx = float(self.get_parameter("fallback_cx").value)
        self.fallback_cy = float(self.get_parameter("fallback_cy").value)
        self.override_inconsistent_camera_info = bool(
            self.get_parameter("override_inconsistent_camera_info").value)
        self.sync_rgb_depth = bool(
            self.get_parameter("sync_rgb_depth").value)
        self.max_pair_delta_s = max(
            0.0,
            float(self.get_parameter("max_rgb_depth_pair_delta_ms").value)
            * 1e-3,
        )
        self.max_pair_buffer_age_s = max(
            0.0,
            float(self.get_parameter("max_pair_buffer_age_ms").value) * 1e-3,
        )
        self.wait_for_odom_before_publish = bool(
            self.get_parameter("wait_for_odom_before_publish").value)
        self.odom_stamp_slop_s = max(
            0.0,
            float(self.get_parameter("odom_stamp_slop_ms").value) * 1e-3,
        )
        self.latest_odom_stamp_s: float | None = None
        self.last_published_stamp_s: float | None = None
        self.latest_camera_info: CameraInfo | None = None
        self.image_buffer: list[Image] = []
        self.depth_buffer: list[Image] = []
        self._warned_color_conversion = False

        # Camera topics are high-bandwidth sensor streams. Publishing them
        # reliable can backpressure the routed VM<->Jetson DDS link when the
        # subscriber falls behind, which stalls unrelated topics like /clock
        # and /tf. Match normal camera-driver behavior here.
        image_qos = QoSProfile(depth=5)
        image_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        info_qos = QoSProfile(depth=1)
        info_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.image_pub = self.create_publisher(
            Image,
            str(self.get_parameter("output_image_topic").value),
            image_qos,
        )
        self.depth_pub = self.create_publisher(
            Image,
            str(self.get_parameter("output_depth_topic").value),
            image_qos,
        )
        self.camera_info_pub = self.create_publisher(
            CameraInfo,
            str(self.get_parameter("output_camera_info_topic").value),
            info_qos,
        )
        self.depth_info_pub = self.create_publisher(
            CameraInfo,
            str(self.get_parameter("output_depth_info_topic").value),
            info_qos,
        )

        self.image_sub = self.create_subscription(
            Image,
            str(self.get_parameter("input_image_topic").value),
            self._image_callback,
            qos_profile_sensor_data,
        )
        self.depth_sub = self.create_subscription(
            Image,
            str(self.get_parameter("input_depth_topic").value),
            self._depth_callback,
            qos_profile_sensor_data,
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            str(self.get_parameter("input_camera_info_topic").value),
            self._camera_info_callback,
            qos_profile_sensor_data,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._odom_callback,
            50,
        )
        self.create_timer(0.02, self._try_publish_synced_pairs)

        self.tf_static = StaticTransformBroadcaster(self)
        self._publish_static_camera_transforms()

        self.get_logger().info(
            "Relaying Gazebo RGB-D camera into ZED topics with frame %s "
            "(sync_rgb_depth=%s, wait_for_odom=%s)"
            % (
                self.optical_frame_id,
                self.sync_rgb_depth,
                self.wait_for_odom_before_publish,
            )
        )

    def _stamp_if_zero(self, msg) -> None:
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            msg.header.stamp = self.get_clock().now().to_msg()

    def _image_callback(self, msg: Image) -> None:
        self._stamp_if_zero(msg)
        msg.header.frame_id = self.optical_frame_id
        if not self.sync_rgb_depth:
            self._convert_color_image(msg)
            self.image_pub.publish(msg)
            return
        self.image_buffer.append(msg)
        self._trim_buffers()
        self._try_publish_synced_pairs()

    def _depth_callback(self, msg: Image) -> None:
        self._stamp_if_zero(msg)
        msg.header.frame_id = self.optical_frame_id
        if not self.sync_rgb_depth:
            self.depth_pub.publish(msg)
            return
        self.depth_buffer.append(msg)
        self._trim_buffers()
        self._try_publish_synced_pairs()

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        self._stamp_if_zero(msg)
        msg.header.frame_id = self.optical_frame_id
        self._fill_camera_info_if_empty(msg)
        if not self.sync_rgb_depth:
            self.camera_info_pub.publish(msg)
            self.depth_info_pub.publish(msg)
            return
        self.latest_camera_info = msg
        self._try_publish_synced_pairs()

    def _odom_callback(self, msg: Odometry) -> None:
        self.latest_odom_stamp_s = _stamp_to_float(msg.header.stamp)
        self._try_publish_synced_pairs()

    def _try_publish_synced_pairs(self) -> None:
        if not self.sync_rgb_depth:
            return
        if self.latest_camera_info is None:
            return
        if self.wait_for_odom_before_publish and self.latest_odom_stamp_s is None:
            return

        while self.image_buffer and self.depth_buffer:
            image = self.image_buffer[0]
            image_stamp_s = _stamp_to_float(image.header.stamp)
            nearest_depth_idx = min(
                range(len(self.depth_buffer)),
                key=lambda idx: abs(
                    _stamp_to_float(self.depth_buffer[idx].header.stamp)
                    - image_stamp_s
                ),
            )
            depth = self.depth_buffer[nearest_depth_idx]
            depth_stamp_s = _stamp_to_float(depth.header.stamp)
            delta_s = abs(image_stamp_s - depth_stamp_s)

            if delta_s > self.max_pair_delta_s:
                if image_stamp_s < depth_stamp_s:
                    self.image_buffer.pop(0)
                else:
                    self.depth_buffer.pop(nearest_depth_idx)
                continue

            # Use the older capture stamp for the coherent RGB-D pair. Gazebo
            # can stamp RGB/depth submessages a frame apart; choosing the
            # earlier stamp avoids asking Nav2/TF for a transform that is
            # newer than the paired sensor data that actually arrived.
            pair_stamp_s = min(image_stamp_s, depth_stamp_s)
            if (self.last_published_stamp_s is not None
                    and pair_stamp_s <= self.last_published_stamp_s):
                self.image_buffer.pop(0)
                self.depth_buffer.pop(nearest_depth_idx)
                continue

            if (self.wait_for_odom_before_publish
                    and self.latest_odom_stamp_s is not None
                    and self.latest_odom_stamp_s + self.odom_stamp_slop_s
                    < pair_stamp_s):
                return

            pair_stamp = _float_to_stamp(pair_stamp_s)
            out_image = deepcopy(image)
            out_depth = deepcopy(depth)
            out_info = deepcopy(self.latest_camera_info)
            for out_msg in (out_image, out_depth, out_info):
                out_msg.header.stamp = pair_stamp
                out_msg.header.frame_id = self.optical_frame_id

            self._convert_color_image(out_image)
            self._fill_camera_info_if_empty(out_info)
            self.camera_info_pub.publish(out_info)
            self.depth_info_pub.publish(out_info)
            self.image_pub.publish(out_image)
            self.depth_pub.publish(out_depth)
            self.last_published_stamp_s = pair_stamp_s
            self.image_buffer.pop(0)
            self.depth_buffer.pop(nearest_depth_idx)
            self._trim_buffers()

    def _trim_buffers(self) -> None:
        if self.max_pair_buffer_age_s <= 0.0:
            return
        latest_stamp_s = None
        for buffer in (self.image_buffer, self.depth_buffer):
            if buffer:
                stamp_s = _stamp_to_float(buffer[-1].header.stamp)
                latest_stamp_s = (
                    stamp_s if latest_stamp_s is None
                    else max(latest_stamp_s, stamp_s)
                )
        if latest_stamp_s is None:
            return
        cutoff_s = latest_stamp_s - self.max_pair_buffer_age_s
        self.image_buffer = [
            msg for msg in self.image_buffer
            if _stamp_to_float(msg.header.stamp) >= cutoff_s
        ]
        self.depth_buffer = [
            msg for msg in self.depth_buffer
            if _stamp_to_float(msg.header.stamp) >= cutoff_s
        ]

    def _fill_camera_info_if_empty(self, msg: CameraInfo) -> None:
        if not self._camera_info_needs_override(msg):
            return
        width = int(msg.width) if msg.width else self.fallback_width
        height = int(msg.height) if msg.height else self.fallback_height
        msg.width = width
        msg.height = height
        fx = self._fallback_fx(width)
        fy = self._fallback_fy(width)
        cx = self._fallback_cx(width)
        cy = self._fallback_cy(height)
        msg.distortion_model = "plumb_bob"
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

    def _camera_info_needs_override(self, msg: CameraInfo) -> bool:
        if msg.k[0] <= 0.0 or msg.k[4] <= 0.0:
            return True
        if not self.override_inconsistent_camera_info:
            return False

        width = int(msg.width) if msg.width else self.fallback_width
        height = int(msg.height) if msg.height else self.fallback_height
        expected_fx = self._fallback_fx(width)
        expected_fy = self._fallback_fy(width)
        expected_cx = self._fallback_cx(width)
        expected_cy = self._fallback_cy(height)
        # ros_gz_bridge/Fortress can report 960x540 image dimensions while
        # leaving 320x240 intrinsics in K/P. Projection then maps real ground
        # tape pixels to impossible base-frame heights, so treat a principal
        # point or focal length far from the bag-derived ZED calibration as
        # invalid.
        fx_bad = (
            abs(float(msg.k[0]) - expected_fx)
            > max(5.0, 0.05 * expected_fx)
        )
        fy_bad = (
            abs(float(msg.k[4]) - expected_fy)
            > max(5.0, 0.05 * expected_fy)
        )
        return (
            fx_bad
            or fy_bad
            or abs(float(msg.k[2]) - expected_cx) > max(4.0, 0.05 * width)
            or abs(float(msg.k[5]) - expected_cy) > max(4.0, 0.05 * height)
        )

    def _fallback_fx(self, width: int) -> float:
        if self.fallback_fx > 0.0:
            return self.fallback_fx
        return (0.5 * width) / math.tan(0.5 * self.fallback_horizontal_fov_rad)

    def _fallback_fy(self, width: int) -> float:
        if self.fallback_fy > 0.0:
            return self.fallback_fy
        return self._fallback_fx(width)

    def _fallback_cx(self, width: int) -> float:
        if self.fallback_cx >= 0.0:
            return self.fallback_cx
        return 0.5 * (width - 1)

    def _fallback_cy(self, height: int) -> float:
        if self.fallback_cy >= 0.0:
            return self.fallback_cy
        return 0.5 * (height - 1)

    def _convert_color_image(self, msg: Image) -> None:
        target = self.output_color_encoding
        if target in ("", "preserve") or msg.encoding.lower() == target:
            return
        if target != "bgra8" or np is None:
            if not self._warned_color_conversion:
                self.get_logger().warning(
                    "Cannot convert image encoding %s -> %s; publishing as %s"
                    % (msg.encoding, target, msg.encoding))
                self._warned_color_conversion = True
            return

        src = msg.encoding.lower()
        width = int(msg.width)
        height = int(msg.height)
        try:
            if src == "rgb8":
                rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    height, msg.step)[:, :width * 3].reshape(height, width, 3)
                bgra = np.empty((height, width, 4), dtype=np.uint8)
                bgra[..., 0] = rgb[..., 2]
                bgra[..., 1] = rgb[..., 1]
                bgra[..., 2] = rgb[..., 0]
                bgra[..., 3] = 255
            elif src == "bgr8":
                bgr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    height, msg.step)[:, :width * 3].reshape(height, width, 3)
                bgra = np.empty((height, width, 4), dtype=np.uint8)
                bgra[..., :3] = bgr
                bgra[..., 3] = 255
            elif src == "rgba8":
                rgba = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    height, msg.step)[:, :width * 4].reshape(height, width, 4)
                bgra = rgba[..., [2, 1, 0, 3]].copy()
            else:
                if not self._warned_color_conversion:
                    self.get_logger().warning(
                        "Unsupported color conversion %s -> %s; publishing as %s"
                        % (msg.encoding, target, msg.encoding))
                    self._warned_color_conversion = True
                return
        except ValueError:
            if not self._warned_color_conversion:
                self.get_logger().warning(
                    "Image buffer shape does not match %sx%s %s; publishing "
                    "without conversion" % (width, height, msg.encoding))
                self._warned_color_conversion = True
            return

        msg.encoding = "bgra8"
        msg.step = width * 4
        msg.is_bigendian = 0
        msg.data = array("B", bgra.reshape(-1).tobytes())

    def _publish_static_camera_transforms(self) -> None:
        stamp = self.get_clock().now().to_msg()
        transforms = []

        parent_frame = self.camera_link_frame_id
        if self.camera_center_frame_id:
            to_center = TransformStamped()
            to_center.header.stamp = stamp
            to_center.header.frame_id = self.camera_link_frame_id
            to_center.child_frame_id = self.camera_center_frame_id
            to_center.transform.rotation.w = 1.0
            transforms.append(to_center)
            parent_frame = self.camera_center_frame_id

        to_camera_frame = TransformStamped()
        to_camera_frame.header.stamp = stamp
        to_camera_frame.header.frame_id = parent_frame
        to_camera_frame.child_frame_id = self.camera_frame_id
        to_camera_frame.transform.rotation.w = 1.0
        transforms.append(to_camera_frame)

        to_optical = TransformStamped()
        to_optical.header.stamp = stamp
        to_optical.header.frame_id = self.camera_frame_id
        to_optical.child_frame_id = self.optical_frame_id
        qx, qy, qz, qw = _q_from_rpy(-math.pi / 2.0, 0.0, -math.pi / 2.0)
        to_optical.transform.rotation.x = qx
        to_optical.transform.rotation.y = qy
        to_optical.transform.rotation.z = qz
        to_optical.transform.rotation.w = qw
        transforms.append(to_optical)

        self.tf_static.sendTransform(transforms)


def main(argv: list[str] | None = None) -> int:
    rclpy.init(args=sys.argv if argv is None else argv)
    node = IgvcCameraBridge()
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

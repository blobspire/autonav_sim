# Real Bag To Sim Alignment

Last updated: 2026-05-31.

This note records the current real-robot bag measurements used to tune the
Gazebo/IGVC sim. Keep this file current when future bags change the measured
camera, depth, odom, or detector behavior.

## Source Bags

Real robot bags sampled from:

- `/Users/cole/autonav_bags/jetson_real_robot_calibration_20260530_125258/camera_line_static_canon_two_feet/bag`
- `/Users/cole/autonav_bags/jetson_real_robot_calibration_20260530_125258/camera_line_static_canon_four_feet/bag`
- `/Users/cole/autonav_bags/jetson_real_robot_calibration_20260530_125258/camera_line_static_canon_diagonal_two_feet/bag`
- `/Users/cole/autonav_bags/jetson_real_robot_calibration_20260530_125258/camera_line_detection_rerun/bag`
- `/Users/cole/autonav_bags/jetson_real_robot_calibration_20260530_125258/manual_course_rerun_7/bag`

Focused CUDA replay and detector analysis should run on the Jetson, not the
Mac. The Mac can read bag metadata and SQLite storage, but it cannot run the
CUDA detector path because it has no NVIDIA CUDA device.

## ZED Camera Contract

The sim camera bridge should publish the same topic shape as the real ZED bags:

- RGB topic: `/zed/zed_node/rgb/color/rect/image`
- RGB encoding: `bgra8`
- RGB size: `960x540`
- RGB frame: `zed_left_camera_frame_optical`
- Depth topic: `/zed/zed_node/depth/depth_registered`
- Depth encoding: `32FC1`
- Depth size: `960x540`
- Depth frame: `zed_left_camera_frame_optical`
- Camera info: `fx=539.702`, `fy=539.702`, `cx=472.965`, `cy=255.161`
- Effective rectified horizontal FOV: about `1.453833` rad

The Gazebo camera should use the rectified FOV derived from bag intrinsics, not
the raw marketing FOV. The detector consumes rectified images, so the rectified
projection is the behavior to match.

## Current Sim Changes

The sim now:

- renders the Gazebo camera at about `1.453833` rad horizontal FOV,
- republishes RGB as `bgra8`,
- republishes camera/depth frames as `zed_left_camera_frame_optical`,
- fills or overrides camera info with the bag-derived ZED intrinsics, and
- publishes the real ZED static TF chain:
  `zed_camera_link -> zed_camera_center -> zed_left_camera_frame -> zed_left_camera_frame_optical`.

Live VM smoke check after the change:

- `/zed/zed_node/rgb/color/rect/image`: `bgra8`, step `3840`, frame `zed_left_camera_frame_optical`
- `/zed/zed_node/depth/depth_registered`: `32FC1`
- `/zed/zed_node/rgb/color/rect/camera_info`: bag-derived K/P values above
- camera rate: about `11 Hz`
- depth rate: about `10.5 Hz`
- `/odom` rate: about `25 Hz`

## Real Bag Behavior

Raw ZED image/depth recording is sparse in several selected bags. Do not tune
the sim camera down to the sparse raw bag recording rate without confirming a
dedicated high-rate camera bag; the sparsity is likely recorder/QoS/bandwidth
related. Use downstream detector diagnostics, costmaps, and camera header
fields for sim alignment.

Observed examples:

- `manual_course_rerun_7`: raw RGB/depth recorded sparsely, while detector and
  costmap topics continued to publish.
- `camera_line_static_canon_diagonal_two_feet`: better raw recording coverage,
  but projected `/line_points` was empty in the sampled messages.
- `camera_line_static_canon_two_feet`: `/line_points` published at about `3.2 Hz`
  with non-empty points in about half the sampled messages.
- `manual_course_rerun_7`: `/line_points` published at about `2.2 Hz` with
  non-empty points in about 70 percent of sampled messages.

Real `/local_ekf/odom` in the sampled manual bag was materially slower than the
current sim odom bridge. It was roughly in the high-single-digit to low-teens Hz
range depending on the bag window, with noticeable jitter. Current sim odom is
about `25 Hz`. This is not currently blocking sim use, but it is a fidelity gap
to revisit if Jetson load, TF timing, or detector synchronization diverges from
real robot behavior.

## Jetson CUDA Workflow

Use the Jetson for CUDA-dependent replay:

1. Copy a focused bag to the Jetson or into the `koopa-kingdom` container.
2. Run the camera line detector inside the container against replayed ZED RGB,
   depth, camera info, `/tf`, and `/tf_static`.
3. Compare detector diagnostics, `/line_points`, `/line_costmap`, and Nav2
   behavior against the live sim run.

Metadata-only comparisons can stay on the Mac. CUDA detector timing and
projection behavior should not be considered validated until replayed or run
live on the Jetson.

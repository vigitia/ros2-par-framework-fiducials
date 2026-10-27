# Claude Code usage documentation

## 2026-09-24 — Compatibility with the Logitech Brio node, detector parameters (branch `rework`)

### What changed
- `fiducials_detector_node.py`
  - The subscribed topic is the parameter `topic_name_frame`, default `/camera/brio/image_raw`
    (it was hardcoded to `/camera/generic_webcam_frame_full`, which nothing publishes anymore).
  - The subscription uses BEST_EFFORT / KEEP_LAST depth 1. The old `qos_profile=10` means RELIABLE,
    which cannot connect to the Brio's best-effort publisher, so the node silently got no frames.
  - Frames are converted with `imgmsg_to_cv2(..., desired_encoding='mono8')`.
  - New read-only parameters: `marker_type` (aruco | apriltag), `dictionary`, `corner_refinement`
    (none | subpix | contour | apriltag), `adaptive_thresh_constant`.
  - The detector is created before the subscription, so invalid parameters fail at startup.
- `fiducials_detector.py`: `FiducialsDetector.__init__` takes the settings above, validates them and
  lists the valid values in its error message. Removed the unused `ADAPTIVE_THRESH_CONSTANT` and the
  commented-out `ARUCO_DICT` alternatives, which the parameters replace.
- `package.xml`: declared `rcl_interfaces`, `sensor_msgs` and `cv_bridge`. They were imported but
  undeclared, so the node only built because the Brio's dependencies pulled them in.
- `requirements.txt`: `opencv-python==4.11.0.86`, the same pin as `logitech_brio`. All
  requirements go through one `pip install`, so a different pin would fail the image build.

### Why / alternatives
- **mono8 instead of converting in the detector:** `imgmsg_to_cv2` without an encoding returns
  the frame's native channel order. With the Brio's `encoding: rgb8`, the detector's
  `COLOR_BGR2GRAY` would have used swapped channel weights. Letting cv_bridge convert handles
  bgr8, rgb8 and mono8. The detector keeps its own greyscale step for the standalone
  `__main__` test.
- **`marker_type` + short `dictionary` name** instead of one full name like `DICT_APRILTAG_36h11`:
  you asked for an explicit aruco/apriltag switch. An empty dictionary falls back to
  a default per type (4X4_100, the previous hardcoded value, and 36h11), so switching the type
  alone works. The name is resolved with `getattr(cv2.aruco, ...)`, which is simpler than a
  hand-maintained table, and it picks up any dictionary OpenCV adds.
- **Read-only parameters:** the subscription and the `ArucoDetector` are created once, so a
  runtime change would do nothing. This matches the Brio's `qos_reliability`/`encoding`. A
  parameter callback that rebuilds the detector was possible, but nothing needs it yet.
- **`adaptive_thresh_constant` defaults to 7.0**, the OpenCV default and the value actually in
  effect before. The old constant 10 was never applied.

## 2026-09-24 — Publish empty fiducial lists

### What changed
`listener_callback` publishes a `FiducialsList` for every frame, also when no marker was found.
Before, it published only when at least one marker was detected.

### Why / alternatives
Without the empty message, a subscriber can't tell a marker was removed: its last position
stayed valid forever. The alternative was a timeout on the subscriber side, which every consumer
would have to reimplement and tune. Publishing every frame costs one small message per camera frame.

## 2026-09-24 — Built and tested; lint fixes for the new code

### What changed
Rewrapped the lines added above to flake8's 99 characters and moved the new `rcl_interfaces`
import before `rclpy` (alphabetical order). The 15 remaining flake8 errors are all in older code
and were left as they were.

### Tested
Built on the host (`/opt/ros/lyrical` + `.venv`, `colcon build --packages-up-to fiducials_detector`).
- Stub publisher standing in for the Brio (best-effort, depth 1, `rgb8`, 1920×1080): the marker
  image gave ids `[9, 11, 10, 7]`, blank frames gave empty lists.
- Real Brio with `bringup/config/par_setup.yaml`: the node receives its frames
  (`frame_id: brio_optical_frame`) and publishes `/fiducials` at about 21 Hz while the Brio captures at
  60 fps. Processing a 1080p frame takes 20–35 ms. Many best-effort frames are also lost before the
  callback, even at 640×480, so part of the gap is transport, not the node.
- `colcon test` runs 0 tests on this host (it falls back to unittest), so the lint tests were run
  with `python -m pytest test/` directly: flake8 fails on the older code only, pep257 passes.

## OpenCV from opencv-contrib-python

**What changed:** `requirements.txt` pins `opencv-contrib-python==4.11.0.86` instead of `opencv-python==4.11.0.86`.

**Why:** framework-wide switch, because `mediapipe` (hand_landmarks node) depends on the contrib build and both
distributions install into the same `cv2/` directory, silently shadowing each other. Same OpenCV version, so this
node's behaviour does not change. Full reasoning in `ros2-par-framework/claude_usage_documentation.md`.

### Clean shutdown on Ctrl-C / launch shutdown (2026-09-25)

**What:** `main()` catches `KeyboardInterrupt` and `ExternalShutdownException` around `rclpy.spin()`. It also
swallows other exceptions once `rclpy.ok()` is false, and ends with `rclpy.try_shutdown()`. This is the same pattern
as `hand_landmarks`.

**Why:** when a launch ends (e.g. `projection_calibration_kinect.launch.py` after the calibrator finishes), each node
gets SIGINT. Before, this node died with a traceback: `fiducials_detector` from the uncaught `KeyboardInterrupt`,
`object_detector` from the race where a callback still publishes after the context is shut down
(`RCLError: publisher's context is invalid`). Errors while the context is still valid are still re-raised, so real
bugs keep surfacing.
- Alternative: a shared helper for all nodes. Rejected: each node is its own repo, and a shared package only for
  this would be a new dependency.

**Tested:** `par_framework_kinect.launch.py` live, stopped with SIGINT after 60 s: all nine processes report
"finished cleanly", with no traceback.

### 3D marker corners from a registered depth image (2026-09-25)

**What:**
- **`fiducials_detector.py`:** new module function `corners_to_camera_space(corners, depth_m, k, d)`.
  - It takes the valid depth pixels inside the marker polygon, shrunk to 70 % towards the centre, and makes them 3D
    points (undistorted rays times depth).
  - It fits a plane (SVD), drops points more than 1 cm off it (e.g. a finger on the marker), and fits again.
  - Result: the four corners' rays intersected with the plane, as a 4x3 array in metres in the optical frame. None
    with fewer than 20 valid pixels, or when a ray does not meet the plane in front of the camera.
  - The mask is built on the bounding box only.
- **`fiducials_detector_node.py`:**
  - new read-only parameter `use_depth` (default false);
  - with it: colour and depth pairs (`ApproximateTimeSynchronizer`), `camera_info` (Brio name, remapped by the Kinect
    launch), `corners_3d` per marker, the same missing-depth warning as `hand_landmarks` (resolved topic name), and a
    size check against the colour image;
  - without it: unchanged. `listener_callback` gets an optional depth image.
- **`package.xml`:** `geometry_msgs` and `message_filters` declared.
- **`test/test_corners_to_camera_space.py`** (new): flat marker, marker tilted by 30°, a finger 4 cm in front of the
  marker, too little depth.

**Why:** your request, markers in 3D in RViz like the hands.
- **Plane fit instead of a window per corner (your choice):** a marker is flat. Hundreds of pixels from the inner
  area are robust against holes and edge pixels, and the corners need no depth of their own. A window around a corner
  would be only a quarter marker, and four independent depths can bend the square.
- **Unchanged:** the pixel fields, so the projector calibration is unaffected.

**Tested:** `pytest` 4/4. My first tilted test wrongly assumed a square in 3D: a square in pixels on a tilted plane is
a trapezoid, so it now checks the exact ray-plane intersection. `ament_flake8` has no new findings compared to HEAD.
Live test pending.

### Orientation from the image (IPPE), distance from the depth (2026-09-25)

**What:** `corners_to_camera_space` no longer uses the depth plane for the orientation.
- **Coverage:** at least 50 % of the inner area must have depth (`MIN_DEPTH_COVERAGE`).
- **Orientation:** `cv2.solvePnPGeneric` with `SOLVEPNP_IPPE_SQUARE` for a square of edge 1 (`UNIT_SQUARE`). Of its
  two solutions, only those with at most 3 px reprojection error (`MAX_REPROJECTION_ERROR_PX`) count, and the one
  whose normal is closest to the depth plane's normal wins.
- **Distance:** the median depth of the inner area divided by the unit solution's distance gives the scale. Scaling
  about the camera origin keeps every corner on its ray.
- **Removed:** the plane refit (`MAX_PLANE_DISTANCE_M`), the square check (`MAX_SQUARE_RATIO`) and `_fit_plane`. The
  result is a square by construction.
- **Tests:** the pixel-square flat test and the "no square" test are gone. New: a marker facing the camera (1°), the
  wrong depth tilt (depth plane at 40°, marker at 30°), a kite (no view of a square gives None); the finger test now
  uses a real 1° square.

**Why:** live on your cube (marker edge 5.7 cm), the depth plane showed steep side faces 10° too steep: #5 became
60 x 77 mm with 100 % depth coverage, most likely ToF multipath in the corner between face and table. Marker #6 had
13 % coverage, and those pixels were the table.
- **Your choice:** orientation from the image, distance from the depth.
- **Alternatives:** keep only strict rejection (#5 would have had no 3D), or loosen the square check (#5 visibly
  stretched).
- IPPE's degenerate case (a noise-free square facing the camera exactly) only occurs in synthetic data: at 0.01° or
  with 0.3 px corner noise, the error was below 0.25 px. The 3 px limit catches it anyway.

**Tested:**
- `pytest` 7/7; `ament_flake8` has no new findings compared to HEAD.
- Live, with a second node next to your launch:
  - #2: 57.9 mm (57.3-58.1);
  - #5: 59.2 mm, viewing angle 50° (the depth plane had said 61°);
  - angle between the faces #2/#5: 90° (89-90°);
  - #6 (no depth) correctly without 3D.

### Camera topics renamed to the PAR framework's neutral interface (2026-10-01)

**What:** the camera topics are `/camera/image_raw` (object_detector: `/camera/image_raw/compressed`),
`/camera/camera_info` and, with depth, `/camera/depth_registered/image_raw`, instead of the Brio's
`/camera/brio/...` and the Azure Kinect's `/depth_to_rgb/image_raw`.

**Why:** the framework's launch files now remap every camera driver onto these names
(`launch_helpers.DRIVER_REMAPPINGS` in ros2-par-framework), instead of remapping every consumer onto the Brio's names.
Run outside the framework, remap the camera's topics onto these names or set `topic_name_frame`.

## 2026-10-13 — `corners_to_camera_space`: reduced SVD

- **What:** the plane fit over the depth points inside a marker calls `np.linalg.svd(..., full_matrices=False)`.
- **Why:** only the 3x3 `Vt` is used, but the default full SVD also builds `U`, an N x N matrix for the N depth pixels
  of the marker. Its cost grows with the square of the marker's pixel count: negligible for small markers, but for
  markers that are large in the image (e.g. projected calibration markers on a high-resolution projector) it took
  seconds per marker. With `use_depth`, colour and depth frames are kept one deep, so frames arriving during such a
  slow callback overwrite each other, pairs of the same capture rarely meet, and `/fiducials` was published only
  sporadically. The result is identical; the reduced SVD runs in milliseconds.
- **Alternatives:** the eigenvectors of the 3x3 covariance matrix (`np.linalg.eigh`): equally fast, but a second way
  of writing the same plane fit; the other depth plane fits of the framework (`object_detector`,
  `camera_pose_calibrator`) already use the reduced SVD. Subsampling the depth pixels: rejected, it trades accuracy
  for speed while the full-matrix cost was the only problem. Deeper subscription queues: not needed once the callback
  is fast, and they would add latency.

## 2026-10-22: `topic_name_frame` removed

**What changed:** the node subscribes to the constant `TOPIC_NAME_FRAME` (`/camera/image_raw`), next to the depth and camera_info topics; the `topic_name_frame` parameter is gone.

**Why:** remapping is ROS 2's way to choose another topic and covers every topic; nothing set the parameter. Reasoning for the whole framework in the framework repo's log.

**Alternatives:** keeping the parameter: rejected, it duplicated remapping for one topic only, and a compressed topic name would have subscribed with the wrong message type.

## 2026-10-22: comments and descriptions no longer name camera models

**What changed:** the file header, the comments on the camera topics, the QoS and the depth start-up, and the `use_depth` description describe camera behaviour in general terms (drivers that publish best-effort, a depth camera, a driver that only computes registered depth once it sees a subscriber) instead of naming the Brio or the Azure Kinect. The `use_depth` description uses double quotes instead of an escaped apostrophe, which also clears flake8's Q003.

**Why:** the node only sees the framework's neutral `/camera/...` topics and does not know which camera runs; a reason tied to one model reads as if the code only applied to it.

**Alternatives:** keeping the model as an example: rejected, the node should not suggest knowledge of the connected camera.

## 2026-10-23: private methods with one underscore instead of two

**What changed:** methods named `__method` are `_method`, with their calls; behaviour is unchanged. Methods in hand-written files that changed carry a `Co-authored by Claude Code` marker.

**Why:** one underscore is Python's convention for "private"; two trigger name mangling, which only helps against name clashes in subclasses, and none of these classes is subclassed. Each new name was checked against the file's other names and against `rclpy.node.Node`'s members first. Reasoning in the framework repo's log.

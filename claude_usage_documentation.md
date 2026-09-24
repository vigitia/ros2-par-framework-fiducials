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

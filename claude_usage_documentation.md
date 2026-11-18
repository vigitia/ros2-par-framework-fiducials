# Claude Code usage documentation

## 2026-09-24: Compatibility with the Logitech Brio node, detector parameters

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
  an explicit aruco/apriltag switch. An empty dictionary falls back to a default per type
  (4X4_100, the previous hardcoded value, and 36h11), so switching the type alone works. The name
  is resolved with `getattr(cv2.aruco, ...)`, which is simpler than a hand-maintained table, and it
  picks up any dictionary OpenCV adds.
- **Read-only parameters:** the subscription and the `ArucoDetector` are created once, so a
  runtime change would do nothing. This matches the Brio's `qos_reliability`/`encoding`. A
  parameter callback that rebuilds the detector was possible, but nothing needs it yet.
- **`adaptive_thresh_constant` defaults to 7.0**, the OpenCV default and the value actually in
  effect before. The old constant 10 was never applied.

## 2026-09-24: Publish empty fiducial lists

### What changed
`listener_callback` publishes a `FiducialsList` for every frame, also when no marker was found.
Before, it published only when at least one marker was detected.

### Why / alternatives
Without the empty message, a subscriber can't tell a marker was removed: its last position
stayed valid forever. The alternative was a timeout on the subscriber side, which every consumer
would have to reimplement and tune. Publishing every frame costs one small message per camera frame.

## 2026-09-24: Lint fixes for the new code

### What changed
Rewrapped the lines added above to flake8's 99 characters and moved the new `rcl_interfaces`
import before `rclpy` (alphabetical order). The remaining flake8 errors are all in older code
and were left as they were.

### Why / alternatives
- The package's `test_flake8` checks 99 characters. Fixing the older code as well was left out:
  it was not part of this change.
- Checked with a stub publisher standing in for the Brio (best-effort, depth 1, `rgb8`) and with
  the real camera: markers are found, blank frames give empty lists. The node publishes
  `/fiducials` at a lower rate than the camera captures; part of the gap is best-effort frames
  lost in transport before the callback, not the node.

## 2026-09-24: OpenCV from opencv-contrib-python

### What changed
`requirements.txt` pins `opencv-contrib-python==4.11.0.86` instead of `opencv-python==4.11.0.86`.

### Why / alternatives
Framework-wide switch, because `mediapipe` (hand_landmarks node) depends on the contrib build and
both distributions install into the same `cv2/` directory, silently shadowing each other. Same
OpenCV version, so this node's behaviour does not change. Full reasoning in
`ros2-par-framework/claude_usage_documentation.md`.

## 2026-09-25: Clean shutdown on Ctrl-C / launch shutdown

### What changed
`main()` catches `KeyboardInterrupt` and `ExternalShutdownException` around `rclpy.spin()`. It also
swallows other exceptions once `rclpy.ok()` is false, and ends with `rclpy.try_shutdown()`. This is
the same pattern as `hand_landmarks`.

### Why / alternatives
- When a launch ends (e.g. `projection_calibration_kinect.launch.py` after the calibrator
  finishes), each node gets SIGINT. Before, this node died with a traceback: `fiducials_detector`
  from the uncaught `KeyboardInterrupt`, `object_detector` from the race where a callback still
  publishes after the context is shut down (`RCLError: publisher's context is invalid`). Errors
  while the context is still valid are still re-raised, so real bugs keep surfacing.
- Rejected: a shared helper for all nodes. Each node is its own repo, and a shared package only for
  this would be a new dependency.
- Checked by stopping a full launch with SIGINT: every process reports "finished cleanly", with no
  traceback.

## 2026-09-25: 3D marker corners from a registered depth image

### What changed
- **`fiducials_detector.py`:** new module function `corners_to_camera_space(corners, depth_m, k, d)`.
  - It takes the valid depth pixels inside the marker polygon, shrunk to 70 % towards the centre,
    and makes them 3D points (undistorted rays times depth).
  - It fits a plane (SVD), drops points more than 1 cm off it (e.g. a finger on the marker), and
    fits again.
  - Result: the four corners' rays intersected with the plane, as a 4x3 array in metres in the
    optical frame. None with fewer than 20 valid pixels, or when a ray does not meet the plane in
    front of the camera.
  - The mask is built on the bounding box only.
- **`fiducials_detector_node.py`:**
  - new read-only parameter `use_depth` (default false);
  - with it: colour and depth pairs (`ApproximateTimeSynchronizer`), `camera_info` (Brio name,
    remapped by the Kinect launch), `corners_3d` per marker, the same missing-depth warning as
    `hand_landmarks` (resolved topic name), and a size check against the colour image;
  - without it: unchanged. `listener_callback` gets an optional depth image.
- **`package.xml`:** `geometry_msgs` and `message_filters` declared.
- **`test/test_corners_to_camera_space.py`** (new): flat marker, marker tilted by 30°, a finger
  4 cm in front of the marker, too little depth.

### Why / alternatives
- Markers in 3D in RViz, like the hand landmarks.
- **Plane fit instead of a window per corner:** a marker is flat. Hundreds of pixels from the inner
  area are robust against holes and edge pixels, and the corners need no depth of their own. A
  window around a corner would be only a quarter marker, and four independent depths can bend the
  square.
- **Unchanged:** the pixel fields, so the projector calibration is unaffected.
- The tilted test checks the exact ray-plane intersection: a square in pixels on a tilted plane is
  a trapezoid in 3D, not a square.

## 2026-09-25: Orientation from the image (IPPE), distance from the depth

### What changed
`corners_to_camera_space` no longer uses the depth plane for the orientation.
- **Coverage:** at least 50 % of the inner area must have depth (`MIN_DEPTH_COVERAGE`).
- **Orientation:** `cv2.solvePnPGeneric` with `SOLVEPNP_IPPE_SQUARE` for a square of edge 1
  (`UNIT_SQUARE`). Of its two solutions, only those with at most 3 px reprojection error
  (`MAX_REPROJECTION_ERROR_PX`) count, and the one whose normal is closest to the depth plane's
  normal wins.
- **Distance:** the median depth of the inner area divided by the unit solution's distance gives
  the scale. Scaling about the camera origin keeps every corner on its ray.
- **Removed:** the plane refit (`MAX_PLANE_DISTANCE_M`), the square check (`MAX_SQUARE_RATIO`) and
  `_fit_plane`. The result is a square by construction.
- **Tests:** the pixel-square flat test and the "no square" test are gone. New: a marker facing the
  camera (1°), the wrong depth tilt (depth plane at 40°, marker at 30°), a kite (no view of a
  square gives None); the finger test now uses a real 1° square.

### Why / alternatives
- On markers on the steep side faces of an object, the depth plane came out tilted too steeply,
  most likely from time-of-flight multipath in the corner between face and table, which stretched
  the marker in 3D. On faces with little depth coverage, the depth pixels belonged to the table.
- Chosen: orientation from the image, distance from the depth.
- Rejected: only strict rejection (such markers would have had no 3D), or a looser square check
  (the markers visibly stretched).
- IPPE's degenerate case (a noise-free square facing the camera exactly) only occurs in synthetic
  data: with a slight tilt or a little corner noise the reprojection error stays small. The 3 px
  limit catches it anyway.
- Checked live on a cube: edge lengths and the angle between faces match the real cube closely,
  and a marker without depth correctly gets no 3D.

## 2026-10-01: Camera topics renamed to the PAR framework's neutral interface

### What changed
The camera topics are `/camera/image_raw` (object_detector: `/camera/image_raw/compressed`),
`/camera/camera_info` and, with depth, `/camera/depth_registered/image_raw`, instead of the Brio's
`/camera/brio/...` and the Azure Kinect's `/depth_to_rgb/image_raw`.

### Why / alternatives
The framework's launch files now remap every camera driver onto these names
(`launch_helpers.DRIVER_REMAPPINGS` in ros2-par-framework), instead of remapping every consumer onto
the Brio's names. Run outside the framework, remap the camera's topics onto these names or set
`topic_name_frame`.

## 2026-10-13: `corners_to_camera_space`: reduced SVD

### What changed
The plane fit over the depth points inside a marker calls `np.linalg.svd(..., full_matrices=False)`.

### Why / alternatives
- Only the 3x3 `Vt` is used, but the default full SVD also builds `U`, an N x N matrix for the N
  depth pixels of the marker. Its cost grows with the square of the marker's pixel count:
  negligible for small markers, but far too slow for markers that are large in the image (e.g.
  projected calibration markers on a high-resolution projector). With `use_depth`, colour and depth
  frames are kept one deep, so frames arriving during such a slow callback overwrite each other,
  pairs of the same capture rarely meet, and `/fiducials` was published only sporadically. The
  result is identical, and the reduced SVD is cheap.
- Rejected: the eigenvectors of the 3x3 covariance matrix (`np.linalg.eigh`): equally fast, but a
  second way of writing the same plane fit; the other depth plane fits of the framework
  (`object_detector`, `camera_pose_calibrator`) already use the reduced SVD.
- Rejected: subsampling the depth pixels. It trades accuracy for speed while the full-matrix cost
  was the only problem.
- Rejected: deeper subscription queues. Not needed once the callback is fast, and they would add
  latency.

## 2026-10-22: `topic_name_frame` removed

### What changed
The node subscribes to the constant `TOPIC_NAME_FRAME` (`/camera/image_raw`), next to the depth and
camera_info topics; the `topic_name_frame` parameter is gone.

### Why / alternatives
- Remapping is ROS 2's way to choose another topic and covers every topic; nothing set the
  parameter. Reasoning for the whole framework in `ros2-par-framework/claude_usage_documentation.md`.
- Rejected: keeping the parameter. It duplicated remapping for one topic only, and a compressed
  topic name would have subscribed with the wrong message type.

## 2026-10-22: Comments and descriptions no longer name camera models

### What changed
The file header, the comments on the camera topics, the QoS and the depth start-up, and the
`use_depth` description describe camera behaviour in general terms (drivers that publish
best-effort, a depth camera, a driver that only computes registered depth once it sees a
subscriber) instead of naming the Brio or the Azure Kinect. The `use_depth` description uses double
quotes instead of an escaped apostrophe, which also clears flake8's Q003.

### Why / alternatives
- The node only sees the framework's neutral `/camera/...` topics and does not know which camera
  runs; a reason tied to one model reads as if the code only applied to it.
- Rejected: keeping the model as an example. The node should not suggest knowledge of the
  connected camera.

## 2026-10-23: Private methods with one underscore instead of two

### What changed
Methods named `__method` are `_method`, with their calls; behaviour is unchanged. Methods in
hand-written files that changed carry a `Co-authored by Claude Code` marker.

### Why / alternatives
One underscore is Python's convention for "private"; two trigger name mangling, which only helps
against name clashes in subclasses, and none of these classes is subclassed. Each new name was
checked against the file's other names and against `rclpy.node.Node`'s members first. Reasoning in
`ros2-par-framework/claude_usage_documentation.md`.

## 2026-10-27: Optional compressed input (`use_compressed`)

### What changed
With `use_compressed`, the node subscribes to `/camera/image_raw/compressed`
(`sensor_msgs/CompressedImage`) instead of `/camera/image_raw` and decodes each JPEG with
`cv2.imdecode(..., IMREAD_GRAYSCALE)`. The message type is chosen once and used for both the plain
and the `use_depth` subscription, as in `object_detector`. The check that the depth image is on the
colour image's pixel grid moved from `listener_callback_with_depth` into `listener_callback`, after
decoding, because `CompressedImage` has no width and height. A frame that cannot be decoded is
skipped with a throttled warning.

### Why / alternatives
- A camera driver that passes its JPEGs through then needs no decode for this node, a raw frame is
  many times larger than its JPEG to transfer, and decoding straight to greyscale skips the colour
  channels the detector never uses. The greyscale from the JPEG's brightness channel differs from
  cv_bridge's colour-to-mono conversion only by rounding.
- Rejected: a separate callback for compressed frames. Two copies of the detection path; the
  `isinstance` branch keeps one, like `object_detector`.
- Rejected: forbidding `use_compressed` together with `use_depth`. message_filters synchronises
  `CompressedImage` like `Image`, so the combination costs nothing.

## 2026-10-27: Names say what they hold, with units

### What changed
Constants, functions, methods and variables in all three modules are renamed after the naming
rules: units in names (`_px`, `_m`, `_deg`, `_sec`), no single letters or generic words, a verb for
functions, and a leading underscore for methods only used inside their class. Notable ones:
`corners_to_camera_space` is `compute_corners_in_camera_space(corners_px, depth_image_m,
camera_matrix, distortion_coefficients)`, the node's callbacks are `_on_frame_received` /
`_on_frame_and_depth_received`, `convert_to_fiducials_interface_msg` is `_create_fiducials_list_msg`,
the `FiducialsDetector` constructor takes `ros_node` and `dictionary_name`,
`FiducialsDetectorDebugger` is `DetectionAccuracyDebugger`, and `DEPTH_MISSING_SEC` is
`DEPTH_MISSING_TIMEOUT_SEC`. Index loops over the detected markers and pose candidates became `zip`
loops; the message dictionary keys (`corners`, `centroid`, ...) and the ROS parameter names are
unchanged.

### Why / alternatives
- The names must be readable on their own, e.g. in a traceback or a grep, and the 3D corner code
  mixes pixels and metres.
- Checked: the committed and the renamed module give identical detections (ids, angles, corners,
  centroids) on a rendered image and identical 3D corners for a flat and a tilted square on
  synthetic depth; the node starts with and without `use_depth`. Nothing outside this repository
  used the renamed names.

## 2026-10-27: Detector without ROS node, debugger counters per instance

### What changed
- `FiducialsDetector` no longer takes a ROS node. `detect_fiducials` only detects; drawing the
  markers in a window moved into the module's `__main__` block, which now takes the image path as
  argument (`python -m fiducials_detector.fiducials_detector <image path>`), because the image it
  read does not exist.
- `DetectionAccuracyDebugger` sets its counters in `__init__` instead of as class attributes, and
  its prints start with `[DetectionAccuracyDebugger]: `.
- `detect_fiducials` checks `marker_ids is not None` instead of `np.all(marker_ids is not None)`.
- The node no longer logs a greeting and the working directory at startup.

### Why / alternatives
- The node was passed only so that a missing node could open a window that blocks on
  `cv2.waitKey(0)`; detection logic stays free of ROS objects, as the framework's node convention
  asks.
- Class attributes are shared by all instances, so a list in one debugger showed up in every other
  one, and the interval started at import instead of at creation.
- The `np.all` around a single boolean did nothing. The startup lines carried no information.
- Rejected: a `show_detections` flag in the constructor. Nothing but the standalone test uses it,
  and a window inside a node callback would block it.

## 2026-10-27: Node split into one method per step, depth arrival monitor as a class

### What changed
`__init__` delegates the subscriptions to `_create_subscriptions` (the name the other nodes use),
which leaves the depth pairing to `_create_frame_and_depth_subscriptions`; `_on_frame_received`
calls `_decode_frame_mono`, `_get_depth_image_on_frame_grid`, the detector and `_add_corners_3d` in
turn. The three "does depth arrive" timestamps and their condition moved into `DepthArrivalMonitor`
in `fiducials_detector.py` (no ROS, testable with plain pytest); the node only feeds it times and
logs the warning. `_create_fiducials_list_msg` unpacks the corners into keyword arguments and takes
a required header. Timer period, warning throttle and the millimetre factor are named constants;
the log messages no longer repeat the node name (the logger adds it); imports are one sorted block;
the commented-out debug line and the per-block AI markers are gone (one note at the top, one marker
per method).

### Why / alternatives
- Both methods did several things inline, and the monitor's state existed only in depth mode,
  spread over three callbacks.
- Rejected: keeping the monitor in the node with a docstring. Its state and condition are plain
  logic, which the node convention puts outside the node.
- Checked: the old and the new message creation give equal messages; the monitor gives the old
  condition's answer at every step of several frame/depth timelines; a synthetic two-marker frame
  through the node gives the same `/fiducials` raw and as JPEG; the node starts in depth mode.

## 2026-10-27: Debug mode as a ROS parameter, with a preview window

### What changed
The module constant `DEBUG_DETECTION_ACCURACY` is replaced by the read-only node parameter `debug`,
passed to `FiducialsDetector(debug=...)`. With it, every frame is shown in the OpenCV window
`fiducials_detector` with the detected markers outlined and labelled with their ids, and the
detection accuracy is printed as before. The `__main__` test uses the same preview instead of
drawing on its own, and no longer squeezes the image to 800x600.

### Why / alternatives
- A constant needed a code change to switch; a parameter can be set per run (`debug:=true`) or in
  `par_setup.yaml`. The name `debug` and the preview (fixed width of 800 px, resizable window sized
  once, `waitKey(1)`) follow `hand_landmarks` and `gesture_detector`, so all nodes switch and look
  the same. The frame is scaled down before drawing, so the preview costs as little detection time
  as possible.
- Rejected: a separate debug image topic for RViz or rqt. The other nodes use a window, and
  publishing a second image stream costs more than drawing a small one.
- Left out: drawing the rejected marker candidates too. Not needed yet, but useful when tuning
  `adaptive_thresh_constant`.

## 2026-10-27: Detection accuracy printed once per interval

### What changed
`DetectionAccuracyDebugger.on_new_frame` prints the accuracy of the interval that just ended, once
per `RESET_INTERVAL_SEC`, instead of a running value for every frame. The check and reset happen
before the new frame is counted, so a frame and its markers always land in the same interval.

### Why / alternatives
With `debug` now a parameter, the debugger runs live at the camera's frame rate, where one line per
frame floods the terminal.

## 2026-10-27: Multiple-markers warning once per interval

### What changed
`on_new_marker` only collects the ids; `on_new_frame` prints the "multiple markers" warning once per
interval, after the accuracy line and with the number of ids seen.

### Why / alternatives
Printed for every marker of every frame, the warning flooded the terminal as soon as a page with
four markers was in view, and hid the accuracy line it qualifies (with several markers the accuracy
exceeds 100 %).

## 2026-10-27: Detection rate per marker instead of the multiple-markers warning

### What changed
`DetectionAccuracyDebugger` counts detections per marker id and prints, once per interval, one line
with the rate of every marker seen in it, e.g. `Detection rate over 61 frames: id 96: 100 %, id 97:
51 %`, or `no markers`. The overall accuracy line and the "multiple markers" warning are gone.

### Why / alternatives
- The overall count mixed all markers (several in view gave more than 100 %), which is what the
  warning was about. Per marker, the rates stay meaningful with a whole page in view, and show which
  corner marker drops out, e.g. under a hand or during fast motion.
- Rejected: keeping the overall line next to the list. With one marker it says the same, with
  several it is the misleading number.
- Rejected: listing every id of the dictionary. Only the markers seen in the interval matter, the
  others would be 0 % noise.

## 2026-10-28: Adaptive threshold window sizes as parameters

### What changed
Two read-only parameters, `adaptive_thresh_win_size_min_px` and `adaptive_thresh_win_size_max_px`
(OpenCV defaults 3 and 23), passed to `FiducialsDetector` and from there to the ArUco detector.
Values OpenCV would reject (min below 3, max below min) raise a `ValueError` when the detector is
created.

### Why / alternatives
- The detector thresholds every frame once per window size (3, 13 and 23 px by default) and
  searches each result for markers. On large camera frames detection was the larger part of the
  node's time per frame, so the node processed fewer frames than the camera delivered. Windows much
  smaller than the markers in the image only produce noise; a single window that suits the marker
  size detected noticeably faster in a benchmark on recorded frames, with the same markers and
  identical corners. The right size depends on how large the markers appear, so it is a per-setup
  parameter, not a new default.
- Rejected: ArUco3 detection (`useAruco3Detection`). It lost markers on the recorded frames and was
  not faster.
- Rejected: a larger `minMarkerPerimeterRate`. No measurable effect.
- Rejected for now: decoding the JPEG at half size (`IMREAD_REDUCED_GRAYSCALE_2`). Faster, but
  coarser corners; full resolution is kept.
- Left as the next step if needed: decoding and detecting in parallel threads. It raises the frame
  rate the node keeps up with, but not the time per frame, and adds threading to the node.

## 2026-10-28: Decoding and detection in parallel threads, tried and reverted

### What changed
The frame callback only decoded the frame, and a detection loop in the main thread took the newest
decoded frame from a one-slot hand-over, so decoding the next frame overlapped detecting the current
one. Reverted; the node processes each frame serially as before.

### Why / alternatives
- It raised the rate of processed frames, but not how soon a frame's markers are published, which
  is what makes projected content follow a moving page. A pipeline's delay per frame is still
  decode + wait + detect: a decoded frame waits for the busy detection stage about as long as a raw
  frame waited for the busy node before. Measured live, the age of `/fiducials` did not improve. The
  threading (a spin thread, a hand-over, error passing between threads) was not worth a higher rate
  alone.
- What would shorten the delay instead: less work per frame, e.g. searching only near the markers'
  last positions and the whole frame only when they are lost.

## 2026-11-04: Detection on the mean of several frames (`temporal_mean_frame_count`)

### What changed
- `fiducials_detector.py`:
  - `FrameMeanBuffer` keeps a running sum of the last N greyscale frames and returns their mean.
  - `merge_fiducials_from_mean_image()` returns the frame's markers plus those only the mean image
    found.
- `fiducials_detector_node.py`: new read-only parameter `temporal_mean_frame_count`, default 0
  (off). With 2 or more, the node also detects on the mean of the last N processed frames, with a
  second detector without debug, and publishes the merged markers. The detector settings are now
  built once and shared by both detectors.

### Why / alternatives
- With a short exposure at high gain (e.g. an IR camera behind a filter, where light is scarce),
  sensor noise makes ArUco miss markers even when they lie still. Averaging frames removes that
  noise, but a moving marker turns into shifted copies in the mean and is lost there. Offline on
  recorded noisy frames, the frame alone and the mean alone each failed in one situation (noise at
  rest, motion), while their merged markers were the most complete at rest, slow and fast.
- *Frame markers win, mean markers are only added*, because the mean lags behind a moving marker. A
  mean marker counts as already found when a frame marker with the same id lies within one marker
  size, so two markers sharing an id in different places (two copies of a page) are both kept.
  Rejected: merging by id alone, which would drop such copies.
- Rejected: spatial denoising (Gaussian, median filter) instead of the mean. It lost more markers
  than the raw frame in the same test, because with few pixels per marker cell, smoothing blurs the
  cell edges more than it removes noise.
- *Running sum* instead of averaging N frames per call: one addition and one subtraction per frame.
- *Off by default*: the second detection roughly doubles the work per frame, which costs frame rate
  on cameras that don't need it.
- Known limit (marked `Simplification:` in the node): the buffer holds the frames the node
  processed, not consecutive camera frames, when detection is slower than the camera. At rest that
  makes no difference to the noise.

## 2026-11-04: Tracking mode (`tracking_window_size_px`, `full_scan_rate_hz`)

### What changed
- `fiducials_detector.py`:
  - `FiducialsDetector.detect_fiducials_in_windows()` detects markers only inside given windows and
    returns them in frame pixels. Its own OpenCV detector gets the full frame's smallest marker
    perimeter in pixels (`_get_window_detector`).
  - `MarkerTracker` keeps each marker's last centroid, velocity and time seen, and returns a search
    window around each marker's predicted position. Markers unseen for a while are dropped.
  - `BackgroundFullScanner` searches whole frames in its own thread at a fixed rate, with its own
    detector, and hands its result to the tracker.
  - `_is_marker_in_list()` is shared by the mean-image merge and the window de-duplication.
- `fiducials_detector_node.py`: new read-only parameters `tracking_window_size_px` (0 = off, the
  default) and `full_scan_rate_hz`. With tracking on, each frame is searched only in the windows;
  frame times come from the camera stamps. `temporal_mean_frame_count` is ignored while tracking is
  on. `destroy_node` stops the scanner thread.

### Why / alternatives
- On a large frame with small markers, a full search is slow enough that projected content
  following a moving page lags visibly. The markers cover a tiny part of the frame; searching only
  near them is many times faster.
- *Full search in a background thread* instead of every Nth frame in the same thread: every Nth
  frame would take as long as a full search, so the published positions would stutter. OpenCV
  releases the GIL, so the thread really runs in parallel. Each thread has its own detector,
  because one detector must not run in two threads at once.
- *Window detector with the minimum marker size in pixels:* OpenCV's `minMarkerPerimeterRate` is
  relative to the image size. In a small window it allows tiny contours, and with a noisy camera
  every speck of noise became a candidate, which made the windows several times slower. Scaling it
  per window keeps the full frame's limit.
- *Velocity prediction:* the window is placed where the marker should be now, from its last two
  sightings, so it can stay small during fast, steady motion. A sudden reversal faster than half a
  window per frame loses the marker until the next full search.
- *Comparing scan results with backdated tracks:* a scan result comes from an older frame. Without
  backdating, a fast marker was added a second time at its old position.
- Rejected: OpenCV's `useAruco3Detection`, tried as the simpler option. With small markers, its
  downscaled search found none, and without downscaling it was not faster.
- Rejected as the only change: cheaper settings (one threshold window). Faster, but not enough.
- Known limits: a new marker appears only after the next full search (up to
  `1 / full_scan_rate_hz`); the full search still uses several cores.

## 2026-11-04: `camera_info` dropped once calibrated

### What changed
`fiducials_detector_node` (subscribed only with depth) destroys its `/camera/camera_info`
subscription as soon as a message with a calibrated camera matrix (`k[0] != 0`) has arrived, and
logs that. Uncalibrated messages keep the subscription.

### Why / alternatives
- `camera_info` comes with every camera frame, so at a high frame rate it costs one Python callback
  per frame, yet the intrinsics do not change while the camera runs. Measured first in `demo_app`,
  where it lowered the node's CPU load; applied to every node that only stores the message.
- Rejected: keeping the subscription with a smaller queue. rclpy still takes and deserialises every
  message.
- Rejected: reading the intrinsics from the calibration file. Some cameras publish factory
  intrinsics that exist only on the topic.
- Rejected for now: a shared helper for the pattern. It would need a ROS-side shared package; the
  pattern is a few lines per node.
- Accepted limit (marked `Simplification:`): intrinsics stored with `set_camera_info` while the
  node runs take effect only after a restart.

## 2026-11-04: Bit-reading parameters and one OpenCV thread in tracking mode

### What changed
- New read-only parameters `bit_sample_px_per_cell` and `bit_ignored_margin_per_cell` (OpenCV's
  `perspectiveRemovePixelPerCell` and `perspectiveRemoveIgnoredMarginPerCell`, defaults unchanged
  at 4 and 0.13), passed to both the full-frame and the window detector, and validated in
  `FiducialsDetector`.
- With `tracking_window_size_px` set, the node calls `cv2.setNumThreads(1)`.

### Why / alternatives
- The tracking mode still used several cores. Measured offline on recorded noisy IR frames of small
  markers:
  - OpenCV's thread pool multiplied the CPU time of the small window searches compared with one
    thread, for little or no gain in wall time. The full scan runs in its own thread, where a
    slower scan adds no latency to the tracked markers.
  - Most of the CPU went to the full scan, which thresholds and searches the frame once per
    threshold window size. A single window size cuts that substantially, but lost markers. Reading
    the bits from more pixels per cell with a wider margin brought them back. Together, a fraction
    of the CPU time at the same detection rate.
- Rejected: turning off OpenCV's busy waiting (`OPENCV_THREAD_POOL_ACTIVE_WAIT_*`). No reduction;
  the cost is the parallel work itself, not idle spinning.
- Rejected: a thread-count parameter. Only tracking mode benefits, and full-frame mode keeps the
  parallel speed-up for its latency.
- Rejected earlier: downscaled full scans or OpenCV's `useAruco3Detection`. Markers a few pixels
  per cell wide are not found at a lower resolution.

## 2026-11-04: Raw frames read from their serialized messages

### What changed
Without `use_compressed` and `use_depth`, the node subscribes to `/camera/image_raw` with
`raw=True`. The new module `serialized_image.py` (no rclpy) reads stamp, `frame_id`, size, encoding
and pixels straight from the serialized CDR bytes; for `mono8` the frame is a NumPy view of the
received buffer, `bgr8` and `rgb8` are converted to grey. Anything else (another encoding, another
CDR representation, sizes that don't match the data) falls back to `deserialize_message` and the
existing conversion. Detection and publishing moved into `_detect_and_publish_fiducials`, shared by
both paths.

### Why / alternatives
- With a large mono camera frame at a high frame rate, turning each frame into a Python message
  took far more CPU than receiving it serialized, so the node's main thread was saturated by
  receiving alone and dropped most frames; their wait in the queue was most of the delay from
  capture to `/fiducials`. Read in place, the node keeps up with the camera, and the delay from the
  frame's stamp to `/fiducials` dropped to a fraction.
- Rejected: a C++ node, or a composable node sharing memory with the driver. It removes the cost
  completely, but rewrites the node in another language and ties it to C++ drivers; not needed
  while the Python path keeps up with the camera.
- Rejected: a parameter to switch it on. The path is cheaper for every camera, and unhandled frames
  fall back by themselves.
- Unchanged: the depth path. `message_filters` synchronises deserialized messages only.
- Verified offline against `cv_bridge` for `mono8`, `bgr8` and `rgb8`, with and without padded
  rows. With padded colour rows, `cv_bridge` itself returned a different image than the rows hold;
  the new reader takes `step` into account.

## 2026-11-08: Registered as a sensor processing node in the ament index

### What changed
`resource/par_framework_sensor_nodes/fiducials_detector` (display name, description, the sensor
nodes it needs) and one `data_files` line in `setup.py` that installs it under the resource type
`par_framework_sensor_nodes`.

### Why / alternatives
- `par.launch.py` and the setup wizard now find the framework's sensor nodes through the ament
  index instead of a hard-coded list (`DETECTORS`), so a new or third-party node appears without a
  change in the core. Format and blueprint: `templates/sensor_processing_node_entry.yaml` in
  ros2-par-framework.
- Rejected: a shared list in `launch_helpers`. Every new node would need a core change.

## 2026-11-10: Relative topic and service names

### What changed
The node subscribes and publishes with relative names (`image_raw`, `camera_info`, `fiducials`,
`projection_mapping`, ...) instead of absolute ones (`/camera/image_raw`, `/projection_mapping`,
...).

### Why / alternatives
- A first step towards setups with several cameras. ROS 2 nodes use relative names so that the
  launch file can place them in a namespace: with several cameras, each node runs in its camera's
  namespace (`/cameras/<name>/`) and finds that camera's topics without remapping. Until then
  `par.launch.py` and the calibration launch files remap the camera streams onto `/camera/...`
  (`launch_helpers.create_camera_stream_remappings`); the other relative names resolve in the root
  namespace exactly as the absolute ones did, so nothing changes from outside. A node started alone
  with `ros2 run` needs `-r image_raw:=/camera/image_raw` (and the other camera streams it reads) or
  `-r __ns:=/camera`.
- Rejected: keeping the absolute names and remapping every topic per node run in the launch files.
  It works, but every node run needs a remap per topic and it is not the ROS 2 convention.
- Rejected: a parameter per topic name. Against the framework rule that topics are chosen by
  remapping, not by parameter.

## 2026-11-17: `fiducials_detector.py` split into modules, smaller helpers

### What changed
- `fiducials_detector.py` keeps the detector (`FiducialsDetector`) and the marker-list helpers
  (`merge_fiducials_from_mean_image`, `_is_marker_in_list`). Moved unchanged into their own modules:
  `marker_depth.py` (`compute_corners_in_camera_space`, `DepthArrivalMonitor` and the depth
  constants), `marker_tracker.py` (`MarkerTracker`, `BackgroundFullScanner`) and
  `frame_mean_buffer.py` (`FrameMeanBuffer`). The node imports from the new modules.
- The marker angle is computed as `atan2` of the top edge directly. The old code measured the angle
  against `(frame height, 0)`; only the direction of that vector mattered, so the result is the same
  (checked against the old formula on random corners). The method no longer needs the frame, and
  the StackOverflow helper is gone. The result stays a truncated `int`, because `Fiducial.angle` is
  an `int32`.
- The centroid is `corners.mean(axis=0)` (a NumPy array instead of a tuple; the node unpacks it the
  same way).
- The marker dict is built in one place (`_create_marker_dict`) instead of twice.
- Both `DetectorParameters` objects (full frame and windows) are built by one function
  (`_create_detector_parameters`) instead of copying attributes by a list of names.
- Small fixes: unused return value as `_`, `aruco.` used consistently, comments that contradicted
  the code.
- `test/test_corners_to_camera_space.py` imports `compute_corners_in_camera_space` from
  `marker_depth`. It still imported the old name `corners_to_camera_space`, so the test failed on
  import.

### Why / alternatives
- The module held five separate jobs in over 600 lines. Each module can now be read on its own. A
  list of attribute names to copy fails silently: a setting added later but missing from the list
  would leave the window detector on OpenCV's default.
- Rejected: `merge_fiducials_from_mean_image` in `frame_mean_buffer.py`. It needs the marker
  matching that the window search also uses, so it would import a private helper across modules.
- Rejected: copying the parameter object. `cv2` objects do not support `copy` reliably.
- Rejected: keeping one file. Five jobs in one module are hard to read.

## 2026-11-17: Node cleanup (`fiducials_detector_node.py`)

### What changed
- `QUEUE_SIZE` split into `FIDUCIALS_PUBLISHER_QUEUE_SIZE` and `FRAME_DEPTH_SYNC_QUEUE_SIZE`;
  `SYNC_LEEWAY_TIME_SEC` renamed to `FRAME_DEPTH_SYNC_SLOP_SEC` ("slop" is the `message_filters`
  term).
- `_on_camera_info` stores `camera_matrix` and `distortion_coefficients` once calibrated intrinsics
  arrive, instead of the whole message; `_add_corners_3d` only checks `camera_matrix is None`.
- The missing-depth warning names the parameter through `PARAMETER_USE_DEPTH`.
- `destroy_node` moved next to `__init__`; `full_scanner` renamed to `background_full_scanner`,
  `header` to `frame_header` in `_create_fiducials_list_msg`.
- Every name for a time in seconds ends in `_sec` / `_SEC`: the `_s` names in `marker_tracker.py`
  and the node (`time_sec`, `last_seen_time_sec`, `velocity_px_per_sec`,
  `TRACKING_MAX_UNSEEN_TIME_SEC`, ...) were renamed.

### Why / alternatives
- One constant for two unrelated queue depths would change both when only one is meant. The
  intrinsics never change after the subscription ends, so reshaping them per frame and checking
  "uncalibrated" in two places was redundant.
- Rejected: `_s` / `_S` everywhere instead of `_sec`. `_sec` reads unambiguously.

## 2026-11-17: One file per class, plain Python constructs

### What changed
- Each module is named after the class it holds: `marker_tracker.py` (`MarkerTracker`),
  `background_full_scanner.py` (`BackgroundFullScanner`), `depth_arrival_monitor.py`
  (`DepthArrivalMonitor`), `frame_mean_buffer.py` (`FrameMeanBuffer`). The function
  `compute_corners_in_camera_space` and its constants are in `corners_in_camera_space.py`, named
  after what it computes. `marker_depth.py` and `marker_tracking.py` are gone.
- No `**dict` unpacking: the detector calls `_create_detector_parameters` with its arguments; the
  node builds every `FiducialsDetector` through `_create_fiducials_detector(debug)`, which reads the
  parameters itself, so `_create_tracking` and `_create_mean_image_detection` no longer take a
  settings dict.
- No `lambda`: the IPPE pose closest to the depth plane is chosen in a plain loop.
- No nested tuple unpacking: the node indexes the corner list (`corners_px[0][0]` ...).
- No `!r` in f-strings, and `dictionary_name or default` became an explicit `if`.

### Why / alternatives
- Python that reads simply, without constructs a reader has to know specifically, and a class that
  can be found by its file name.
- Rejected: wrapping `compute_corners_in_camera_space` in a class so its file is named after a
  class. It would be a class without state.
- Rejected: moving `DepthArrivalMonitor` into the node. Two files keep the plain logic out of the
  node.
- List comprehensions stay; they were not part of this change.

## 2026-11-17: Detection on the mean of several frames removed

### What changed
`FrameMeanBuffer` (`frame_mean_buffer.py`), `merge_fiducials_from_mean_image`, the node's
`_create_mean_image_detection` and `_add_fiducials_from_mean_image`, and the parameter
`temporal_mean_frame_count` are gone. `_is_marker_in_list` stays: the window search uses it to
report a marker seen in two windows once. The entry in the core repo's `par_setup.yaml` is removed
as well.

### Why / alternatives
- No setup used it: the parameter was 0 in `par_setup.yaml` and in every profile. The noisy IR
  setup it was built for got its markers from tuned bit reading (`bit_sample_px_per_cell`,
  `bit_ignored_margin_per_cell`) and tracking instead, which is cheaper, and the node ignored the
  mean while tracking was on, so the two could not be combined. Keeping it cost about 100 lines, a
  parameter to explain, and a second detector per frame when enabled.
- Rejected: keeping it as an option for a future noisier camera. Unused code; it is in git history,
  e.g. commit d104eb2, if it is needed again.

## 2026-11-17: `marker_id_to_debug` removed from `DetectionAccuracyDebugger`

### What changed
The debugger's constructor no longer takes `marker_id_to_debug`; `on_new_marker` counts every id.

### Why / alternatives
- `FiducialsDetector` always created the debugger without arguments, so the filter was always off
  (-1). The output already lists the rate per marker id, so one marker's rate can be read from it
  without a filter.
- Rejected: exposing the filter as a ROS parameter. Nothing needs it, and the per-id output covers
  the use case.

## 2026-11-17: `python3-numpy` declared in `package.xml`

### What changed
`<exec_depend>python3-numpy</exec_depend>`.

### Why / alternatives
- The node and its modules import numpy, but it only arrived indirectly (the OpenCV pip package and
  `cv_bridge` pull it in). rosdep reads only `package.xml`, so an undeclared dependency works by
  luck and fails at import if those packages change. Every other framework package that imports
  numpy declares it the same way.
- Rejected: `numpy` in `requirements.txt`. No framework package does this, and it would install a
  second numpy through pip next to the apt one `cv_bridge` needs.
- Rejected: `<depend>`. An `ament_python` package needs numpy only at runtime.

## 2026-11-17: Dedicated `tracking_mode` parameter

### What changed
- `fiducials_detector_node.py`: new read-only bool parameter `tracking_mode` (default `false`)
  switches tracking on. `tracking_window_size_px` now only sets the window size (default 200) and
  must be positive when tracking is on.

### Why / alternatives
- A window size of 0 meaning "off" hid an on/off switch inside a size, so it was not obvious from
  the parameter list that tracking exists or is off. A bool says it directly.
- Rejected: keeping 0 as "off" in addition. Two ways to switch it off would conflict.

## 2026-11-17: Clearer method names in `fiducials_detector_node.py`

### What changed
- `_create_tracking` → `_init_tracking_mode`,
  `_detect_fiducials_with_tracking` → `_detect_fiducials_in_tracking_mode`,
  `_on_frame_arrived` / `_on_depth_arrived` → `_record_frame_arrival_time` /
  `_record_depth_arrival_time`,
  `_on_frame_and_depth_received` → `_on_frame_and_depth_pair_received`,
  `_on_frame_received` → `_detect_and_publish_fiducials_in_frame_msg`,
  `_on_camera_info` → `_on_camera_info_received`.

### Why / alternatives
- "arrived" and "received" meant different things (recording a time for the depth monitor vs.
  processing), which the names did not show. `_on_…_received` is now used only for subscription
  callbacks, and a method that is also called directly says what it does.
- Rejected: `_process_frame_msg` for the shared frame path. "process" is too generic.

## 2026-11-17: `MarkerTracker` renames and out-of-order frames

### What changed
- Renamed `search_windows_px` → `compute_search_windows_px`, `update_from_windows` →
  `update_from_window_detections`, `add_from_full_scan` → `add_untracked_fiducials`,
  `_add` → `_add_tracked_marker`, `_find_tracked_marker` → `_find_matching_tracked_marker`,
  `_update_or_add` → `_update_or_add_tracked_marker`; attributes are private; parameters are
  `detected_fiducials`.
- A detection older than a track's last sighting no longer updates that track.
- `Simplification:` comment on the duplicate track a stale full scan can add.

### Why / alternatives
- Frames can arrive out of order; updating with an older one moved the track back in time.
- `add_untracked_fiducials` describes what the method does (only adds, never updates) instead
  of where the detections come from; the tracker does not need to know about the full scan.
- Rejected: `add_new_fiducials`. "new" could mean newly detected or untracked.

## 2026-11-17: `tracking/` and `depth/` subpackages

### What changed
- `marker_tracker.py` and `background_full_scanner.py` moved to `fiducials_detector/tracking/`,
  `corners_in_camera_space.py` and `depth_arrival_monitor.py` to `fiducials_detector/depth/`, each
  with an empty `__init__.py` (`find_packages` picks them up). The node's imports follow.

### Why / alternatives
- Each folder matches one parameter (`tracking_mode`, `use_depth`): code that is only active with
  that option is grouped, and someone not using it can skip the folder. The node, the detector
  core and `serialized_image.py` stay at the top, so the entry point is unchanged.
- Rejected: a layer split (`ros/` vs. `core/`). It would move the entry point, and the
  `*_node.py` naming already separates ROS code from plain logic.

## 2026-11-17: Debugger moved to `utils/`

### What changed
- `fiducials_detector_debugger.py` moved to `fiducials_detector/utils/` (with `git mv`, so its
  history follows); `fiducials_detector.py` imports it from there.

### Why / alternatives
- It is a testing aid (accuracy preview), not part of detection, so it sits apart from the core
  modules at the top level.

## 2026-11-17: `serialized_image.py` moved to `utils/`

### What changed
- `serialized_image.py` moved to `fiducials_detector/utils/` (with `git mv`); the node imports it
  from there.

### Why / alternatives
- A helper for reading frames from serialized messages, not detection logic, so it joins the
  other helper in `utils/`.

## 2026-11-17: Debug preview window in its own class

### What changed
- New `utils/detection_preview_window.py` with `DetectionPreviewWindow.show_frame_with_markers()`;
  it holds the preview constants and the "window sized once" state. `FiducialsDetector` creates
  one with `debug` and calls it instead of its former `_show_preview`.

### Why / alternatives
- Drawing and window handling are debug-only and unrelated to detection; separated, the detector
  file holds only detection, next to the accuracy debugger in `utils/`.
- Rejected: merging it into `DetectionAccuracyDebugger`. That class counts rates and prints, the
  window shows images; one class per job keeps both small.

## 2026-11-17: Stay subscribed to `camera_info`

### What changed
- `fiducials_detector_node.py`: the node no longer unsubscribes from `camera_info` after the first
  calibrated message. Every calibrated message updates the camera matrix and distortion; the info
  log is written once, when the first one arrives.

### Why / alternatives
- ROS 2 camera consumers (image_transport's CameraSubscriber, image_proc, depth_image_proc) keep
  reading `camera_info`; unsubscribing missed intrinsics stored with `set_camera_info` while the
  camera runs. The message is small compared with the detection on the same frame.
- Rejected: synchronizing `camera_info` with the frames. Intrinsics rarely change, and a third
  synchronized topic would drop frames whenever one message is late.
- Rejected: re-subscribing on a timer. More code and still a delay.

## 2026-11-17: Tracking attributes default to `None`

### What changed
- `fiducials_detector_node.py`: `marker_tracker` and `background_full_scanner` are class
  attributes defaulting to `None`, next to `camera_matrix`; `_init_tracking_mode` overrides them.

### Why / alternatives
- With the `tracking_mode` check moved into `__init__`, both attributes only existed with tracking
  on; without it the first frame raised `AttributeError`, and so did shutdown in `destroy_node`.
- Setting them to `None` in `__init__` before the check would work as well; class attributes
  match how the node already defaults `camera_matrix`.

## 2026-11-17: 16-bit greyscale frames scaled by their own range

### What changed
- New `utils/mono16_scaling.py` with `scale_mono16_to_mono8()`: scales a 16-bit frame so its
  brightest pixel becomes 255. `_decode_frame_mono` uses it for `mono16` / `16UC1` frames.

### Why / alternatives
- cv_bridge converts 16 to 8 bit by dividing by 256. Infrared cameras (e.g. the Kinect's infrared
  on the colour grid) use only a small part of the 16-bit range, so the detector got an almost
  black image and found no markers. Scaling each frame by its own maximum works at any exposure;
  the adaptive threshold only needs local contrast.
- Rejected: scaling in the camera driver (its fixed `ir_mono8_scaling_factor`). A fixed factor
  clips or darkens when the exposure or illumination changes, and it would also change the raw
  infrared other nodes read.
- Scaling by a high percentile instead of the maximum would resist single bright pixels better,
  but costs a sort per frame; marked as `Simplification:`.

## 2026-11-18: Window detector created on first use

### What changed
- `fiducials_detector.py`: `FiducialsDetector` no longer builds the window detector in `__init__`.
  `_get_window_detector` creates it on its first call from `aruco_dictionary` and
  `aruco_detector_parameters`, then rescales `minMarkerPerimeterRate` through
  `getDetectorParameters` / `setDetectorParameters`. The attribute `window_detector_parameters`
  and its second `_create_detector_parameters` call are gone.

### Why / alternatives
- Each instance uses only one of the two detectors: without tracking, and in the background full
  scanner, only the full-frame detector; in tracking mode the main instance only the window
  detector. Creating the window detector eagerly built an unused object and duplicated the
  parameter setup.
- Building it from `aruco_detector_parameters` is safe because `ArucoDetector` keeps its own copy
  of the parameters and `getDetectorParameters` returns a copy (checked with OpenCV 4.11), so
  rescaling the window detector never changes the full-frame reference value.
- Rejected: a separate parameter object copied with `copy.copy` (OpenCV's `DetectorParameters`
  cannot be copied that way), and keeping a stored `window_detector_parameters` attribute (the
  detector already holds the parameters, a second copy only has to be kept in sync).
- Rejected: also creating the full-frame detector lazily. Its parameters are the reference for
  the window size anyway, and an eager detector keeps `detect_fiducials` free of a `None` check.
## 2026-11-18: Split compute_corners_in_camera_space into steps

### What changed
`fiducials_detector/depth/corners_in_camera_space.py`: `compute_corners_in_camera_space` now
reads as its four steps, each a private function with its own docstring:
`_get_depth_samples_inside_marker` (inner polygon mask, coverage checks),
`_compute_square_pose_candidates` (IPPE poses below the reprojection limit),
`_compute_depth_plane_normal` (SVD of the depth points) and `_select_pose_closest_to_normal`.
The combined `or` condition and the nested `np.maximum`/`np.minimum` bounding-box expression are
split into named single steps. Results are unchanged: old and new code returned identical arrays
(or both `None`) for randomly posed synthetic markers with partly missing depth.

### Why / alternatives
- One 80-line function mixed masking, pose estimation, plane fitting and scaling; each step can
  now be read and tested on its own.
- Rejected: moving the helpers into separate modules. They are only used here, and one module
  per computed result matches the package's layout.
- Rejected: `max(candidate_poses, key=...)` for the pose selection, in favour of an explicit
  list of agreements and `np.argmax`, which avoids a lambda.

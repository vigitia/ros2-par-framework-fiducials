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

## 2026-10-27: optional compressed input (`use_compressed`)

**What changed:** with `use_compressed`, the node subscribes to `/camera/image_raw/compressed`
(`sensor_msgs/CompressedImage`) instead of `/camera/image_raw` and decodes each JPEG with
`cv2.imdecode(..., IMREAD_GRAYSCALE)`. The message type is chosen once and used for both the plain and the
`use_depth` subscription, as in `object_detector`. The check that the depth image is on the colour image's pixel
grid moved from `listener_callback_with_depth` into `listener_callback`, after decoding, because `CompressedImage`
has no width and height. A frame that cannot be decoded is skipped with a throttled warning.

**Why:** a camera driver that passes its JPEGs through then needs no decode for this node, a raw frame is many times
larger than its JPEG to transfer, and decoding straight to greyscale skips the colour channels the detector never
uses. The greyscale from the JPEG's brightness channel differs from cv_bridge's colour-to-mono conversion only by
rounding.

**Alternatives:** a separate callback for compressed frames (rejected: two copies of the detection path; the
`isinstance` branch keeps one, like `object_detector`); rejecting `use_compressed` together with `use_depth`
(rejected: message_filters synchronises `CompressedImage` like `Image`, so the combination costs nothing).

## 2026-10-27: names say what they hold, with units

**What changed:** constants, functions, methods and variables in all three modules are renamed after the naming
rules: units in names (`_px`, `_m`, `_deg`, `_sec`), no single letters or generic words, a verb for functions,
and a leading underscore for methods only used inside their class. Notable ones: `corners_to_camera_space` is
`compute_corners_in_camera_space(corners_px, depth_image_m, camera_matrix, distortion_coefficients)`, the node's
callbacks are `_on_frame_received` / `_on_frame_and_depth_received`, `convert_to_fiducials_interface_msg` is
`_create_fiducials_list_msg`, the `FiducialsDetector` constructor takes `ros_node` and `dictionary_name`,
`FiducialsDetectorDebugger` is `DetectionAccuracyDebugger`, and `DEPTH_MISSING_SEC` is `DEPTH_MISSING_TIMEOUT_SEC`.
Index loops over the detected markers and pose candidates became `zip` loops; the message dictionary keys
(`corners`, `centroid`, ...) and the ROS parameter names are unchanged.

**Why:** the names must be readable on their own, e.g. in a traceback or a grep, and the 3D corner code mixes
pixels and metres.

**Checked:** the committed and the renamed module give identical detections (ids, angles, corners, centroids) on a
rendered image and identical 3D corners for a flat and a tilted square on synthetic depth; the node starts with and
without `use_depth`. Nothing outside this repository used the renamed names.

## 2026-10-27: detector without ROS node, debugger counters per instance

**What changed:**
- `FiducialsDetector` no longer takes a ROS node. `detect_fiducials` only detects; drawing the markers in a window
  moved into the module's `__main__` block, which now takes the image path as argument
  (`python -m fiducials_detector.fiducials_detector <image path>`), because the image it read does not exist.
- `DetectionAccuracyDebugger` sets its counters in `__init__` instead of as class attributes, and its prints start
  with `[DetectionAccuracyDebugger]: `.
- `detect_fiducials` checks `marker_ids is not None` instead of `np.all(marker_ids is not None)`.
- The node no longer logs a greeting and the working directory at startup.

**Why:** the node was passed only so that a missing node could open a window that blocks on `cv2.waitKey(0)`;
detection logic stays free of ROS objects, as the framework's node convention asks. Class attributes are shared by
all instances, so a list in one debugger showed up in every other one, and the interval started at import instead
of at creation. The `np.all` around a single boolean did nothing. The startup lines carried no information.

**Alternatives:** keeping a `show_detections` flag in the constructor (rejected: nothing but the standalone test
uses it, and a window inside a node callback would block it).

## 2026-10-27: node split into one method per step, depth arrival monitor as a class

**What changed:** `__init__` delegates the subscriptions to `_create_subscriptions` (the name the other
nodes use), which leaves the depth pairing to `_create_frame_and_depth_subscriptions`; `_on_frame_received` calls `_decode_frame_mono`, `_get_depth_image_on_frame_grid`,
the detector and `_add_corners_3d` in turn. The three "does depth arrive" timestamps and their condition moved into
`DepthArrivalMonitor` in `fiducials_detector.py` (no ROS, testable with plain pytest); the node only feeds it
times and logs the warning. `_create_fiducials_list_msg` unpacks the corners into keyword arguments and takes a
required header. Timer period, warning throttle and the millimetre factor are named constants; the log messages no
longer repeat the node name (the logger adds it); imports are one sorted block; the commented-out debug line and the
per-block AI markers are gone (one note at the top, one marker per method).

**Why:** both methods did several things inline, and the monitor's state existed only in depth mode, spread over
three callbacks.

**Checked:** the old and the new message creation give equal messages; the monitor gives the old condition's answer
at every step of several frame/depth timelines; a synthetic two-marker frame through the node gives the same
`/fiducials` raw and as JPEG; the node starts in depth mode.

**Alternatives:** keeping the monitor in the node with a docstring (rejected: its state and condition are plain
logic, which the node convention puts outside the node).

## 2026-10-27: debug mode as a ROS parameter, with a preview window

**What changed:** the module constant `DEBUG_DETECTION_ACCURACY` is replaced by the read-only node parameter `debug`,
passed to `FiducialsDetector(debug=...)`. With it, every frame is shown in the OpenCV window `fiducials_detector`
with the detected markers outlined and labelled with their ids, and the detection accuracy is printed as before. The
`__main__` test uses the same preview instead of drawing on its own, and no longer squeezes the image to 800x600.

**Why:** a constant needed a code change to switch; a parameter can be set per run (`debug:=true`) or in
`par_setup.yaml`. The name `debug` and the preview (fixed width of 800 px, resizable window sized once, `waitKey(1)`)
follow `hand_landmarks` and `gesture_detector`, so all nodes switch and look the same. The frame is scaled down before
drawing, so the preview costs as little detection time as possible.

**Alternatives:** a separate debug image topic for RViz or rqt (rejected: the other nodes use a window, and
publishing a second image stream costs more than drawing a small one); drawing the rejected marker candidates too
(left out: not asked for, but useful when tuning `adaptive_thresh_constant`).

## 2026-10-27: detection accuracy printed once per interval

**What changed:** `DetectionAccuracyDebugger.on_new_frame` prints the accuracy of the interval that just ended, once
per `RESET_INTERVAL_SEC`, instead of a running value for every frame. The check and reset happen before the new frame
is counted, so a frame and its markers always land in the same interval.

**Why:** with `debug` now a parameter, the debugger runs live at the camera's frame rate, where one line per frame
floods the terminal.

## 2026-10-27: multiple-markers warning once per interval

**What changed:** `on_new_marker` only collects the ids; `on_new_frame` prints the "multiple markers" warning once per
interval, after the accuracy line and with the number of ids seen.

**Why:** printed for every marker of every frame, the warning flooded the terminal as soon as a page with four markers
was in view, and hid the accuracy line it qualifies (with several markers the accuracy exceeds 100 %).

## 2026-10-27: detection rate per marker instead of the multiple-markers warning

**What changed:** `DetectionAccuracyDebugger` counts detections per marker id and prints, once per interval, one
line with the rate of every marker seen in it, e.g. `Detection rate over 61 frames: id 96: 100 %, id 97: 51 %`, or
`no markers`. The overall accuracy line and the "multiple markers" warning are gone.

**Why:** the overall count mixed all markers (several in view gave more than 100 %), which is what the warning was
about. Per marker, the rates stay meaningful with a whole page in view, and show which corner marker drops out, e.g.
under a hand or during fast motion.

**Alternatives:** keeping the overall line next to the list (rejected: with one marker it says the same, with several
it is the misleading number); listing every id of the dictionary (rejected: only the markers seen in the interval
matter, the others would be 0 % noise).

## 2026-10-28: adaptive threshold window sizes as parameters

**What changed:** two read-only parameters, `adaptive_thresh_win_size_min_px` and `adaptive_thresh_win_size_max_px`
(OpenCV defaults 3 and 23), passed to `FiducialsDetector` and from there to the ArUco detector. Values OpenCV would
reject (min below 3, max below min) raise a `ValueError` when the detector is created.

**Why:** the detector thresholds every frame once per window size (3, 13 and 23 px by default) and searches each
result for markers. On full-HD camera frames detection was the larger part of the node's time per frame, so the node
processed fewer frames than the camera delivered. Windows much smaller than the markers in the image only produce
noise; a single window that suits the marker size detected noticeably faster in a benchmark on recorded frames, with
the same markers and identical corners. The right size depends on how large the markers appear, so it is a per-setup
parameter, not a new default.

**Alternatives:**
- *ArUco3 detection (`useAruco3Detection`)*: lost markers on the recorded frames and was not faster. Rejected.
- *A larger `minMarkerPerimeterRate`*: no measurable effect. Rejected.
- *Decoding the JPEG at half size* (`IMREAD_REDUCED_GRAYSCALE_2`): faster, but coarser corners; rejected for now in
  favour of keeping the full resolution.
- *Decoding and detecting in parallel threads*: raises the frame rate the node keeps up with, but not the time per
  frame, and adds threading to the node. Left as the next step if needed.

## 2026-10-28: decoding and detection in parallel threads, tried and reverted

**What was tried:** the frame callback only decoded the frame, and a detection loop in the main thread took the
newest decoded frame from a one-slot hand-over, so decoding the next frame overlapped detecting the current one.
Reverted; the node processes each frame serially as before.

**Why reverted:** it raised the rate of processed frames, but not how soon a frame's markers are published, which is
what makes projected content follow a moving page. A pipeline's delay per frame is still decode + wait + detect: a
decoded frame waits for the busy detection stage about as long as a raw frame waited for the busy node before.
Measured live, the age of `/fiducials` did not improve. The threading (a spin thread, a hand-over, error passing
between threads) was not worth a higher rate alone.

**What would shorten the delay instead:** less work per frame, e.g. searching only near the markers' last positions
and the whole frame only when they are lost.

## 2026-11-04: detection on the mean of several frames (`temporal_mean_frame_count`)

**What changed:**
- `fiducials_detector.py`:
  - `FrameMeanBuffer` keeps a running sum of the last N greyscale frames and returns their mean.
  - `merge_fiducials_from_mean_image()` returns the frame's markers plus those only the mean image found.
- `fiducials_detector_node.py`: new read-only parameter `temporal_mean_frame_count`, default 0 (off). With 2 or more,
  the node also detects on the mean of the last N processed frames, with a second detector without debug, and
  publishes the merged markers. The detector settings are now built once and shared by both detectors.

**Why:** with a short exposure at high gain (e.g. an IR camera behind a filter, where light is scarce), sensor noise
makes ArUco miss markers even when they lie still. Averaging frames removes that noise, but a moving marker turns
into shifted copies in the mean and is lost there. Offline on recorded noisy frames, the frame alone and the mean
alone each failed in one situation (noise at rest, motion), while their merged markers were the most complete at rest,
slow and fast. The test on the projector light card ("Handle projector light in detection") has the numbers.

**Decisions and alternatives:**
- *Frame markers win, mean markers are only added*, because the mean lags behind a moving marker. A mean marker counts
  as already found when a frame marker with the same id lies within one marker size, so two markers sharing an id in
  different places (two copies of a page) are both kept. Rejected: merging by id alone, which would drop such copies.
- *Spatial denoising* (Gaussian, median filter) instead of the mean: lost more markers than the raw frame in the same
  test, because with few pixels per marker cell, smoothing blurs the cell edges more than it removes noise. Rejected.
- *Running sum* instead of averaging N frames per call: one addition and one subtraction per frame.
- *Off by default*: the second detection roughly doubles the work per frame, which costs frame rate on cameras that
  don't need it.
- Known limit (marked `Simplification:` in the node): the buffer holds the frames the node processed, not consecutive
  camera frames, when detection is slower than the camera. At rest that makes no difference to the noise.

## 2026-11-04: tracking mode (`tracking_window_size_px`, `full_scan_rate_hz`)

**What changed:**
- `fiducials_detector.py`:
  - `FiducialsDetector.detect_fiducials_in_windows()` detects markers only inside given windows and returns them in
    frame pixels. Its own OpenCV detector gets the full frame's smallest marker perimeter in pixels
    (`_get_window_detector`).
  - `MarkerTracker` keeps each marker's last centroid, velocity and time seen, and returns a search window around
    each marker's predicted position. Markers unseen for a while are dropped.
  - `BackgroundFullScanner` searches whole frames in its own thread at a fixed rate, with its own detector, and
    hands its result to the tracker.
  - `_is_marker_in_list()` is shared by the mean-image merge and the window de-duplication.
- `fiducials_detector_node.py`: new read-only parameters `tracking_window_size_px` (0 = off, the default) and
  `full_scan_rate_hz`. With tracking on, each frame is searched only in the windows; frame times come from the
  camera stamps. `temporal_mean_frame_count` is ignored while tracking is on. `destroy_node` stops the scanner
  thread.

**Why:** on a large frame (1920x1200) with small markers, a full search takes tens of milliseconds, so projected
content following a moving page lagged visibly. The markers cover a tiny part of the frame; searching only near
them is over ten times faster.

**Decisions and alternatives:**
- *Full search in a background thread* instead of every Nth frame in the same thread: every Nth frame would take as
  long as a full search, so the published positions would stutter. OpenCV releases the GIL, so the thread really
  runs in parallel. Each thread has its own detector, because one detector must not run in two threads at once.
- *Window detector with the minimum marker size in pixels:* OpenCV's `minMarkerPerimeterRate` is relative to the
  image size. In a small window it allows tiny contours, and with a noisy camera every speck of noise became a
  candidate, which made the windows several times slower. Scaling it per window keeps the full frame's limit.
- *Velocity prediction:* the window is placed where the marker should be now, from its last two sightings, so it
  can stay small during fast, steady motion. A sudden reversal faster than half a window per frame loses the marker
  until the next full search.
- *Comparing scan results with backdated tracks:* a scan result comes from an older frame. Without backdating, a
  fast marker was added a second time at its old position.
- *OpenCV's `useAruco3Detection`* was tried as the simpler option: with the marker size here, its downscaled search
  found no markers, and without downscaling it was not faster. Rejected.
- *Only the cheaper settings (one threshold window)*: about twice as fast, not enough. Rejected as the only change.
- Known limits: a new marker appears only after the next full search (up to `1 / full_scan_rate_hz`); the full
  search still uses several cores.

## 2026-11-04: `camera_info` dropped once calibrated

**What changed:** `fiducials_detector_node` (subscribed only with depth) destroys its `/camera/camera_info` subscription as soon as a message with a calibrated
camera matrix (`k[0] != 0`) has arrived, and logs that. Uncalibrated messages keep the subscription.

**Why:** `camera_info` comes with every camera frame, so at a high frame rate it costs one Python callback per frame,
yet the intrinsics do not change while the camera runs. Measured first in `demo_app`, where it lowered the node's
CPU load; applied to every node that only stores the message.

**Alternatives:**
- *Keep the subscription with a smaller queue*: rejected, rclpy still takes and deserialises every message.
- *Read the intrinsics from the calibration file*: rejected, some cameras publish factory intrinsics that exist only
  on the topic.
- *A shared helper for the pattern*: rejected for now, it would need a ROS-side shared package; the pattern is a few
  lines per node.
- Accepted limit (marked `Simplification:`): intrinsics stored with `set_camera_info` while the node runs take effect
  only after a restart.

## 2026-11-04: bit-reading parameters and one OpenCV thread in tracking mode

**What changed:**
- New read-only parameters `bit_sample_px_per_cell` and `bit_ignored_margin_per_cell` (OpenCV's
  `perspectiveRemovePixelPerCell` and `perspectiveRemoveIgnoredMarginPerCell`, defaults unchanged at 4 and 0.13),
  passed to both the full-frame and the window detector, and validated in `FiducialsDetector`.
- With `tracking_window_size_px` set, the node calls `cv2.setNumThreads(1)`.

**Why:** The tracking mode still used several cores. Measured offline on recorded noisy IR frames of small markers:
- OpenCV's thread pool roughly doubled the CPU time of the small window searches compared with one thread, for
  little or no gain in wall time. The full scan runs in its own thread, where a slower scan adds no latency to
  the tracked markers.
- Most of the CPU went to the full scan, which thresholds and searches the frame once per threshold window size.
  A single window size cuts that to about a third, but lost markers. Reading the bits from more pixels per cell
  with a wider margin brought them back (also the result of the earlier offline comparison with learned
  detectors). Together, about a sixth of the CPU time at the same detection rate.

**Alternatives:**
- *Turning off OpenCV's busy waiting* (`OPENCV_THREAD_POOL_ACTIVE_WAIT_*`): measured, no reduction; the cost is
  the parallel work itself, not idle spinning.
- *A thread-count parameter*: rejected, only tracking mode benefits, and full-frame mode keeps the parallel speed-up
  for its latency.
- *Downscaled full scans or OpenCV's `useAruco3Detection`*: rejected earlier, markers a few pixels per cell wide are
  not found at a lower resolution.

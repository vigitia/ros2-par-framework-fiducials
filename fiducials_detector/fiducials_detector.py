# Choice between ArUco markers and AprilTags, the dictionary and detector settings (including the
# adaptive threshold window sizes) as constructor arguments added with help of Claude Code.
# Claude Code also renamed constants, functions and variables so their names say what they hold,
# with units, and added the mean image of several frames (FrameMeanBuffer) whose markers are
# merged with the frame's, for noisy frames of still markers, and the tracking mode that searches
# only windows around known markers (MarkerTracker, detect_fiducials_in_windows) while whole
# frames are searched in a background thread (BackgroundFullScanner).

import collections
import math
import threading
import time

import cv2
import cv2.aruco as aruco
import numpy as np

from .fiducials_detector_debugger import DetectionAccuracyDebugger

# Prefix of the cv2.aruco dictionary constants per marker type, e.g. DICT_4X4_100 or
# DICT_APRILTAG_36h11
MARKER_DICTIONARY_PREFIXES = {
    'aruco': 'DICT_',
    'apriltag': 'DICT_APRILTAG_',
}
# Used when no dictionary is given, so switching the marker type alone gives a working detector
DEFAULT_MARKER_DICTIONARIES = {
    'aruco': '4X4_100',
    'apriltag': '36h11',
}
# OpenCV rejects adaptive threshold windows smaller than this
MIN_ADAPTIVE_THRESH_WIN_SIZE_PX = 3
CORNER_REFINEMENT_METHODS = {
    'none': aruco.CORNER_REFINE_NONE,
    'subpix': aruco.CORNER_REFINE_SUBPIX,
    'contour': aruco.CORNER_REFINE_CONTOUR,
    'apriltag': aruco.CORNER_REFINE_APRILTAG,
}

# Debug preview: the frame scaled to this width for every camera (up or down), also the window's
# starting width; the window can be resized further by dragging
PREVIEW_WIDTH_PX = 800
PREVIEW_WINDOW_NAME = 'fiducials_detector'

# Following lines until compute_corners_in_camera_space written by Claude Code
# The depth of a marker is taken from its inner part only: the polygon shrunk towards its centre to
# this fraction. Pixels at the edge may already see what lies around the marker.
DEPTH_SAMPLE_INNER_FRACTION = 0.7
# Fewer valid depth pixels than this in the inner part: no 3D position, e.g. a marker far away
MIN_DEPTH_PIXEL_COUNT = 20
# Less of the inner part with depth than this: no 3D position. Seen live on a cube's side face that
# the depth camera saw almost edge-on: 13 % valid pixels, and those were the table behind it.
MIN_DEPTH_COVERAGE_FRACTION = 0.5
# A square of edge 1 in solvePnP's IPPE_SQUARE order, which is also OpenCV's marker corner order
# (clockwise from the top left). The real size comes from the depth, so no marker size is needed.
UNIT_SQUARE_CORNERS = np.array([[-0.5, 0.5, 0.0], [0.5, 0.5, 0.0], [0.5, -0.5, 0.0],
                                [-0.5, -0.5, 0.0]])
# A pose that misses the corners by more than this is discarded: they are not the image of a square
# (e.g. a wrong detection). Also IPPE's one degenerate case, a noise-free square facing the camera
# exactly, which only synthetic data produces.
MAX_REPROJECTION_ERROR_PX = 3.0


def compute_corners_in_camera_space(corners_px, depth_image_m, camera_matrix,
                                    distortion_coefficients):
    """
    Return the four corners of a marker in the camera's optical frame, in metres, or None.

    corners_px: 4x2 pixel positions in the raw colour image. depth_image_m: depth image
    registered to that image (the same pixel grid), in metres, 0 where there is no depth.
    camera_matrix, distortion_coefficients: intrinsics of the colour image.

    The orientation comes from the marker's square in the colour image (IPPE), the distance from
    the depth inside it. The depth alone gave steep faces a wrong tilt: on a cube's side face 10
    degrees too steep, a 60 x 77 mm "square" for a 57 mm marker, probably because the
    time-of-flight light also reaches the face via the table in the corner between them
    (multipath). The result is a square that lies on the corners' rays. Returns a 4x3 array; None
    when too little of the marker has depth, or the corners are no view of a square.
    """
    corners_px = np.asarray(corners_px, np.float64).reshape(4, 2)
    center_px = corners_px.mean(axis=0)
    inner_corners_px = center_px + DEPTH_SAMPLE_INNER_FRACTION * (corners_px - center_px)
    # Mask only the bounding box, not the whole image, for every marker in every frame
    depth_height_px, depth_width_px = depth_image_m.shape[:2]
    box_left_px, box_top_px = np.maximum(np.floor(inner_corners_px.min(axis=0)).astype(int), 0)
    box_right_px, box_bottom_px = np.minimum(np.ceil(inner_corners_px.max(axis=0)).astype(int) + 1,
                                             (depth_width_px, depth_height_px))
    if box_left_px >= box_right_px or box_top_px >= box_bottom_px:
        return None
    depth_box_m = depth_image_m[box_top_px:box_bottom_px, box_left_px:box_right_px]
    inner_mask = np.zeros(depth_box_m.shape, np.uint8)
    inner_corners_in_box_px = np.round(inner_corners_px - (box_left_px, box_top_px))
    cv2.fillConvexPoly(inner_mask, inner_corners_in_box_px.astype(np.int32), 1)
    valid_rows_px, valid_columns_px = np.nonzero((inner_mask > 0) & (depth_box_m > 0))
    valid_pixel_count = valid_columns_px.size
    if (valid_pixel_count < MIN_DEPTH_PIXEL_COUNT
            or valid_pixel_count < MIN_DEPTH_COVERAGE_FRACTION * np.count_nonzero(inner_mask)):
        return None

    # A square seen at an angle is ambiguous: IPPE returns both tilts that fit the image
    pose_found, rotation_vectors, translation_vectors, reprojection_errors_px = (
        cv2.solvePnPGeneric(UNIT_SQUARE_CORNERS, corners_px, camera_matrix,
                            distortion_coefficients, flags=cv2.SOLVEPNP_IPPE_SQUARE))
    candidate_poses = []
    for rotation_vector, translation_vector, reprojection_error_px in zip(
            rotation_vectors, translation_vectors, reprojection_errors_px):
        if reprojection_error_px.ravel()[0] <= MAX_REPROJECTION_ERROR_PX:
            rotation_matrix = cv2.Rodrigues(rotation_vector)[0]
            candidate_poses.append((rotation_matrix, translation_vector.ravel()))
    if not pose_found or not candidate_poses:
        return None
    # The depth's tilt is not exact, but good enough to tell the two apart: its plane's normal is
    # the direction of least spread of the depth points
    depths_m = depth_box_m[valid_rows_px, valid_columns_px].astype(np.float64)
    valid_pixels_px = np.column_stack([valid_columns_px + box_left_px,
                                       valid_rows_px + box_top_px]).astype(np.float64)
    normalized_points = cv2.undistortPoints(valid_pixels_px.reshape(-1, 1, 2), camera_matrix,
                                            distortion_coefficients).reshape(-1, 2)
    depth_points_m = np.column_stack([normalized_points * depths_m[:, None], depths_m])
    # Only Vt (3x3) is needed. The full SVD would also build U, N x N for the N depth pixels: for a
    # marker that is large in the image, gigabytes and seconds per marker
    centred_depth_points_m = depth_points_m - depth_points_m.mean(axis=0)
    depth_plane_normal = np.linalg.svd(centred_depth_points_m, full_matrices=False)[2][2]
    # The square lies in its own xy plane, so the rotation's third column is its normal
    square_rotation, square_translation = max(
        candidate_poses, key=lambda pose: abs(pose[0][:, 2] @ depth_plane_normal))
    unit_square_in_camera = UNIT_SQUARE_CORNERS @ square_rotation.T + square_translation
    # Scaled about the camera, every corner stays on its ray. The median depth of the inner part
    # is the depth of the marker's centre; a finger on part of the marker does not move it.
    metric_scale = np.median(depths_m) / unit_square_in_camera.mean(axis=0)[2]
    if not (np.isfinite(metric_scale) and metric_scale > 0):
        return None
    return unit_square_in_camera * metric_scale


class DepthArrivalMonitor:
    """
    Tells whether colour frames arrive but depth frames don't, e.g. without a depth camera.

    Frames are only processed in colour + depth pairs, so missing depth means nothing is
    published at all. Times are in seconds from any monotonic clock.
    """

    # Written by Claude Code
    def __init__(self, timeout_sec):
        self.timeout_sec = timeout_sec
        self.first_frame_time_sec = None
        self.last_frame_time_sec = None
        self.last_depth_time_sec = None

    # Written by Claude Code
    def on_frame(self, now_sec):
        self.last_frame_time_sec = now_sec
        if self.first_frame_time_sec is None:
            self.first_frame_time_sec = now_sec

    # Written by Claude Code
    def on_depth(self, now_sec):
        self.last_depth_time_sec = now_sec

    # Written by Claude Code
    def is_depth_missing(self, now_sec):
        """Return True while colour frames arrive and depth frames have stopped or never came."""
        if self.last_frame_time_sec is None:
            return False
        frames_arrive = now_sec - self.last_frame_time_sec < self.timeout_sec
        # Not before timeout_sec of colour frames: a driver may only start computing registered
        # depth once it sees a subscriber
        frames_arrived_long_enough = now_sec - self.first_frame_time_sec > self.timeout_sec
        if not (frames_arrive and frames_arrived_long_enough):
            return False
        if self.last_depth_time_sec is None:
            return True
        return now_sec - self.last_depth_time_sec > self.timeout_sec


# Written by Claude Code
class FrameMeanBuffer:
    """
    Mean image of the last frame_count frames, for marker detection with less sensor noise.

    Keeps a running sum, so each new frame costs one addition and one subtraction instead of
    averaging all frames again. Only useful while the markers lie still: a moving marker appears
    as shifted copies in the mean and is lost there.
    """

    def __init__(self, frame_count):
        if frame_count < 2:
            raise ValueError(f'frame_count must be at least 2, got {frame_count}')
        self.frame_count = frame_count
        self.frames_mono = collections.deque()
        self.frame_sum = None

    def add_frame_and_get_mean(self, frame_mono):
        """
        Add a greyscale frame; return the mean of the last frame_count frames, or None.

        None until frame_count frames have arrived, also after the frame size changed.
        """
        if self.frame_sum is None or self.frame_sum.shape != frame_mono.shape:
            self.frames_mono.clear()
            # uint32 holds the sum of many uint8 frames without overflow
            self.frame_sum = np.zeros(frame_mono.shape, np.uint32)
        self.frames_mono.append(frame_mono)
        self.frame_sum += frame_mono
        if len(self.frames_mono) > self.frame_count:
            oldest_frame_mono = self.frames_mono.popleft()
            self.frame_sum -= oldest_frame_mono
        if len(self.frames_mono) < self.frame_count:
            return None
        # Integer division with rounding to the nearest grey value
        rounded_sum = self.frame_sum + self.frame_count // 2
        mean_image_mono = rounded_sum // self.frame_count
        return mean_image_mono.astype(np.uint8)


# Written by Claude Code
def merge_fiducials_from_mean_image(frame_fiducials, mean_image_fiducials):
    """
    Return the frame's markers plus those that only the mean image found.

    The frame's markers always win: the mean lags behind a moving marker. A marker from the mean
    image counts as already found when the frame has a marker with the same id whose centroid
    lies within that marker's size, so two markers with the same id in different places (e.g. two
    copies of a page) are both kept.
    """
    merged_fiducials = list(frame_fiducials)
    for mean_image_marker in mean_image_fiducials:
        if not _is_marker_in_list(mean_image_marker, frame_fiducials):
            merged_fiducials.append(mean_image_marker)
    return merged_fiducials


# Written by Claude Code
def _is_marker_in_list(candidate_marker, markers):
    """Return True if markers holds one with the same id whose centroid lies within its size."""
    for marker in markers:
        if marker['id'] != candidate_marker['id']:
            continue
        centroid_distance_px = np.linalg.norm(np.subtract(marker['centroid'],
                                                          candidate_marker['centroid']))
        if centroid_distance_px < _compute_marker_size_px(marker['corners']):
            return True
    return False


# Written by Claude Code
def _compute_marker_size_px(corners_px):
    """Length of the longest edge of a marker's quadrilateral."""
    corners_px = np.asarray(corners_px, np.float64).reshape(4, 2)
    next_corners_px = np.roll(corners_px, -1, axis=0)
    edge_lengths_px = np.linalg.norm(next_corners_px - corners_px, axis=1)
    return float(edge_lengths_px.max())


# Written by Claude Code
class MarkerTracker:
    """
    Remembers where markers were and where to search for them in the next frame.

    Each tracked marker keeps its last centroid, its velocity from the last two sightings, and
    when it was last seen. search_windows_px() returns a square window around each marker's
    predicted position, so a detector only has to look there instead of in the whole frame.
    Markers that new full-frame searches find are added; markers not seen for max_unseen_time_s
    are dropped.
    """

    def __init__(self, search_window_size_px, max_unseen_time_s):
        self.search_window_size_px = search_window_size_px
        self.max_unseen_time_s = max_unseen_time_s
        # One dict per tracked marker: id, centroid_px, velocity_px_per_s, last_seen_time_s
        self.tracked_markers = []

    def search_windows_px(self, time_s, frame_width_px, frame_height_px):
        """Return a (left, top, right, bottom) window per tracked marker, clipped to the frame."""
        half_window_size_px = self.search_window_size_px // 2
        search_windows_px = []
        for tracked_marker in self.tracked_markers:
            elapsed_time_s = time_s - tracked_marker['last_seen_time_s']
            predicted_centroid_px = (tracked_marker['centroid_px']
                                     + tracked_marker['velocity_px_per_s'] * elapsed_time_s)
            center_x_px = int(round(predicted_centroid_px[0]))
            center_y_px = int(round(predicted_centroid_px[1]))
            left_px = max(center_x_px - half_window_size_px, 0)
            top_px = max(center_y_px - half_window_size_px, 0)
            right_px = min(center_x_px + half_window_size_px, frame_width_px)
            bottom_px = min(center_y_px + half_window_size_px, frame_height_px)
            if right_px > left_px and bottom_px > top_px:
                search_windows_px.append((left_px, top_px, right_px, bottom_px))
        return search_windows_px

    def update_from_windows(self, fiducials, time_s):
        """Update the tracks with the markers found in this frame's windows, drop stale ones."""
        for fiducial in fiducials:
            self._update_or_add(fiducial, time_s)
        self._drop_unseen_markers(time_s)

    def add_from_full_scan(self, fiducials, scan_time_s):
        """
        Add markers that a full-frame search found and that are not tracked yet.

        The scan ran on an older frame, and a marker may have moved on since. A tracked marker
        with the same id that was close to the scanned position at the scan's frame time
        (backdated with its velocity) is the same marker and is left alone: its track is more
        recent.
        """
        for fiducial in fiducials:
            if self._find_tracked_marker(fiducial, scan_time_s) is None:
                self._add(fiducial, scan_time_s)

    def _update_or_add(self, fiducial, time_s):
        tracked_marker = self._find_tracked_marker(fiducial, time_s)
        if tracked_marker is None:
            self._add(fiducial, time_s)
            return
        centroid_px = np.asarray(fiducial['centroid'], np.float64)
        elapsed_time_s = time_s - tracked_marker['last_seen_time_s']
        if elapsed_time_s > 0.0:
            displacement_px = centroid_px - tracked_marker['centroid_px']
            tracked_marker['velocity_px_per_s'] = displacement_px / elapsed_time_s
        tracked_marker['centroid_px'] = centroid_px
        tracked_marker['last_seen_time_s'] = time_s

    def _find_tracked_marker(self, fiducial, time_s):
        """Return the tracked marker with the same id within half a window at time_s, or None."""
        centroid_px = np.asarray(fiducial['centroid'], np.float64)
        max_distance_px = self.search_window_size_px / 2
        for tracked_marker in self.tracked_markers:
            if tracked_marker['id'] != fiducial['id']:
                continue
            elapsed_time_s = time_s - tracked_marker['last_seen_time_s']
            tracked_centroid_px = (tracked_marker['centroid_px']
                                   + tracked_marker['velocity_px_per_s'] * elapsed_time_s)
            distance_px = np.linalg.norm(centroid_px - tracked_centroid_px)
            if distance_px < max_distance_px:
                return tracked_marker
        return None

    def _add(self, fiducial, time_s):
        self.tracked_markers.append({
            'id': fiducial['id'],
            'centroid_px': np.asarray(fiducial['centroid'], np.float64),
            'velocity_px_per_s': np.zeros(2),
            'last_seen_time_s': time_s,
        })

    def _drop_unseen_markers(self, time_s):
        still_tracked_markers = []
        for tracked_marker in self.tracked_markers:
            unseen_time_s = time_s - tracked_marker['last_seen_time_s']
            if unseen_time_s <= self.max_unseen_time_s:
                still_tracked_markers.append(tracked_marker)
        self.tracked_markers = still_tracked_markers


# Written by Claude Code
class BackgroundFullScanner:
    """
    Searches whole frames for markers in its own thread, at most scan_rate_hz times a second.

    It finds markers that tracking can't: a page just put down, or a marker that was covered and
    lost. submit_frame() hands over the newest frame (only a reference, no copy), and
    get_new_result() returns the markers of the last finished scan once. Uses its own detector,
    because one OpenCV detector must not run in two threads at once; OpenCV releases the GIL
    during detection, so the scan really runs alongside the tracking.
    """

    def __init__(self, fiducials_detector, scan_rate_hz):
        self.fiducials_detector = fiducials_detector
        self.scan_period_s = 1.0 / scan_rate_hz
        self.lock = threading.Lock()
        self.newest_frame_mono = None
        self.newest_frame_time_s = None
        self.new_result = None
        self.stop_event = threading.Event()
        self.scan_thread = threading.Thread(target=self._run_scans, daemon=True)
        self.scan_thread.start()

    def submit_frame(self, frame_mono, time_s):
        with self.lock:
            self.newest_frame_mono = frame_mono
            self.newest_frame_time_s = time_s

    def get_new_result(self):
        """Return (fiducials, frame time) of the last scan not returned yet, or None."""
        with self.lock:
            result = self.new_result
            self.new_result = None
        return result

    def stop(self):
        self.stop_event.set()
        self.scan_thread.join()

    def _run_scans(self):
        while not self.stop_event.is_set():
            scan_start_s = time.monotonic()
            with self.lock:
                frame_mono = self.newest_frame_mono
                frame_time_s = self.newest_frame_time_s
                self.newest_frame_mono = None
            if frame_mono is not None:
                fiducials = self.fiducials_detector.detect_fiducials(frame_mono)
                with self.lock:
                    self.new_result = (fiducials, frame_time_s)
            # A scan slower than the period starts the next one right away
            remaining_time_s = self.scan_period_s - (time.monotonic() - scan_start_s)
            self.stop_event.wait(max(remaining_time_s, 0.0))


# Currently supporting Aruco Markers and Apriltags
class FiducialsDetector:

    # Co-authored by Claude Code
    def __init__(self, marker_type='aruco', dictionary_name='', corner_refinement='none',
                 adaptive_thresh_constant=7.0, adaptive_thresh_win_size_min_px=3,
                 adaptive_thresh_win_size_max_px=23, debug=False):
        self.debug = debug
        # Whether the preview window got its starting size
        self.preview_window_sized = False

        if marker_type not in MARKER_DICTIONARY_PREFIXES:
            raise ValueError(f'marker_type must be one of {list(MARKER_DICTIONARY_PREFIXES)}, '
                             f'got {marker_type!r}')
        if corner_refinement not in CORNER_REFINEMENT_METHODS:
            raise ValueError(f'corner_refinement must be one of '
                             f'{list(CORNER_REFINEMENT_METHODS)}, got {corner_refinement!r}')
        # OpenCV would only fail on these inside the first detectMarkers call, with an assertion
        if adaptive_thresh_win_size_min_px < MIN_ADAPTIVE_THRESH_WIN_SIZE_PX:
            raise ValueError(f'adaptive_thresh_win_size_min_px must be at least '
                             f'{MIN_ADAPTIVE_THRESH_WIN_SIZE_PX}, got '
                             f'{adaptive_thresh_win_size_min_px}')
        if adaptive_thresh_win_size_max_px < adaptive_thresh_win_size_min_px:
            raise ValueError(f'adaptive_thresh_win_size_max_px '
                             f'({adaptive_thresh_win_size_max_px}) must not be smaller than '
                             f'adaptive_thresh_win_size_min_px '
                             f'({adaptive_thresh_win_size_min_px})')

        dictionary_prefix = MARKER_DICTIONARY_PREFIXES[marker_type]
        dictionary_name = dictionary_name or DEFAULT_MARKER_DICTIONARIES[marker_type]
        dictionary_id = getattr(aruco, dictionary_prefix + dictionary_name, None)
        if dictionary_id is None:
            valid_dictionary_names = [name[len(dictionary_prefix):] for name in dir(aruco)
                                      if name.startswith(dictionary_prefix)]
            raise ValueError(f'Unknown {marker_type} dictionary {dictionary_name!r}, valid are '
                             f'{valid_dictionary_names}')

        self.aruco_dictionary = aruco.getPredefinedDictionary(dictionary_id)
        self.aruco_detector_parameters = aruco.DetectorParameters()
        detector_parameters = self.aruco_detector_parameters
        detector_parameters.cornerRefinementMethod = CORNER_REFINEMENT_METHODS[corner_refinement]
        detector_parameters.adaptiveThreshConstant = adaptive_thresh_constant
        # The image is thresholded once per window size from min to max (step 10 px) and searched
        # for markers each time: fewer windows are faster, and windows much smaller than the
        # markers in the image only add noise
        detector_parameters.adaptiveThreshWinSizeMin = adaptive_thresh_win_size_min_px
        detector_parameters.adaptiveThreshWinSizeMax = adaptive_thresh_win_size_max_px

        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dictionary,
                                                      self.aruco_detector_parameters)
        # For detect_fiducials_in_windows: same settings, but its minimum marker size is rescaled
        # per window, see _get_window_detector
        self.window_detector_parameters = aruco.DetectorParameters()
        for parameter_name in ('cornerRefinementMethod', 'adaptiveThreshConstant',
                               'adaptiveThreshWinSizeMin', 'adaptiveThreshWinSizeMax'):
            setattr(self.window_detector_parameters, parameter_name,
                    getattr(detector_parameters, parameter_name))
        self.window_detector = cv2.aruco.ArucoDetector(self.aruco_dictionary,
                                                       self.window_detector_parameters)
        self.window_detector_size_px = None

        if self.debug:
            self.detection_accuracy_debugger = DetectionAccuracyDebugger()

    # Co-authored by Claude Code: names, debug preview
    def detect_fiducials(self, camera_frame) -> list:
        if self.debug:
            self.detection_accuracy_debugger.on_new_frame()

        # Convert RGB frames to greyscale
        if len(camera_frame.shape) == 3:
            camera_frame = cv2.cvtColor(camera_frame, cv2.COLOR_BGR2GRAY)

        marker_corners_px, marker_ids, rejected_candidates_px = (
            self.aruco_detector.detectMarkers(camera_frame))

        fiducials = []

        # detectMarkers returns None instead of an empty array when it finds no marker
        if marker_ids is not None:
            for corners_px, marker_id in zip(marker_corners_px, marker_ids):
                detected_marker = {
                    'id': marker_id[0],
                    'angle': self._compute_marker_angle_deg(corners_px[0], camera_frame),
                    'corners': corners_px[0],
                    'centroid': self._compute_centroid_px(corners_px[0])}
                fiducials.append(detected_marker)
                if self.debug:
                    self.detection_accuracy_debugger.on_new_marker(detected_marker['id'])

        if self.debug:
            self._show_preview(camera_frame, marker_corners_px, marker_ids)

        return fiducials

    # Written by Claude Code
    def detect_fiducials_in_windows(self, camera_frame_mono, search_windows_px) -> list:
        """
        Detect markers only inside the given windows of a greyscale frame.

        search_windows_px: (left, top, right, bottom) boxes in frame pixels, already clipped to
        the frame. Returns the same marker dicts as detect_fiducials, in frame pixels. A marker
        seen in two overlapping windows is returned once. No debug preview: it would show only
        the windows.
        """
        fiducials = []
        frame_size_px = max(camera_frame_mono.shape[:2])
        for left_px, top_px, right_px, bottom_px in search_windows_px:
            window_image = camera_frame_mono[top_px:bottom_px, left_px:right_px]
            window_size_px = max(right_px - left_px, bottom_px - top_px)
            window_detector = self._get_window_detector(window_size_px, frame_size_px)
            marker_corners_px, marker_ids, _ = window_detector.detectMarkers(window_image)
            # detectMarkers returns None instead of an empty array when it finds no marker
            if marker_ids is None:
                continue
            window_origin_px = np.array([left_px, top_px], np.float32)
            for corners_px, marker_id in zip(marker_corners_px, marker_ids):
                frame_corners_px = corners_px[0] + window_origin_px
                detected_marker = {
                    'id': marker_id[0],
                    'angle': self._compute_marker_angle_deg(frame_corners_px, camera_frame_mono),
                    'corners': frame_corners_px,
                    'centroid': self._compute_centroid_px(frame_corners_px)}
                if not _is_marker_in_list(detected_marker, fiducials):
                    fiducials.append(detected_marker)
        return fiducials

    # Written by Claude Code
    def _get_window_detector(self, window_size_px, frame_size_px):
        """
        Return the window detector, with the full frame's smallest marker size in pixels.

        OpenCV sets the smallest marker perimeter relative to the image size. In a small window
        that relative value allows tiny contours, and every speck of sensor noise is then checked
        as a marker candidate, which makes a noisy window several times slower.
        """
        if window_size_px != self.window_detector_size_px:
            min_perimeter_px = (self.aruco_detector_parameters.minMarkerPerimeterRate
                                * frame_size_px)
            self.window_detector_parameters.minMarkerPerimeterRate = (min_perimeter_px
                                                                      / window_size_px)
            self.window_detector.setDetectorParameters(self.window_detector_parameters)
            self.window_detector_size_px = window_size_px
        return self.window_detector

    # Written by Claude Code
    def _show_preview(self, camera_frame_mono, marker_corners_px, marker_ids):
        """Show the frame with the detected markers outlined and labelled with their ids."""
        # Scaled down before drawing: drawing on and showing a full camera frame costs detection
        # time; the same preview width also keeps it readable at every camera resolution
        frame_height_px, frame_width_px = camera_frame_mono.shape[:2]
        preview_scale = PREVIEW_WIDTH_PX / frame_width_px
        preview_height_px = round(frame_height_px * preview_scale)
        preview_image_mono = cv2.resize(camera_frame_mono, (PREVIEW_WIDTH_PX, preview_height_px))
        # In colour, so the marker outlines stand out
        preview_image = cv2.cvtColor(preview_image_mono, cv2.COLOR_GRAY2BGR)
        if marker_ids is not None:
            preview_corners_px = []
            for corners_px in marker_corners_px:
                scaled_corners_px = corners_px * preview_scale
                preview_corners_px.append(scaled_corners_px.astype(np.float32))
            cv2.aruco.drawDetectedMarkers(preview_image, preview_corners_px, marker_ids)

        # Resizable by dragging, keeping the aspect ratio; no Qt toolbar and status bar, which
        # would take part of the window size below. Does nothing once the window exists.
        cv2.namedWindow(PREVIEW_WINDOW_NAME,
                        cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_NORMAL)
        # Qt opens such a window small, whatever the image size: set it to the image's, once, so
        # later frames don't undo a size dragged by hand
        if not self.preview_window_sized:
            cv2.resizeWindow(PREVIEW_WINDOW_NAME, PREVIEW_WIDTH_PX, preview_height_px)
            self.preview_window_sized = True
        cv2.imshow(PREVIEW_WINDOW_NAME, preview_image)
        # Lets the window draw itself; returns after 1 ms instead of waiting for a key
        cv2.waitKey(1)

    # Calculate the rotation_diff of an aruco marker relative to the frame.
    # Returns the angle in the range from 0° to 360°
    # Co-authored by Claude Code: names
    def _compute_marker_angle_deg(self, marker_corners_px, camera_frame):
        first_corner_px = marker_corners_px[0]
        second_corner_px = marker_corners_px[1]
        frame_x_axis = np.array([camera_frame.shape[0], 0])
        marker_top_edge_px = np.array([second_corner_px[0] - first_corner_px[0],
                                       second_corner_px[1] - first_corner_px[1]])

        angle_deg = self._compute_angle_between_vectors_deg(frame_x_axis, marker_top_edge_px)
        return angle_deg

    # Calculate the angle between the two given vectors
    # https://stackoverflow.com/questions/2827393/angles-between-two-n-dimensional-vectors-in-python
    # Co-authored by Claude Code: names
    def _compute_angle_between_vectors_deg(self, first_vector, second_vector):
        angle_rad = math.atan2(np.linalg.det([first_vector, second_vector]),
                               np.dot(first_vector, second_vector))
        angle_deg = np.degrees(angle_rad)
        if angle_deg < 0:
            angle_deg = angle_deg + 360
        # TODO: ROUND HERE OR RETURN FLOAT INSTEAD?
        return int(angle_deg)

    # Get the centroid of a polygon
    # https://progr.interplanety.org/en/python-how-to-find-the-polygon-center-coordinates/
    # Co-authored by Claude Code: names
    def _compute_centroid_px(self, vertices_px):
        x_values_px = [vertex_px[0] for vertex_px in vertices_px]
        y_values_px = [vertex_px[1] for vertex_px in vertices_px]
        vertex_count = len(vertices_px)
        centroid_x_px = sum(x_values_px) / vertex_count
        centroid_y_px = sum(y_values_px) / vertex_count
        return centroid_x_px, centroid_y_px

    # OLD CODE FOR DEBUGGING DETECTION ACCURACY
    # DEBUG_MODE = True
    # N_FRAMES_DETECTION_RATIO = 100
    # markers_detected = [False] * N_FRAMES_DETECTION_RATIO
    # marker_detection_rate = None
    # if DEBUG_MODE:
    #     self.markers_detected = self.markers_detected[1:N_FRAMES_DETECTION_RATIO] + [
    #         True if (len(aruco_markers) > 0) else False]
    #     self.marker_detection_rate = sum(self.markers_detected) / N_FRAMES_DETECTION_RATIO


# Co-authored by Claude Code: image path as argument, markers drawn here instead of in
# detect_fiducials, so detection never opens a window
if __name__ == '__main__':
    import sys

    if len(sys.argv) != 2:
        sys.exit('[fiducials_detector]: usage: python -m fiducials_detector.fiducials_detector '
                 '<image path>')
    test_frame = cv2.imread(sys.argv[1])
    if test_frame is None:
        sys.exit(f'[fiducials_detector]: could not read the image {sys.argv[1]}')
    fiducials_detector = FiducialsDetector(debug=True)
    detected_markers = fiducials_detector.detect_fiducials(test_frame)
    print(f'[fiducials_detector]: found ids {[int(marker["id"]) for marker in detected_markers]}')
    # The debug preview is open: keep it until a key is pressed
    cv2.waitKey(0)

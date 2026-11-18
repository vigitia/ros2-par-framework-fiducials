# Written with the help of Claude Code

import math

import cv2
import cv2.aruco as aruco
import numpy as np

from .utils.detection_preview_window import DetectionPreviewWindow
from .utils.fiducials_detector_debugger import DetectionAccuracyDebugger

# Available settings added with help of Claude Code.

# Prefixes of the cv2.aruco dictionary constants per marker type, e.g. DICT_4X4_100 or
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


# Written by Claude Code
def _is_marker_in_list(candidate_marker, markers):
    """Return True if the list "markers" holds one with the same id whose centroid lies within its size."""
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
def _create_marker_dict(marker_id, corners_px):
    """Return the dict that describes one detected marker; corners_px is 4x2 in frame pixels."""
    return {
        'id': marker_id,
        'angle': _compute_marker_angle_deg(corners_px),
        'corners': corners_px,
        'centroid': corners_px.mean(axis=0),
    }


# Co-authored by Claude Code
def _compute_marker_angle_deg(marker_corners_px):
    """
    Return the angle of the marker's top edge against the image's x axis, in whole degrees.

    The top edge runs from the first to the second corner. The angle grows from the x axis
    towards the y axis (clockwise in the image) and lies in 0 to 359. Whole degrees, because
    Fiducial.angle is an int32; the fraction is cut off.
    """
    top_edge_px = marker_corners_px[1] - marker_corners_px[0]
    angle_deg = math.degrees(math.atan2(top_edge_px[1], top_edge_px[0]))
    if angle_deg < 0:
        angle_deg = angle_deg + 360
    return int(angle_deg)


# Written by Claude Code
def _create_detector_parameters(corner_refinement_method, adaptive_thresh_constant,
                                adaptive_thresh_win_size_min_px, adaptive_thresh_win_size_max_px,
                                bit_sample_px_per_cell, bit_ignored_margin_per_cell):
    """Return aruco.DetectorParameters with the given settings, OpenCV's defaults otherwise."""
    detector_parameters = aruco.DetectorParameters()
    detector_parameters.cornerRefinementMethod = corner_refinement_method
    detector_parameters.adaptiveThreshConstant = adaptive_thresh_constant
    # The image is thresholded once per window size from min to max (step 10 px) and searched
    # for markers each time: fewer windows are faster, and windows much smaller than the
    # markers in the image only add noise
    detector_parameters.adaptiveThreshWinSizeMin = adaptive_thresh_win_size_min_px
    detector_parameters.adaptiveThreshWinSizeMax = adaptive_thresh_win_size_max_px
    # Each candidate's bits are read from its square resampled to this many pixels per cell,
    # ignoring this fraction of each cell at its edges. Markers only a few camera pixels per
    # cell wide are read more reliably from more samples and a wider margin, which average
    # out sensor noise and blurred cell edges
    detector_parameters.perspectiveRemovePixelPerCell = bit_sample_px_per_cell
    detector_parameters.perspectiveRemoveIgnoredMarginPerCell = bit_ignored_margin_per_cell
    return detector_parameters


class FiducialsDetector:
    """Detects ArUco markers or AprilTags in camera frames."""

    # Co-authored by Claude Code
    def __init__(self, marker_type='aruco', dictionary_name='', corner_refinement='none',
                 adaptive_thresh_constant=7.0, adaptive_thresh_win_size_min_px=3,
                 adaptive_thresh_win_size_max_px=23, bit_sample_px_per_cell=4,
                 bit_ignored_margin_per_cell=0.13, debug=False):
        self.debug = debug

        if marker_type not in MARKER_DICTIONARY_PREFIXES:
            raise ValueError(f'marker_type must be one of {list(MARKER_DICTIONARY_PREFIXES)}, '
                             f"got '{marker_type}'")
        if corner_refinement not in CORNER_REFINEMENT_METHODS:
            raise ValueError(f'corner_refinement must be one of '
                             f"{list(CORNER_REFINEMENT_METHODS)}, got '{corner_refinement}'")
        if adaptive_thresh_win_size_min_px < MIN_ADAPTIVE_THRESH_WIN_SIZE_PX:
            raise ValueError(f'adaptive_thresh_win_size_min_px must be at least '
                             f'{MIN_ADAPTIVE_THRESH_WIN_SIZE_PX}, got '
                             f'{adaptive_thresh_win_size_min_px}')
        if adaptive_thresh_win_size_max_px < adaptive_thresh_win_size_min_px:
            raise ValueError(f'adaptive_thresh_win_size_max_px '
                             f'({adaptive_thresh_win_size_max_px}) must not be smaller than '
                             f'adaptive_thresh_win_size_min_px '
                             f'({adaptive_thresh_win_size_min_px})')
        if bit_sample_px_per_cell < 1:
            raise ValueError(f'bit_sample_px_per_cell must be at least 1, got '
                             f'{bit_sample_px_per_cell}')
        # Half a cell's margin on both sides would leave nothing of the cell to read
        if not 0.0 <= bit_ignored_margin_per_cell < 0.5:
            raise ValueError(f'bit_ignored_margin_per_cell must be in [0, 0.5), got '
                             f'{bit_ignored_margin_per_cell}')

        dictionary_prefix = MARKER_DICTIONARY_PREFIXES[marker_type]
        if dictionary_name == '':
            dictionary_name = DEFAULT_MARKER_DICTIONARIES[marker_type]
        dictionary_id = getattr(aruco, dictionary_prefix + dictionary_name, None)
        if dictionary_id is None:
            valid_dictionary_names = [name[len(dictionary_prefix):] for name in dir(aruco)
                                      if name.startswith(dictionary_prefix)]
            raise ValueError(f"Unknown {marker_type} dictionary '{dictionary_name}', valid are "
                             f'{valid_dictionary_names}')

        # Setup aruco dictionary, parameters and detector
        self.aruco_dictionary = aruco.getPredefinedDictionary(dictionary_id)
        corner_refinement_method = CORNER_REFINEMENT_METHODS[corner_refinement]
        self.aruco_detector_parameters = _create_detector_parameters(
            corner_refinement_method, adaptive_thresh_constant, adaptive_thresh_win_size_min_px,
            adaptive_thresh_win_size_max_px, bit_sample_px_per_cell, bit_ignored_margin_per_cell)
        self.aruco_detector = aruco.ArucoDetector(self.aruco_dictionary,
                                                  self.aruco_detector_parameters)

        # For detect_fiducials_in_windows, created on its first use, see _get_window_detector
        self.window_detector = None
        self.window_detector_size_px = None

        # In debug mode, start the Preview Prints and Window
        if self.debug:
            self.detection_accuracy_debugger = DetectionAccuracyDebugger()
            self.detection_preview_window = DetectionPreviewWindow()

    # Co-authored by Claude Code: names, debug preview
    def detect_fiducials(self, camera_frame) -> list:
        if self.debug:
            self.detection_accuracy_debugger.on_new_frame()

        marker_corners_px, marker_ids, _ = self.aruco_detector.detectMarkers(camera_frame)

        markers = []

        # detectMarkers returns None when it finds no markers
        if marker_ids is not None:
            for corners_px, marker_id in zip(marker_corners_px, marker_ids):
                detected_marker = _create_marker_dict(marker_id[0], corners_px[0])
                markers.append(detected_marker)
                if self.debug:
                    self.detection_accuracy_debugger.on_new_marker(detected_marker['id'])

        if self.debug:
            self.detection_preview_window.show_preview_frame_with_markers(camera_frame, marker_corners_px,
                                                                          marker_ids)

        return markers

    # Written with the help of Claude Code
    def detect_fiducials_in_windows(self, camera_frame_mono, search_windows_px) -> list:
        """
        Detect markers only inside the given windows of a greyscale frame.

        Only when `tracking_mode` is set to `True`
        search_windows_px: (left, top, right, bottom) boxes in frame pixels, already clipped to
        the frame. Returns the same marker dicts as detect_fiducials, in frame pixels. A marker
        seen in two overlapping windows is returned once. No debug preview: it would show only
        the windows.
        """
        markers = []
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
                detected_marker = _create_marker_dict(marker_id[0], frame_corners_px)
                if not _is_marker_in_list(detected_marker, markers):
                    markers.append(detected_marker)
        return markers

    # Written by Claude Code
    def _get_window_detector(self, window_size_px, frame_size_px):
        """
        Return the window detector, with the full frame's smallest marker size in pixels.

        OpenCV sets the smallest marker perimeter relative to the image size. In a small window
        that relative value allows tiny contours, and every speck of sensor noise is then checked
        as a marker candidate, which makes a noisy window several times slower.

        Created on the first call, so an instance that only scans full frames builds no window
        detector. It starts with the full-frame settings: ArucoDetector keeps its own copy of
        the parameters, so rescaling it leaves aruco_detector_parameters unchanged.
        """
        if self.window_detector is None:
            self.window_detector = aruco.ArucoDetector(self.aruco_dictionary,
                                                       self.aruco_detector_parameters)
        if window_size_px != self.window_detector_size_px:
            min_perimeter_px = (self.aruco_detector_parameters.minMarkerPerimeterRate
                                * frame_size_px)
            # getDetectorParameters returns a copy, so the change applies only through
            # setDetectorParameters
            window_detector_parameters = self.window_detector.getDetectorParameters()
            window_detector_parameters.minMarkerPerimeterRate = min_perimeter_px / window_size_px
            self.window_detector.setDetectorParameters(window_detector_parameters)
            self.window_detector_size_px = window_size_px
        return self.window_detector

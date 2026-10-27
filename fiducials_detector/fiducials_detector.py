# Choice between ArUco markers and AprilTags, the dictionary and detector settings as
# constructor arguments added with help of Claude Code. Claude Code also renamed constants,
# functions and variables so their names say what they hold, with units.

import math

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
CORNER_REFINEMENT_METHODS = {
    'none': aruco.CORNER_REFINE_NONE,
    'subpix': aruco.CORNER_REFINE_SUBPIX,
    'contour': aruco.CORNER_REFINE_CONTOUR,
    'apriltag': aruco.CORNER_REFINE_APRILTAG,
}

DEBUG_DETECTION_ACCURACY = False

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


# Currently supporting Aruco Markers and Apriltags
class FiducialsDetector:

    # Co-authored by Claude Code
    def __init__(self, marker_type='aruco', dictionary_name='', corner_refinement='none',
                 adaptive_thresh_constant=7.0):
        if marker_type not in MARKER_DICTIONARY_PREFIXES:
            raise ValueError(f'marker_type must be one of {list(MARKER_DICTIONARY_PREFIXES)}, '
                             f'got {marker_type!r}')
        if corner_refinement not in CORNER_REFINEMENT_METHODS:
            raise ValueError(f'corner_refinement must be one of '
                             f'{list(CORNER_REFINEMENT_METHODS)}, got {corner_refinement!r}')

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

        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dictionary,
                                                      self.aruco_detector_parameters)

        if DEBUG_DETECTION_ACCURACY:
            self.detection_accuracy_debugger = DetectionAccuracyDebugger()

    # Co-authored by Claude Code: names
    def detect_fiducials(self, camera_frame) -> list:
        if DEBUG_DETECTION_ACCURACY:
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
                if DEBUG_DETECTION_ACCURACY:
                    self.detection_accuracy_debugger.on_new_marker(detected_marker['id'])

        return fiducials

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
    test_frame = cv2.resize(test_frame, (800, 600))
    fiducials_detector = FiducialsDetector()
    detected_markers = fiducials_detector.detect_fiducials(test_frame)
    print(f'[fiducials_detector]: found ids {[int(marker["id"]) for marker in detected_markers]}')
    test_frame_corners_px = [marker['corners'][None] for marker in detected_markers]
    test_frame_ids = np.array([[marker['id']] for marker in detected_markers])
    cv2.aruco.drawDetectedMarkers(test_frame, test_frame_corners_px, test_frame_ids)
    cv2.imshow('Detected Aruco Markers', test_frame)
    cv2.waitKey(0)

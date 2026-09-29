# Choice between ArUco markers and AprilTags, the dictionary and detector settings as
# constructor arguments added with help of Claude Code

import math

import cv2
import cv2.aruco as aruco
import numpy as np

from .fiducials_detector_debugger import FiducialsDetectorDebugger

# Prefix of the cv2.aruco dictionary constants per marker type, e.g. DICT_4X4_100 or
# DICT_APRILTAG_36h11
DICTIONARY_PREFIXES = {
    'aruco': 'DICT_',
    'apriltag': 'DICT_APRILTAG_',
}
# Used when no dictionary is given, so switching the marker type alone gives a working detector
DEFAULT_DICTIONARIES = {
    'aruco': '4X4_100',
    'apriltag': '36h11',
}
CORNER_REFINEMENTS = {
    'none': aruco.CORNER_REFINE_NONE,
    'subpix': aruco.CORNER_REFINE_SUBPIX,
    'contour': aruco.CORNER_REFINE_CONTOUR,
    'apriltag': aruco.CORNER_REFINE_APRILTAG,
}

DEBUG_MODE = False

# Following lines until corners_to_camera_space written by Claude Code
# The depth of a marker is taken from its inner part only: the polygon shrunk towards its centre to
# this fraction. Pixels at the edge may already see what lies around the marker.
INNER_FRACTION = 0.7
# Fewer valid depth pixels than this in the inner part: no 3D position, e.g. a marker far away
MIN_DEPTH_PIXELS = 20
# Less of the inner part with depth than this: no 3D position. Seen live on a cube's side face that
# the depth camera saw almost edge-on: 13 % valid pixels, and those were the table behind it.
MIN_DEPTH_COVERAGE = 0.5
# A square of edge 1 in solvePnP's IPPE_SQUARE order, which is also OpenCV's marker corner order
# (clockwise from the top left). The real size comes from the depth, so no marker size is needed.
UNIT_SQUARE = np.array([[-0.5, 0.5, 0.0], [0.5, 0.5, 0.0], [0.5, -0.5, 0.0], [-0.5, -0.5, 0.0]])
# A pose that misses the corners by more than this is discarded: they are not the image of a square
# (e.g. a wrong detection). Also IPPE's one degenerate case, a noise-free square facing the camera
# exactly, which only synthetic data produces.
MAX_REPROJECTION_ERROR_PX = 3.0


def corners_to_camera_space(corners, depth_m, k, d):
    """
    Return the four corners of a marker in the camera's optical frame, in metres, or None.

    corners: 4x2 pixel positions in the raw colour image. depth_m: depth image registered to that
    image (the same pixel grid), in metres, 0 where there is no depth. k, d: camera matrix and
    distortion coefficients of the colour image.

    The orientation comes from the marker's square in the colour image (IPPE), the distance from
    the depth inside it. The depth alone gave steep faces a wrong tilt: on a cube's side face 10
    degrees too steep, a 60 x 77 mm "square" for a 57 mm marker, probably because the
    time-of-flight light also reaches the face via the table in the corner between them
    (multipath). The result is a square that lies on the corners' rays. Returns a 4x3 array; None
    when too little of the marker has depth, or the corners are no view of a square.
    """
    corners = np.asarray(corners, np.float64).reshape(4, 2)
    center = corners.mean(axis=0)
    inner = center + INNER_FRACTION * (corners - center)
    # Mask only the bounding box, not the whole image, for every marker in every frame
    height, width = depth_m.shape[:2]
    u0, v0 = np.maximum(np.floor(inner.min(axis=0)).astype(int), 0)
    u1, v1 = np.minimum(np.ceil(inner.max(axis=0)).astype(int) + 1, (width, height))
    if u0 >= u1 or v0 >= v1:
        return None
    box = depth_m[v0:v1, u0:u1]
    mask = np.zeros(box.shape, np.uint8)
    cv2.fillConvexPoly(mask, np.round(inner - (u0, v0)).astype(np.int32), 1)
    v, u = np.nonzero((mask > 0) & (box > 0))
    if u.size < MIN_DEPTH_PIXELS or u.size < MIN_DEPTH_COVERAGE * np.count_nonzero(mask):
        return None

    # A square seen at an angle is ambiguous: IPPE returns both tilts that fit the image
    found, rvecs, tvecs, errors = cv2.solvePnPGeneric(UNIT_SQUARE, corners, k, d,
                                                      flags=cv2.SOLVEPNP_IPPE_SQUARE)
    poses = [(cv2.Rodrigues(rvec)[0], tvec.ravel())
             for rvec, tvec, error in zip(rvecs, tvecs, errors)
             if error.ravel()[0] <= MAX_REPROJECTION_ERROR_PX]
    if not found or not poses:
        return None
    # The depth's tilt is not exact, but good enough to tell the two apart: its plane's normal is
    # the direction of least spread of the depth points
    z = box[v, u].astype(np.float64)
    normalized = cv2.undistortPoints(np.column_stack([u + u0, v + v0]).astype(np.float64)
                                     .reshape(-1, 1, 2), k, d).reshape(-1, 2)
    points = np.column_stack([normalized * z[:, None], z])
    depth_normal = np.linalg.svd(points - points.mean(axis=0))[2][2]
    # The square lies in its own xy plane, so the rotation's third column is its normal
    rotation, translation = max(poses, key=lambda pose: abs(pose[0][:, 2] @ depth_normal))
    unit_corners = UNIT_SQUARE @ rotation.T + translation
    # Scaled about the camera, every corner stays on its ray. The median depth of the inner part
    # is the depth of the marker's centre; a finger on part of the marker does not move it.
    scale = np.median(z) / unit_corners.mean(axis=0)[2]
    if not (np.isfinite(scale) and scale > 0):
        return None
    return unit_corners * scale

# Currently supporting Aruco Markers and Apriltags
class FiducialsDetector:

    # Co-authored by Claude Code
    def __init__(self, node=None, marker_type='aruco', dictionary='', corner_refinement='none',
                 adaptive_thresh_constant=7.0):
        self.node = node

        if marker_type not in DICTIONARY_PREFIXES:
            raise ValueError(f'marker_type must be one of {list(DICTIONARY_PREFIXES)}, '
                             f'got {marker_type!r}')
        if corner_refinement not in CORNER_REFINEMENTS:
            raise ValueError(f'corner_refinement must be one of {list(CORNER_REFINEMENTS)}, '
                             f'got {corner_refinement!r}')

        prefix = DICTIONARY_PREFIXES[marker_type]
        dictionary = dictionary or DEFAULT_DICTIONARIES[marker_type]
        dictionary_id = getattr(aruco, prefix + dictionary, None)
        if dictionary_id is None:
            valid = [name[len(prefix):] for name in dir(aruco) if name.startswith(prefix)]
            raise ValueError(f'Unknown {marker_type} dictionary {dictionary!r}, valid are {valid}')

        self.aruco_dictionary = aruco.getPredefinedDictionary(dictionary_id)
        self.aruco_detector_parameters = aruco.DetectorParameters()
        parameters = self.aruco_detector_parameters
        parameters.cornerRefinementMethod = CORNER_REFINEMENTS[corner_refinement]
        parameters.adaptiveThreshConstant = adaptive_thresh_constant

        self.detector = cv2.aruco.ArucoDetector(self.aruco_dictionary, self.aruco_detector_parameters)

        if DEBUG_MODE:
            self.debugger = FiducialsDetectorDebugger()

    def detect_fiducials(self, frame) -> list:
        if DEBUG_MODE:
            self.debugger.on_new_frame()

        # Convert RGB frames to greyscale
        if len(frame.shape) == 3:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected_points = self.detector.detectMarkers(frame)

        fiducials = []

        # check if the list of IDs is not empty
        if np.all(ids is not None):
            for i in range(len(ids)):
                aruco_marker = {'id': ids[i][0],
                                'angle': self.calculate_aruco_marker_rotation(corners[i][0], frame),
                                'corners': corners[i][0],
                                'centroid': self.centroid(corners[i][0])}
                fiducials.append(aruco_marker)
                if DEBUG_MODE:
                    self.debugger.on_new_marker(aruco_marker['id'])

        if self.node is None:
            # TODO: Draw marker in frame for debugging
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            cv2.imshow(f'Detected Aruco Markers', frame)
            cv2.waitKey(0)

        return fiducials

    # Calculate the rotation_diff of an aruco marker relative to the frame.
    # Returns the angle in the range from 0° to 360°
    def calculate_aruco_marker_rotation(self, aruco_marker_corners, frame):
        tracker_point_one = aruco_marker_corners[0]
        tracker_point_two = aruco_marker_corners[1]
        vector_one = np.array([frame.shape[0], 0])
        vector_two = np.array(
            [tracker_point_two[0] - tracker_point_one[0], tracker_point_two[1] - tracker_point_one[1]])

        angle = self.calculate_angle(vector_one, vector_two)
        return angle

    # Calculate the angle between the two given vectors
    # https://stackoverflow.com/questions/2827393/angles-between-two-n-dimensional-vectors-in-python
    def calculate_angle(self, v1, v2):
        angle = math.atan2(np.linalg.det([v1, v2]), np.dot(v1, v2))
        angle = np.degrees(angle)
        if angle < 0:
            angle = angle + 360
        # TODO: ROUND HERE OR RETURN FLOAT INSTEAD?
        return int(angle)

    # Get the centroid of a polygon
    # https://progr.interplanety.org/en/python-how-to-find-the-polygon-center-coordinates/
    def centroid(self, vertexes):
        _x_list = [vertex[0] for vertex in vertexes]
        _y_list = [vertex[1] for vertex in vertexes]
        _len = len(vertexes)
        _x = sum(_x_list) / _len
        _y = sum(_y_list) / _len
        return _x, _y

    # OLD CODE FOR DEBUGGING DETECTION ACCURACY
    # DEBUG_MODE = True
    # N_FRAMES_DETECTION_RATIO = 100
    # markers_detected = [False] * N_FRAMES_DETECTION_RATIO
    # marker_detection_rate = None
    # if DEBUG_MODE:
    #     self.markers_detected = self.markers_detected[1:N_FRAMES_DETECTION_RATIO] + [
    #         True if (len(aruco_markers) > 0) else False]
    #     self.marker_detection_rate = sum(self.markers_detected) / N_FRAMES_DETECTION_RATIO


if __name__ == '__main__':
    fiducials_detector = FiducialsDetector()
    test_frame = cv2.imread('assets/aruco_4x4.jpg')
    test_frame = cv2.resize(test_frame, (800, 600))
    aruco_markers = fiducials_detector.detect_fiducials(test_frame)
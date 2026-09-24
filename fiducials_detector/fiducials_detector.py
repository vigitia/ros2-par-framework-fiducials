import math

import cv2
import cv2.aruco as aruco
import numpy as np

from .fiducials_detector_debugger import FiducialsDetectorDebugger

ADAPTIVE_THRESH_CONSTANT = 10  # TODO: Find best value

# TODO: Add settings option to switch between aruco markers and apriltags
# ARUCO_DICT = aruco.DICT_4X4_50
ARUCO_DICT = aruco.DICT_4X4_100
# ARUCO_DICT = aruco.DICT_APRILTAG_16H5
# ARUCO_DICT = aruco.DICT_5X5_50
# ARUCO_DICT = aruco.DICT_5X5_50
# ARUCO_DICT = aruco.DICT_6X6_50

DEBUG_MODE = False

# Currently supporting Aruco Markers and Apriltags
class FiducialsDetector:

    def __init__(self, node=None):
        self.node = node
        self.aruco_dictionary = aruco.getPredefinedDictionary(ARUCO_DICT)
        self.aruco_detector_parameters = aruco.DetectorParameters()
        # TODO: Add options to change more parameters here to improve detection results

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
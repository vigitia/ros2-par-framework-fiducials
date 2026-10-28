# Co-authored by Claude Code. Claude Code added the compatibility with the camera drivers
# (topics, best-effort QoS, colour encoding), the parameters for the marker type, dictionary and
# detector settings, the optional depth input (use_depth) that fills corners_3d, the optional
# compressed input (use_compressed), and split the node into one method per step.

import time

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np
from par_framework_interfaces.msg import Fiducial, FiducialsList
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

from .fiducials_detector import (compute_corners_in_camera_space, DepthArrivalMonitor,
                                 FiducialsDetector)

NODE_NAME = 'fiducials_detector_node'
TOPIC_NAME_FIDUCIALS = 'fiducials'
QUEUE_SIZE = 10
# The PAR framework's camera topics, onto which its launch files remap every camera driver: the
# colour image, depth registered to it (from a depth camera), and the colour
# camera's intrinsics. Another topic is chosen by remapping.
TOPIC_NAME_FRAME = '/camera/image_raw'
# With use_compressed, its compressed variant, named by the image_transport convention: JPEGs a
# camera driver passes through, so it doesn't have to decode them for this node
TOPIC_NAME_FRAME_COMPRESSED = TOPIC_NAME_FRAME + '/compressed'
TOPIC_NAME_FRAME_DEPTH = '/camera/depth_registered/image_raw'
TOPIC_NAME_CAMERA_INFO = '/camera/camera_info'
SYNC_LEEWAY_TIME_SEC = 0.01
# With use_depth, frames are only processed in pairs with a depth frame. Colour frames arriving
# without any depth for this long means no depth camera: nothing is published, which the node
# then says.
DEPTH_MISSING_TIMEOUT_SEC = 2.0
DEPTH_CHECK_PERIOD_SEC = 1.0
WARNING_THROTTLE_PERIOD_SEC = 5.0
# REP 118: depth images in 16UC1 are in millimetres, 32FC1 in metres
M_PER_MM = 0.001

# Camera drivers may publish frames best-effort. A reliable subscriber would be
# QoS-incompatible and silently receive nothing, while a best-effort one also connects to reliable
# publishers. Only the newest frame matters, detecting on an old one would publish outdated marker
# positions.
FRAME_QOS = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                       history=QoSHistoryPolicy.KEEP_LAST, depth=1)

PARAMETER_MARKER_TYPE = 'marker_type'
PARAMETER_DICTIONARY = 'dictionary'
PARAMETER_CORNER_REFINEMENT = 'corner_refinement'
PARAMETER_ADAPTIVE_THRESH_CONSTANT = 'adaptive_thresh_constant'
PARAMETER_USE_DEPTH = 'use_depth'
PARAMETER_USE_COMPRESSED = 'use_compressed'
PARAMETER_DEBUG = 'debug'


class FiducialsDetectorNode(Node):
    # Co-authored by Claude Code
    def __init__(self):
        super().__init__(node_name=NODE_NAME)

        self._declare_parameters()

        self.cv_bridge = CvBridge()
        self.fiducials_detector = FiducialsDetector(
            marker_type=self.get_parameter(PARAMETER_MARKER_TYPE).value,
            dictionary_name=self.get_parameter(PARAMETER_DICTIONARY).value,
            corner_refinement=self.get_parameter(PARAMETER_CORNER_REFINEMENT).value,
            adaptive_thresh_constant=self.get_parameter(PARAMETER_ADAPTIVE_THRESH_CONSTANT).value,
            debug=self.get_parameter(PARAMETER_DEBUG).value)

        self.camera_info = None
        self._create_subscriptions()

        self.fiducials_publisher = self.create_publisher(msg_type=FiducialsList,
                                                         topic=TOPIC_NAME_FIDUCIALS,
                                                         qos_profile=QUEUE_SIZE)

    # Written with the help of Claude Code
    def _declare_parameters(self):
        # All read-only because the subscription and the detector are created once at startup, a
        # later change would have no effect
        self.declare_parameter(PARAMETER_MARKER_TYPE, 'aruco', ParameterDescriptor(
            description='Type of markers to detect: aruco or apriltag.', read_only=True))
        self.declare_parameter(PARAMETER_DICTIONARY, '', ParameterDescriptor(
            description='Marker dictionary without the DICT_ / DICT_APRILTAG_ prefix, e.g. '
                        '4X4_100 or 6X6_250 for aruco, 16h5 or 36h11 for apriltag. Empty uses '
                        '4X4_100 / 36h11.',
            read_only=True))
        self.declare_parameter(PARAMETER_CORNER_REFINEMENT, 'none', ParameterDescriptor(
            description='Corner refinement: none, subpix, contour or apriltag. Refinement '
                        'makes corners more precise at the cost of detection time.',
            read_only=True))
        self.declare_parameter(PARAMETER_ADAPTIVE_THRESH_CONSTANT, 7.0, ParameterDescriptor(
            description='Constant subtracted in the adaptive thresholding (OpenCV default 7). '
                        'Tune it if markers are missed under the projector light.',
            read_only=True))
        self.declare_parameter(PARAMETER_USE_DEPTH, False, ParameterDescriptor(
            description=f'Also subscribe to {TOPIC_NAME_FRAME_DEPTH} (depth registered to the '
                        f"colour image, from a depth camera) and fill each marker's "
                        f'corners_3d, in metres in the optical frame.',
            read_only=True))
        self.declare_parameter(PARAMETER_USE_COMPRESSED, False, ParameterDescriptor(
            description=f'Subscribe to {TOPIC_NAME_FRAME_COMPRESSED} '
                        f'(sensor_msgs/CompressedImage, e.g. JPEGs passed through from the '
                        f'camera) instead of {TOPIC_NAME_FRAME}, and decode them straight to '
                        f'greyscale. Only for camera drivers that publish it.',
            read_only=True))
        self.declare_parameter(PARAMETER_DEBUG, False, ParameterDescriptor(
            description='Show the frames with the detected markers in an OpenCV window (needs a '
                        'display) and print the detection accuracy. Slows detection down a '
                        'little, so only for testing.',
            read_only=True))

    # Written by Claude Code
    def _create_subscriptions(self):
        # Raw frames, or with use_compressed their compressed variant
        if self.get_parameter(PARAMETER_USE_COMPRESSED).value:
            frame_msg_type = CompressedImage
            topic_name_frame = TOPIC_NAME_FRAME_COMPRESSED
        else:
            frame_msg_type = Image
            topic_name_frame = TOPIC_NAME_FRAME

        if self.get_parameter(PARAMETER_USE_DEPTH).value:
            self._create_frame_and_depth_subscriptions(frame_msg_type, topic_name_frame)
            return
        self.create_subscription(msg_type=frame_msg_type,
                                 topic=topic_name_frame,
                                 callback=self._on_frame_received,
                                 qos_profile=FRAME_QOS)

    # Written by Claude Code
    def _create_frame_and_depth_subscriptions(self, frame_msg_type, topic_name_frame):
        """Process colour and depth frames in pairs, and warn when no depth arrives."""
        frame_subscriber = Subscriber(self, frame_msg_type, topic_name_frame,
                                      qos_profile=FRAME_QOS)
        depth_subscriber = Subscriber(self, Image, TOPIC_NAME_FRAME_DEPTH, qos_profile=FRAME_QOS)
        self.frame_depth_synchronizer = ApproximateTimeSynchronizer(
            [frame_subscriber, depth_subscriber], QUEUE_SIZE, SYNC_LEEWAY_TIME_SEC)
        self.frame_depth_synchronizer.registerCallback(self._on_frame_and_depth_received)

        # Each subscriber also reports its own messages, so a missing depth camera shows up even
        # though the synchronizer then never forms a pair
        self.depth_arrival_monitor = DepthArrivalMonitor(DEPTH_MISSING_TIMEOUT_SEC)
        frame_subscriber.registerCallback(self._on_frame_arrived)
        depth_subscriber.registerCallback(self._on_depth_arrived)
        self.create_timer(DEPTH_CHECK_PERIOD_SEC, self._warn_if_depth_missing)
        # The name after remapping, which is what the warnings must name
        self.depth_topic_name = depth_subscriber.sub.topic_name

        self.create_subscription(msg_type=CameraInfo,
                                 topic=TOPIC_NAME_CAMERA_INFO,
                                 callback=self._on_camera_info,
                                 qos_profile=FRAME_QOS)

    # Written by Claude Code
    def _on_camera_info(self, camera_info_msg):
        self.camera_info = camera_info_msg

    # Written by Claude Code
    def _on_frame_arrived(self, frame_msg):
        self.depth_arrival_monitor.on_frame(time.monotonic())

    # Written by Claude Code
    def _on_depth_arrived(self, depth_msg):
        self.depth_arrival_monitor.on_depth(time.monotonic())

    # Written by Claude Code
    def _warn_if_depth_missing(self):
        if self.depth_arrival_monitor.is_depth_missing(time.monotonic()):
            self.get_logger().warning(
                f'use_depth is true, but no depth frames arrive on {self.depth_topic_name}, so '
                f'no fiducials are published. Connect a depth camera or set use_depth to false.',
                throttle_duration_sec=WARNING_THROTTLE_PERIOD_SEC)

    # Written by Claude Code
    def _on_frame_and_depth_received(self, frame_msg, depth_msg):
        depth_image_m = self.cv_bridge.imgmsg_to_cv2(depth_msg)
        if depth_msg.encoding == '16UC1':
            depth_image_m = depth_image_m.astype(np.float32) * M_PER_MM
        self._on_frame_received(frame_msg, depth_image_m)

    # Co-authored by Claude Code: one method per step
    def _on_frame_received(self, frame_msg, depth_image_m=None):
        """Detect the markers in a raw or compressed frame, with depth (metres) for corners_3d."""
        frame_mono = self._decode_frame_mono(frame_msg)
        if frame_mono is None:
            return
        if depth_image_m is not None:
            depth_image_m = self._get_depth_image_on_frame_grid(depth_image_m, frame_mono)

        fiducials = self.fiducials_detector.detect_fiducials(frame_mono)
        if depth_image_m is not None:
            self._add_corners_3d(fiducials, depth_image_m)

        # Published for every frame, also without markers: an empty list is how subscribers learn
        # that a marker disappeared, otherwise its last position would stay valid for them forever
        fiducials_list_msg = self._create_fiducials_list_msg(fiducials, frame_msg.header)
        self.fiducials_publisher.publish(fiducials_list_msg)

    # Written by Claude Code
    def _decode_frame_mono(self, frame_msg):
        """Return the frame in greyscale, or None if a compressed frame can't be decoded."""
        if not isinstance(frame_msg, CompressedImage):
            # Asking cv_bridge for mono8 converts bgr8, rgb8 and mono8 frames correctly,
            # whichever encoding the camera node is set to
            return self.cv_bridge.imgmsg_to_cv2(frame_msg, desired_encoding='mono8')

        # A JPEG decoded straight to greyscale skips its colour channels
        jpeg_bytes = np.frombuffer(frame_msg.data, np.uint8)
        frame_mono = cv2.imdecode(jpeg_bytes, cv2.IMREAD_GRAYSCALE)
        if frame_mono is None:
            self.get_logger().warning(
                f'could not decode a frame on {TOPIC_NAME_FRAME_COMPRESSED} '
                f'(format "{frame_msg.format}"), skipped',
                throttle_duration_sec=WARNING_THROTTLE_PERIOD_SEC)
        return frame_mono

    # Written by Claude Code
    def _get_depth_image_on_frame_grid(self, depth_image_m, frame_mono):
        """Return depth_image_m if it is on the frame's pixel grid, else None with a warning."""
        if depth_image_m.shape[:2] == frame_mono.shape[:2]:
            return depth_image_m
        # A depth image on another pixel grid (e.g. /depth/image_raw) would give every marker the
        # depth of some other point
        depth_height_px, depth_width_px = depth_image_m.shape[:2]
        frame_height_px, frame_width_px = frame_mono.shape[:2]
        self.get_logger().warning(
            f'depth image is {depth_width_px}x{depth_height_px}, the colour image '
            f'{frame_width_px}x{frame_height_px}: {self.depth_topic_name} must be registered to '
            f'the colour image, no 3D positions',
            throttle_duration_sec=WARNING_THROTTLE_PERIOD_SEC)
        return None

    # Written by Claude Code
    def _add_corners_3d(self, fiducials, depth_image_m):
        """Add each marker's corners in the camera's optical frame, once intrinsics are known."""
        # Uncalibrated, the camera matrix is all zeros and pixels have no direction
        if self.camera_info is None or self.camera_info.k[0] == 0.0:
            return
        camera_matrix = np.reshape(self.camera_info.k, (3, 3))
        distortion_coefficients = np.array(self.camera_info.d)
        for detected_marker in fiducials:
            detected_marker['corners_3d'] = compute_corners_in_camera_space(
                detected_marker['corners'], depth_image_m, camera_matrix, distortion_coefficients)

    # Co-authored by Claude Code: names, corners unpacked
    @staticmethod
    def _create_fiducials_list_msg(detected_markers, header):
        fiducials_list_msg = FiducialsList(header=header)
        for detected_marker in detected_markers:
            (x0_px, y0_px), (x1_px, y1_px), (x2_px, y2_px), (x3_px, y3_px) = (
                detected_marker['corners'].tolist())
            centroid_x_px, centroid_y_px = detected_marker['centroid']
            fiducial_msg = Fiducial(id=int(detected_marker['id']),
                                    angle=detected_marker['angle'],
                                    x0=x0_px, y0=y0_px, x1=x1_px, y1=y1_px,
                                    x2=x2_px, y2=y2_px, x3=x3_px, y3=y3_px,
                                    x_centroid=float(centroid_x_px),
                                    y_centroid=float(centroid_y_px))
            # Only with use_depth and enough depth on the marker
            if detected_marker.get('corners_3d') is not None:
                fiducial_msg.corners_3d = [Point(x=float(x_m), y=float(y_m), z=float(z_m))
                                           for x_m, y_m, z_m in detected_marker['corners_3d']]
            fiducials_list_msg.fiducials.append(fiducial_msg)
        return fiducials_list_msg


# Co-authored by Claude Code: clean exit on Ctrl-C / launch shutdown, as in hand_landmarks
def main(args=None):
    rclpy.init(args=args)
    node = FiducialsDetectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # rclpy's signal handler shuts the context down while spin is running. Depending on
        # timing a callback or the executor then fails with an RCLError ("context is invalid")
        # instead of ExternalShutdownException. Errors after the shutdown are just that race;
        # errors while the context is still valid are real and re-raised.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        # The signal handler may have shut the context down already, and a second
        # rclpy.shutdown() raises
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

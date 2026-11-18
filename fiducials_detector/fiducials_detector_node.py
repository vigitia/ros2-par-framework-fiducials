# Co-authored by Claude Code

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
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import Header

from .depth.corners_in_camera_space import compute_corners_in_camera_space
from .depth.depth_arrival_monitor import DepthArrivalMonitor
from .fiducials_detector import FiducialsDetector
from .tracking.background_full_scanner import BackgroundFullScanner
from .tracking.marker_tracker import MarkerTracker
from .utils.mono16_scaling import scale_mono16_to_mono8
from .utils.serialized_image import read_serialized_image_mono

NODE_NAME = 'fiducials_detector_node'
TOPIC_NAME_FIDUCIALS = 'fiducials'
FIDUCIALS_PUBLISHER_QUEUE_SIZE = 10

# Default topic names defined by the conventions of ROS 2 and the PAR framework
TOPIC_NAME_FRAME = 'image_raw'
# With use_compressed: JPEGs a camera driver passes through, so it doesn't have to decode them for this node
TOPIC_NAME_FRAME_COMPRESSED = TOPIC_NAME_FRAME + '/compressed'
TOPIC_NAME_FRAME_DEPTH = 'depth_registered/image_raw'
TOPIC_NAME_CAMERA_INFO = 'camera_info'
# With use_depth: how many colour and depth frames the synchronizer keeps to find pairs, and how
# far apart their stamps may be to count as a pair
FRAME_DEPTH_SYNC_QUEUE_SIZE = 10
FRAME_DEPTH_SYNC_SLOP_SEC = 0.01
# With use_depth, frames are only processed in pairs with a depth frame. Colour frames arriving
# without any depth for this long means no depth camera: nothing is published, which the node
# then says.
DEPTH_MISSING_TIMEOUT_SEC = 2.0
DEPTH_CHECK_PERIOD_SEC = 1.0

# Prevent spamming the console with warnings. Limits output of the error log to only once per this interval
WARNING_THROTTLE_PERIOD_SEC = 5.0

# Single-channel 16-bit frame encodings, e.g. infrared cameras
MONO16_ENCODINGS = ('mono16', '16UC1')

# REP 118: depth images in 16UC1 are in millimetres, 32FC1 in metres
M_PER_MM = 0.001

# Default QoS profile defined by the Framework for camera frames
FRAME_QOS = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                       history=QoSHistoryPolicy.KEEP_LAST, depth=1)

# Set marker type and dict
PARAMETER_MARKER_TYPE = 'marker_type'
PARAMETER_DICTIONARY = 'dictionary'

# OpenCV Marker Detection options
PARAMETER_CORNER_REFINEMENT = 'corner_refinement'
PARAMETER_ADAPTIVE_THRESH_CONSTANT = 'adaptive_thresh_constant'
PARAMETER_ADAPTIVE_THRESH_WIN_SIZE_MIN_PX = 'adaptive_thresh_win_size_min_px'
PARAMETER_ADAPTIVE_THRESH_WIN_SIZE_MAX_PX = 'adaptive_thresh_win_size_max_px'
PARAMETER_BIT_SAMPLE_PX_PER_CELL = 'bit_sample_px_per_cell'
PARAMETER_BIT_IGNORED_MARGIN_PER_CELL = 'bit_ignored_margin_per_cell'

# Settings for camera options
PARAMETER_USE_DEPTH = 'use_depth'
PARAMETER_USE_COMPRESSED = 'use_compressed'

PARAMETER_DEBUG = 'debug'

# Settings for Tracking Mode
PARAMETER_TRACKING_MODE = 'tracking_mode'
PARAMETER_TRACKING_WINDOW_SIZE_PX = 'tracking_window_size_px'
PARAMETER_FULL_SCAN_RATE_HZ = 'full_scan_rate_hz'
# With tracking, a marker not seen in its window for this long is dropped: it was covered, or
# moved faster than its window; the background full scan finds it again
TRACKING_MAX_UNSEEN_TIME_SEC = 0.5


class FiducialsDetectorNode(Node):

    # The colour camera's intrinsics, set once calibrated ones arrive (only with use_depth)
    camera_matrix = None
    distortion_coefficients = None

    # For tracking_mode
    marker_tracker = None
    background_full_scanner = None

    def __init__(self):
        super().__init__(node_name=NODE_NAME)

        self.cv_bridge = CvBridge()

        self._declare_parameters()

        self.fiducials_detector = self._create_fiducials_detector(self.get_parameter(PARAMETER_DEBUG).value)

        # Check if Tracking Mode is enabled
        if self.get_parameter(PARAMETER_TRACKING_MODE).value:
            self._init_tracking_mode()

        self._create_subscriptions()

        self.fiducials_publisher = self.create_publisher(
            msg_type=FiducialsList,
            topic=TOPIC_NAME_FIDUCIALS,
            qos_profile=FIDUCIALS_PUBLISHER_QUEUE_SIZE)

    # Descriptions written with the help of Claude Code
    def _declare_parameters(self):
        # All read-only. So far there has been no need to changes these values during runtime.
        self.declare_parameter(PARAMETER_MARKER_TYPE, 'aruco', ParameterDescriptor(
            description='Type of markers to detect: aruco or apriltag.',
            read_only=True))
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
        self.declare_parameter(PARAMETER_ADAPTIVE_THRESH_WIN_SIZE_MIN_PX, 3, ParameterDescriptor(
            description='Smallest adaptive threshold window in pixels (OpenCV default 3, at '
                        'least 3). The frame is thresholded and searched once per window, from '
                        'min to max in steps of 10.',
            read_only=True))
        self.declare_parameter(PARAMETER_ADAPTIVE_THRESH_WIN_SIZE_MAX_PX, 23, ParameterDescriptor(
            description='Largest adaptive threshold window in pixels (OpenCV default 23). '
                        'min = max thresholds once: faster, if that window suits the size of '
                        'the markers in the image.',
            read_only=True))
        self.declare_parameter(PARAMETER_BIT_SAMPLE_PX_PER_CELL, 4, ParameterDescriptor(
            description='Pixels per marker cell that a candidate square is resampled to before '
                        'its bits are read (OpenCV default 4). More samples average out sensor '
                        'noise on markers only a few camera pixels per cell wide.',
            read_only=True))
        self.declare_parameter(PARAMETER_BIT_IGNORED_MARGIN_PER_CELL, 0.13, ParameterDescriptor(
            description='Fraction of each cell ignored at its edges when reading the bits '
                        '(OpenCV default 0.13, below 0.5). A wider margin skips blurred cell '
                        'edges; too wide loses markers.',
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
        self.declare_parameter(PARAMETER_TRACKING_MODE, False, ParameterDescriptor(
            description='false: every frame is searched whole. true: search each frame only in '
                        f'square windows of {PARAMETER_TRACKING_WINDOW_SIZE_PX} around the known '
                        'markers, shifted by their last movement, and search whole frames in a '
                        f'background thread at {PARAMETER_FULL_SCAN_RATE_HZ} to find new or lost '
                        'markers. Much faster per frame; a new marker appears only after the next '
                        'whole-frame search.',
            read_only=True))
        self.declare_parameter(PARAMETER_TRACKING_WINDOW_SIZE_PX, 200, ParameterDescriptor(
            description=f'With {PARAMETER_TRACKING_MODE}: side length of the square search '
                        'window around each known marker. It must hold the marker plus its '
                        'movement between two processed frames.',
            read_only=True))
        self.declare_parameter(PARAMETER_FULL_SCAN_RATE_HZ, 5.0, ParameterDescriptor(
            description=f'With {PARAMETER_TRACKING_MODE}: how often per second whole '
                        f'frames are searched in the background. Higher finds new markers '
                        f'sooner and takes more CPU from the tracking.',
            read_only=True))
        self.declare_parameter(PARAMETER_DEBUG, False, ParameterDescriptor(
            description='Show the frames with the detected markers in an OpenCV window (needs a '
                        'display) and print the detection accuracy. Slows detection down a '
                        'little, so only for testing.',
            read_only=True))

    def _create_fiducials_detector(self, debug):
        """Return a FiducialsDetector instance with the detector settings from the node's parameters."""
        fiducials_detector = FiducialsDetector(
            marker_type=self.get_parameter(PARAMETER_MARKER_TYPE).value,
            dictionary_name=self.get_parameter(PARAMETER_DICTIONARY).value,
            corner_refinement=self.get_parameter(PARAMETER_CORNER_REFINEMENT).value,
            adaptive_thresh_constant=self.get_parameter(
                PARAMETER_ADAPTIVE_THRESH_CONSTANT).value,
            adaptive_thresh_win_size_min_px=self.get_parameter(
                PARAMETER_ADAPTIVE_THRESH_WIN_SIZE_MIN_PX).value,
            adaptive_thresh_win_size_max_px=self.get_parameter(
                PARAMETER_ADAPTIVE_THRESH_WIN_SIZE_MAX_PX).value,
            bit_sample_px_per_cell=self.get_parameter(PARAMETER_BIT_SAMPLE_PX_PER_CELL).value,
            bit_ignored_margin_per_cell=self.get_parameter(
                PARAMETER_BIT_IGNORED_MARGIN_PER_CELL).value,
            debug=debug)
        return fiducials_detector

    # Written with the help of Claude Code
    def _init_tracking_mode(self):
        """With tracking_mode, init the marker tracker and the background full scanner."""

        # Read parameters for Tracking Mode
        search_window_size_px = self.get_parameter(PARAMETER_TRACKING_WINDOW_SIZE_PX).value
        if search_window_size_px <= 0:
            raise ValueError(f'{PARAMETER_TRACKING_WINDOW_SIZE_PX} must be positive, got '
                             f'{search_window_size_px}')
        full_scan_rate_hz = self.get_parameter(PARAMETER_FULL_SCAN_RATE_HZ).value
        if full_scan_rate_hz <= 0.0:
            raise ValueError(f'{PARAMETER_FULL_SCAN_RATE_HZ} must be positive, got '
                             f'{full_scan_rate_hz}')

        # Init MarkerTracker and BackgroundFullScanner
        self.marker_tracker = MarkerTracker(search_window_size_px, TRACKING_MAX_UNSEEN_TIME_SEC)
        # OpenCV runs one thread for the whole process. The windows are too small to gain from
        # OpenCV's thread pool, which then costs about twice the CPU time of a single thread, and
        # the full scan runs in its own thread anyway, where a slower scan costs no latency
        cv2.setNumThreads(1)
        # Its own detector without debug: one OpenCV detector must not run in two threads, and
        # the preview window belongs to the main thread
        full_scan_detector = self._create_fiducials_detector(debug=False)
        self.background_full_scanner = BackgroundFullScanner(full_scan_detector,
                                                             full_scan_rate_hz)

    # Written with the help of Claude Code
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
        if frame_msg_type is Image:
            # Serialized: turning a full camera frame into a Python message costs more CPU than
            # detecting the markers in it, see _on_serialized_frame_received
            self.create_subscription(msg_type=Image,
                                     topic=topic_name_frame,
                                     callback=self._on_serialized_frame_received,
                                     qos_profile=FRAME_QOS,
                                     raw=True)
            return
        self.create_subscription(msg_type=frame_msg_type,
                                 topic=topic_name_frame,
                                 callback=self._detect_and_publish_fiducials_in_frame_msg,
                                 qos_profile=FRAME_QOS)

    # Written by Claude Code
    def _create_frame_and_depth_subscriptions(self, frame_msg_type, topic_name_frame):
        """Process colour and depth frames in pairs, and warn when no depth arrives."""
        frame_subscriber = Subscriber(self, frame_msg_type, topic_name_frame,
                                      qos_profile=FRAME_QOS)
        depth_subscriber = Subscriber(self, Image, TOPIC_NAME_FRAME_DEPTH, qos_profile=FRAME_QOS)
        self.frame_depth_synchronizer = ApproximateTimeSynchronizer(
            [frame_subscriber, depth_subscriber], FRAME_DEPTH_SYNC_QUEUE_SIZE,
            FRAME_DEPTH_SYNC_SLOP_SEC)
        self.frame_depth_synchronizer.registerCallback(self._on_frame_and_depth_pair_received)

        # Each subscriber also reports its own messages, so a missing depth camera shows up even
        # though the synchronizer then never forms a pair
        self.depth_arrival_monitor = DepthArrivalMonitor(DEPTH_MISSING_TIMEOUT_SEC)
        frame_subscriber.registerCallback(self._record_frame_arrival_time)
        depth_subscriber.registerCallback(self._record_depth_arrival_time)
        self.create_timer(DEPTH_CHECK_PERIOD_SEC, self._warn_if_depth_missing)
        # The name after remapping, which is what the warnings must name
        self.depth_topic_name = depth_subscriber.sub.topic_name

        self.create_subscription(
            msg_type=CameraInfo,
            topic=TOPIC_NAME_CAMERA_INFO,
            callback=self._on_camera_info_received,
            qos_profile=FRAME_QOS)

    # Written with the help of Claude Code
    def _on_camera_info_received(self, camera_info_msg):
        # intrinsics stored with set_camera_info while the camera runs take effect with the next message.

        # Uncalibrated, the camera matrix is all zeros: skipped, the last valid intrinsics stay
        if camera_info_msg.k[0] == 0.0:
            return

        if self.camera_matrix is None:
            self.get_logger().info(f'Calibrated camera info received on '
                                   f'{TOPIC_NAME_CAMERA_INFO}, corners_3d are filled from now on')

        # Save new matrix and distortion coefficients in class variables
        self.camera_matrix = np.reshape(camera_info_msg.k, (3, 3))
        self.distortion_coefficients = np.array(camera_info_msg.d)

    # Written by Claude Code
    def _record_frame_arrival_time(self, frame_msg):
        self.depth_arrival_monitor.on_frame(time.monotonic())

    # Written by Claude Code
    def _record_depth_arrival_time(self, depth_msg):
        self.depth_arrival_monitor.on_depth(time.monotonic())

    # Written by Claude Code
    def _warn_if_depth_missing(self):
        if self.depth_arrival_monitor.is_depth_missing(time.monotonic()):
            self.get_logger().warning(
                f'{PARAMETER_USE_DEPTH} is true, but no depth frames arrive on '
                f'{self.depth_topic_name}, so no fiducials are published. Connect a depth camera '
                f'or set {PARAMETER_USE_DEPTH} to false.',
                throttle_duration_sec=WARNING_THROTTLE_PERIOD_SEC)

    # Written by Claude Code
    def _on_frame_and_depth_pair_received(self, frame_msg, depth_msg):
        depth_image_m = self.cv_bridge.imgmsg_to_cv2(depth_msg)
        if depth_msg.encoding == '16UC1':
            depth_image_m = depth_image_m.astype(np.float32) * M_PER_MM
        self._detect_and_publish_fiducials_in_frame_msg(frame_msg, depth_image_m)

    # Written by Claude Code
    def _on_serialized_frame_received(self, serialized_frame):
        """
        Detect the markers in a frame read straight from its serialized message.

        The pixels are read in place (read_serialized_image_mono), without building a Python
        Image message. Frames it does not handle (e.g. other encodings) are converted the usual
        way.
        """
        image_fields = read_serialized_image_mono(serialized_frame)
        if image_fields is None:
            frame_msg = deserialize_message(serialized_frame, Image)
            self._detect_and_publish_fiducials_in_frame_msg(frame_msg)
            return
        stamp_sec, stamp_nanosec, frame_id, frame_mono = image_fields
        frame_header = Header(frame_id=frame_id)
        frame_header.stamp.sec = stamp_sec
        frame_header.stamp.nanosec = stamp_nanosec
        self._detect_and_publish_fiducials(frame_header, frame_mono)

    # Co-authored by Claude Code
    def _detect_and_publish_fiducials_in_frame_msg(self, frame_msg, depth_image_m=None):
        """Detect the markers in a raw or compressed frame, with optional depth (metres) for corners_3d."""
        frame_mono = self._decode_frame_mono(frame_msg)
        if frame_mono is None:
            return
        self._detect_and_publish_fiducials(frame_msg.header, frame_mono, depth_image_m)

    # Co-authored by Claude Code:
    def _detect_and_publish_fiducials(self, frame_header, frame_mono, depth_image_m=None):
        """Detect the markers in a greyscale frame and publish them with the frame's header."""
        if depth_image_m is not None:
            depth_image_m = self._get_depth_image_on_frame_grid(depth_image_m, frame_mono)

        if self.marker_tracker is not None:
            fiducials = self._detect_fiducials_in_tracking_mode(frame_mono, frame_header.stamp)
        else:
            fiducials = self.fiducials_detector.detect_fiducials(frame_mono)
        if depth_image_m is not None:
            self._add_corners_3d(fiducials, depth_image_m)

        # Published for every frame, also without markers: an empty list is how subscribers learn
        # that a marker disappeared, otherwise its last position would stay valid for them forever
        fiducials_list_msg = self._create_fiducials_list_msg(fiducials, frame_header)
        self.fiducials_publisher.publish(fiducials_list_msg)

    # Written by Claude Code
    def _detect_fiducials_in_tracking_mode(self, frame_mono, frame_stamp):
        """Search only around known markers; whole frames go to the background scanner."""
        # The camera's stamp, not the arrival time: velocities stay right when frames queue up
        frame_time_sec = frame_stamp.sec + frame_stamp.nanosec * 1e-9
        self.background_full_scanner.set_newest_frame(frame_mono, frame_time_sec)
        full_scan_result = self.background_full_scanner.get_new_result()
        if full_scan_result is not None:
            scan_fiducials, scan_time_sec = full_scan_result
            self.marker_tracker.add_untracked_fiducials(scan_fiducials, scan_time_sec)
        frame_height_px, frame_width_px = frame_mono.shape[:2]
        search_windows_px = self.marker_tracker.compute_search_windows_px(
            frame_time_sec, frame_width_px, frame_height_px)
        fiducials = self.fiducials_detector.detect_fiducials_in_windows(frame_mono,
                                                                        search_windows_px)
        self.marker_tracker.update_from_window_detections(fiducials, frame_time_sec)
        return fiducials

    # Written with the help of Claude Code
    def _decode_frame_mono(self, frame_msg):
        """Return the frame in greyscale, or None if a compressed frame can't be decoded."""

        if not isinstance(frame_msg, CompressedImage):
            # 16-bit greyscale (e.g. infrared) is scaled by its own range, as cv_bridge's fixed
            # conversion would leave it almost black
            if frame_msg.encoding in MONO16_ENCODINGS:
                frame_mono16 = self.cv_bridge.imgmsg_to_cv2(frame_msg)
                return scale_mono16_to_mono8(frame_mono16)
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
        if self.camera_matrix is None:
            return
        for detected_marker in fiducials:
            detected_marker['corners_3d'] = compute_corners_in_camera_space(
                detected_marker['corners'], depth_image_m, self.camera_matrix,
                self.distortion_coefficients)

    @staticmethod
    def _create_fiducials_list_msg(detected_markers, frame_header):
        fiducials_list_msg = FiducialsList(header=frame_header)
        for detected_marker in detected_markers:
            corners_px = detected_marker['corners'].tolist()
            centroid_px = detected_marker['centroid'].tolist()
            fiducial_msg = Fiducial(id=int(detected_marker['id']),
                                    angle=detected_marker['angle'],
                                    x0=corners_px[0][0], y0=corners_px[0][1],
                                    x1=corners_px[1][0], y1=corners_px[1][1],
                                    x2=corners_px[2][0], y2=corners_px[2][1],
                                    x3=corners_px[3][0], y3=corners_px[3][1],
                                    x_centroid=centroid_px[0], y_centroid=centroid_px[1])
            # Add depth data if available
            if detected_marker.get('corners_3d') is not None:
                fiducial_msg.corners_3d = [Point(x=float(x_m), y=float(y_m), z=float(z_m))
                                           for x_m, y_m, z_m in detected_marker['corners_3d']]
            fiducials_list_msg.fiducials.append(fiducial_msg)
        return fiducials_list_msg

    def destroy_node(self):
        # End the Background scanner first because it runs in a separate thread
        if self.background_full_scanner is not None:
            self.background_full_scanner.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FiducialsDetectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

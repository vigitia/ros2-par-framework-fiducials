
# Compatibility with the camera drivers (topic, best-effort QoS, colour encoding) and the
# parameters for the marker type, dictionary and detector settings added with help of Claude Code.
# Claude Code also added the optional depth input (use_depth) that fills corners_3d.

import os
import time

from geometry_msgs.msg import Point
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from sensor_msgs.msg import CameraInfo, Image

from cv_bridge import CvBridge

from par_framework_interfaces.msg import FiducialsList, Fiducial


from .fiducials_detector import corners_to_camera_space, FiducialsDetector

NODE_NAME = 'fiducials_detector_node'
TOPIC_NAME_FIDUCIALS = 'fiducials'
QUEUE_SIZE = 10
# Following lines until FRAME_QOS written by Claude Code
# The PAR framework's camera topics, onto which its launch files remap every camera driver: the
# colour image, depth registered to it (from a depth camera), and the colour
# camera's intrinsics. Another topic is chosen by remapping.
TOPIC_NAME_FRAME = '/camera/image_raw'
TOPIC_NAME_FRAME_DEPTH = '/camera/depth_registered/image_raw'
TOPIC_NAME_CAMERA_INFO = '/camera/camera_info'
SYNC_LEEWAY_TIME_SEC = 0.01
# With use_depth, frames are only processed in pairs with a depth frame. Colour frames arriving
# without any depth for this long means no depth camera: nothing is published, which the node
# then says.
DEPTH_MISSING_SEC = 2.0

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


class FiducialsDetectorNode(Node):
    # Co-authored by Claude Code
    def __init__(self):
        super().__init__(node_name=NODE_NAME)
        self.get_logger().info('Hello from FiducialsDetectorNode')
        self.get_logger().info(os.getcwd())

        self._declare_parameters()

        self.cv_bridge = CvBridge()

        self.fiducials_detector = FiducialsDetector(
            self,
            marker_type=self.get_parameter(PARAMETER_MARKER_TYPE).value,
            dictionary=self.get_parameter(PARAMETER_DICTIONARY).value,
            corner_refinement=self.get_parameter(PARAMETER_CORNER_REFINEMENT).value,
            adaptive_thresh_constant=self.get_parameter(PARAMETER_ADAPTIVE_THRESH_CONSTANT).value)

        self.camera_info = None
        if self.get_parameter(PARAMETER_USE_DEPTH).value:
            frame_subscriber = Subscriber(self, Image, TOPIC_NAME_FRAME, qos_profile=FRAME_QOS)
            depth_subscriber = Subscriber(self, Image, TOPIC_NAME_FRAME_DEPTH,
                                          qos_profile=FRAME_QOS)
            self.synchronizer = ApproximateTimeSynchronizer([frame_subscriber, depth_subscriber],
                                                            QUEUE_SIZE, SYNC_LEEWAY_TIME_SEC)
            self.synchronizer.registerCallback(self.listener_callback_with_depth)
            # Each subscriber also reports its own messages, so a missing depth camera shows up
            # even though the synchronizer then never forms a pair
            self.first_frame_time = self.last_frame_time = self.last_depth_time = None
            # The name after remapping, which is what the warning must name
            self.depth_topic = depth_subscriber.sub.topic_name
            frame_subscriber.registerCallback(self.on_frame_arrived)
            depth_subscriber.registerCallback(self.on_depth_arrived)
            self.create_timer(1.0, self.check_depth_arrives)
            self.create_subscription(CameraInfo, TOPIC_NAME_CAMERA_INFO, self.on_camera_info,
                                     FRAME_QOS)
        else:
            self.create_subscription(msg_type=Image,
                                     topic=TOPIC_NAME_FRAME,
                                     callback=self.listener_callback,
                                     qos_profile=FRAME_QOS)

        self.publisher = self.create_publisher(msg_type=FiducialsList,
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

    # Written by Claude Code
    def on_camera_info(self, msg):
        self.camera_info = msg

    # Written by Claude Code
    def on_frame_arrived(self, msg):
        self.last_frame_time = time.monotonic()
        if self.first_frame_time is None:
            self.first_frame_time = self.last_frame_time

    # Written by Claude Code
    def on_depth_arrived(self, msg):
        self.last_depth_time = time.monotonic()

    # Written by Claude Code
    def check_depth_arrives(self):
        now = time.monotonic()
        # Not before DEPTH_MISSING_SEC of colour frames: a driver may only start computing
        # registered depth once it sees a subscriber
        frames_arrive = (self.last_frame_time is not None
                         and now - self.last_frame_time < DEPTH_MISSING_SEC
                         and now - self.first_frame_time > DEPTH_MISSING_SEC)
        depth_missing = (self.last_depth_time is None
                         or now - self.last_depth_time > DEPTH_MISSING_SEC)
        if frames_arrive and depth_missing:
            self.get_logger().warning(
                f'[FiducialsDetectorNode]: use_depth is true, but no depth frames arrive on '
                f'{self.depth_topic}, so no fiducials are published. Connect a depth camera or '
                f'set use_depth to false.', throttle_duration_sec=5.0)

    # Written by Claude Code
    def listener_callback_with_depth(self, msg, msg_depth):
        depth = self.cv_bridge.imgmsg_to_cv2(msg_depth)
        # REP 118: 16UC1 is in millimetres, 32FC1 in metres
        if msg_depth.encoding == '16UC1':
            depth = depth.astype(np.float32) * 0.001
        # A depth image on another pixel grid (e.g. /depth/image_raw) would give every marker the
        # depth of some other point
        if depth.shape[:2] != (msg.height, msg.width):
            self.get_logger().warning(
                f'[FiducialsDetectorNode]: depth image is {depth.shape[1]}x{depth.shape[0]}, the '
                f'colour image {msg.width}x{msg.height}: {self.depth_topic} must be registered to '
                f'the colour image, no 3D positions', throttle_duration_sec=5.0)
            depth = None
        self.listener_callback(msg, depth)

    # Co-authored by Claude Code: optional depth image (in metres, registered to data) for
    # corners_3d
    def listener_callback(self, data, depth=None):
        # Detection works on greyscale. Asking cv_bridge for mono8 converts bgr8, rgb8 and mono8
        # frames correctly, whichever encoding the camera node is set to
        current_frame = self.cv_bridge.imgmsg_to_cv2(data, desired_encoding='mono8')
        # self.get_logger().info(str(current_frame.shape))

        fiducials: list = self.fiducials_detector.detect_fiducials(current_frame)
        # Following lines co-authored by Claude Code: 3D corners, once the intrinsics are known
        if depth is not None and self.camera_info is not None and self.camera_info.k[0] != 0.0:
            k = np.reshape(self.camera_info.k, (3, 3))
            d = np.array(self.camera_info.d)
            for marker in fiducials:
                marker['corners_3d'] = corners_to_camera_space(marker['corners'], depth, k, d)

        # Published for every frame, also without markers: an empty list is how subscribers learn
        # that a marker disappeared, otherwise its last position would stay valid for them forever
        fiducials_interface_message = self.convert_to_fiducials_interface_msg(fiducials,
                                                                              data.header)
        self.publisher.publish(fiducials_interface_message)

    @staticmethod
    def convert_to_fiducials_interface_msg(fiducials_list, header=None):
        fiducials_interface_message = FiducialsList()
        if header is not None:
            fiducials_interface_message.header = header

        for marker in fiducials_list:
            fiducial_marker = Fiducial()
            fiducial_marker.id = int(marker['id'])
            fiducial_marker.angle = marker['angle']

            fiducial_marker.x0 = float(marker['corners'][0][0])
            fiducial_marker.y0 = float(marker['corners'][0][1])
            fiducial_marker.x1 = float(marker['corners'][1][0])
            fiducial_marker.y1 = float(marker['corners'][1][1])
            fiducial_marker.x2 = float(marker['corners'][2][0])
            fiducial_marker.y2 = float(marker['corners'][2][1])
            fiducial_marker.x3 = float(marker['corners'][3][0])
            fiducial_marker.y3 = float(marker['corners'][3][1])

            fiducial_marker.x_centroid = float(marker['centroid'][0])
            fiducial_marker.y_centroid = float(marker['centroid'][1])
            # Following lines co-authored by Claude Code: only with use_depth and enough depth
            if marker.get('corners_3d') is not None:
                fiducial_marker.corners_3d = [Point(x=float(x), y=float(y), z=float(z))
                                              for x, y, z in marker['corners_3d']]

            fiducials_interface_message.fiducials.append(fiducial_marker)

        return fiducials_interface_message

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

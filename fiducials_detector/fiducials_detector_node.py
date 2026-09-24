
# Compatibility with the Logitech Brio node (topic, best-effort QoS, colour encoding) and the
# parameters for the marker type, dictionary and detector settings added with help of Claude Code

import os

from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from sensor_msgs.msg import Image

from cv_bridge import CvBridge

from custom_interfaces.msg import FiducialsList, Fiducial


from .fiducials_detector import FiducialsDetector

NODE_NAME = 'fiducials_detector_node'
TOPIC_NAME_FIDUCIALS = 'fiducials'
QUEUE_SIZE = 10

# Camera frames are published best-effort (see logitech_brio). A reliable subscriber would be
# QoS-incompatible and silently receive nothing, while a best-effort one also connects to reliable
# publishers. Only the newest frame matters, detecting on an old one would publish outdated marker
# positions.
FRAME_QOS = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                       history=QoSHistoryPolicy.KEEP_LAST, depth=1)

PARAMETER_TOPIC_NAME_FRAME = 'topic_name_frame'
PARAMETER_MARKER_TYPE = 'marker_type'
PARAMETER_DICTIONARY = 'dictionary'
PARAMETER_CORNER_REFINEMENT = 'corner_refinement'
PARAMETER_ADAPTIVE_THRESH_CONSTANT = 'adaptive_thresh_constant'


class FiducialsDetectorNode(Node):
    # Co-authored by Claude Code
    def __init__(self):
        super().__init__(node_name=NODE_NAME)
        self.get_logger().info('Hello from FiducialsDetectorNode')
        self.get_logger().info(os.getcwd())

        self.__declare_parameters()

        self.cv_bridge = CvBridge()

        self.fiducials_detector = FiducialsDetector(
            self,
            marker_type=self.get_parameter(PARAMETER_MARKER_TYPE).value,
            dictionary=self.get_parameter(PARAMETER_DICTIONARY).value,
            corner_refinement=self.get_parameter(PARAMETER_CORNER_REFINEMENT).value,
            adaptive_thresh_constant=self.get_parameter(PARAMETER_ADAPTIVE_THRESH_CONSTANT).value)

        self.create_subscription(msg_type=Image,
                                 topic=self.get_parameter(PARAMETER_TOPIC_NAME_FRAME).value,
                                 callback=self.listener_callback,
                                 qos_profile=FRAME_QOS)

        self.publisher = self.create_publisher(msg_type=FiducialsList,
                                               topic=TOPIC_NAME_FIDUCIALS,
                                               qos_profile=QUEUE_SIZE)

    # Written with the help of Claude Code
    def __declare_parameters(self):
        # All read-only because the subscription and the detector are created once at startup, a
        # later change would have no effect
        self.declare_parameter(
            PARAMETER_TOPIC_NAME_FRAME, '/camera/brio/image_raw', ParameterDescriptor(
                description='Image topic to detect fiducials in.', read_only=True))
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

    # Co-authored by Claude Code
    def listener_callback(self, data):
        # Detection works on greyscale. Asking cv_bridge for mono8 converts bgr8, rgb8 and mono8
        # frames correctly, whichever encoding the camera node is set to
        current_frame = self.cv_bridge.imgmsg_to_cv2(data, desired_encoding='mono8')
        # self.get_logger().info(str(current_frame.shape))

        fiducials: list = self.fiducials_detector.detect_fiducials(current_frame)

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

            fiducials_interface_message.fiducials.append(fiducial_marker)

        return fiducials_interface_message

def main(args=None):
    rclpy.init(args=args)
    node = FiducialsDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

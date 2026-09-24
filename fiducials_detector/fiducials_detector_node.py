
import os

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

from cv_bridge import CvBridge

from custom_interfaces.msg import FiducialsList, Fiducial


from .fiducials_detector import FiducialsDetector

NODE_NAME = 'fiducials_detector_node'
TOPIC_NAME_FRAME_FULL = '/camera/generic_webcam_frame_full'
TOPIC_NAME_FIDUCIALS = 'fiducials'
QUEUE_SIZE = 10


class FiducialsDetectorNode(Node):
    def __init__(self):
        super().__init__(node_name=NODE_NAME)
        self.get_logger().info('Hello from FiducialsDetectorNode')
        self.get_logger().info(os.getcwd())

        self.cv_bridge = CvBridge()

        self.create_subscription(msg_type=Image,
                                 topic=TOPIC_NAME_FRAME_FULL,
                                 callback=self.listener_callback,
                                 qos_profile=QUEUE_SIZE)

        self.publisher = self.create_publisher(msg_type=FiducialsList,
                                               topic=TOPIC_NAME_FIDUCIALS,
                                               qos_profile=QUEUE_SIZE)

        self.fiducials_detector = FiducialsDetector(self)

    def listener_callback(self, data):
        current_frame = self.cv_bridge.imgmsg_to_cv2(data)  # Convert ROS Image message to OpenCV image
        # self.get_logger().info(str(current_frame.shape))

        fiducials: list = self.fiducials_detector.detect_fiducials(current_frame)

        if len(fiducials) > 0:
            fiducials_interface_message = self.convert_to_fiducials_interface_msg(fiducials, data.header)
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

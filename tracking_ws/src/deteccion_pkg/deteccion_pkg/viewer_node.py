import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ViewerNode(Node):
    def __init__(self):
        super().__init__('viewer_node')
        self.subscription = self.create_subscription(
            Image,
            'video_feed',
            self.listener_callback,
            10
        )
        self.bridge = CvBridge()
        self.window_name = 'Video en Vivo'
        cv2.namedWindow(self.window_name)

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            cv2.imshow(self.window_name, cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Error al procesar la imagen: {e}')

def main(args=None):
    rclpy.init(args=args)
    viewer_node = ViewerNode()
    rclpy.spin(viewer_node)
    viewer_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
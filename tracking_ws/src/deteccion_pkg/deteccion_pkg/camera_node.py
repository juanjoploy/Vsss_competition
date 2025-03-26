import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.publisher_ = self.create_publisher(Image, 'video_feed', 10)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(2)  # Usando /dev/video2 (Orbbec Astra+)

        if not self.cap.isOpened():
            self.get_logger().error('No se pudo abrir la cámara Orbbec Astra+.')
            exit()

        self.timer = self.create_timer(0.033, self.publish_frame)  # ~30 FPS

    def publish_frame(self):
        ret, frame = self.cap.read()
        if ret:
            image_message = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(image_message)
        else:
            self.get_logger().warn('No se pudo leer un fotograma de la Orbbec Astra+.')

def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    rclpy.spin(camera_node)
    camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
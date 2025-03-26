import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point  # Publicaremos coordenadas (x, y)
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class TrajectoryTracker(Node):
    def __init__(self):
        super().__init__('trajectory_tracker')

        # Suscripción al video
        self.subscription = self.create_subscription(
            Image, 'video_feed', self.listener_callback, 10)

        # Publicador de coordenadas
        self.publisher = self.create_publisher(Point, 'trajectory_data', 10)  

        self.bridge = CvBridge()
        self.previous_center = None
        self.trajectory_image = None  
        self.total_distance_pixels = 0  
        self.pixel_to_cm = 0.1  

        # Mejor ajuste de los valores HSV para detectar rojo
        self.lower_red1 = np.array([0, 120, 70])   # Primer rango de rojo
        self.upper_red1 = np.array([10, 255, 255])

        self.lower_red2 = np.array([170, 120, 70])  # Segundo rango de rojo
        self.upper_red2 = np.array([180, 255, 255])

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            if self.trajectory_image is None:
                self.trajectory_image = np.zeros_like(cv_image)

            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # Detección del color rojo en dos rangos
            mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
            mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
            fullmask = cv2.bitwise_or(mask1, mask2)

            # Mejorar la detección con filtros morfológicos
            kernel = np.ones((5, 5), np.uint8)
            fullmask = cv2.morphologyEx(fullmask, cv2.MORPH_OPEN, kernel)  
            fullmask = cv2.morphologyEx(fullmask, cv2.MORPH_CLOSE, kernel) 

            # Encontrar contornos y seleccionar el más grande
            contours, _ = cv2.findContours(fullmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])

                    cv2.circle(cv_image, (cX, cY), 5, (255, 0, 0), -1)  # Azul para el centroide

                    if self.previous_center is not None:
                        dist = math.sqrt((cX - self.previous_center[0]) ** 2 + (cY - self.previous_center[1]) ** 2)
                        self.total_distance_pixels += dist
                        cv2.line(self.trajectory_image, self.previous_center, (cX, cY), (0, 255, 0), 2)

                    self.previous_center = (cX, cY)

                    overlay = cv2.addWeighted(cv_image, 0.7, self.trajectory_image, 0.3, 0)
                    total_distance_cm = self.total_distance_pixels * self.pixel_to_cm
                    cv2.putText(overlay, f'Distancia: {total_distance_cm:.2f} cm', (20, 50),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

                    cv2.imshow('Seguimiento de Trayectoria', overlay)  
                    cv2.waitKey(1)

                    # **Publicar coordenadas**
                    point_msg = Point()
                    point_msg.x = float(cX)
                    point_msg.y = float(cY)
                    point_msg.z = 0.0  # No usamos z
                    self.publisher.publish(point_msg)  

                    self.get_logger().info(f'📡 Publicando coordenada: ({cX}, {cY})')

        except Exception as e:
            self.get_logger().error(f'Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    trajectory_tracker = TrajectoryTracker()
    rclpy.spin(trajectory_tracker)
    trajectory_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

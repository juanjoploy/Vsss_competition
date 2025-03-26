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
        self.pixel_to_cm = None  # Se calculará dinámicamente

        # Parámetros para la detección del color rojo
        self.lower_red1 = np.array([0, 120, 70])   # Primer rango de rojo
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([170, 120, 70])  # Segundo rango de rojo
        self.upper_red2 = np.array([180, 255, 255])

        # Parámetros para la conversión píxeles -> cm
        self.camera_distance_cm = 43  # Distancia de la cámara al objeto (ajustable)
        self.object_real_size_cm = 11 # Tamaño real del objeto en cm (ajustable)

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            cv_image = cv2.resize(cv_image, (640,480))

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
                x, y, w, h = cv2.boundingRect(largest_contour)

                # **Calcular la conversión píxeles → cm**
                if self.pixel_to_cm is None:
                    self.pixel_to_cm = self.object_real_size_cm / w  # Se toma el ancho del objeto

                object_size_cm = w * self.pixel_to_cm  # Convertir ancho a cm
                self.get_logger().info(f" Tamaño del objeto en cm: {object_size_cm:.2f}")

                # **Cálculo de la distancia recorrida**
                cX, cY = x + w // 2, y + h // 2  # Centro del objeto
                if self.previous_center is not None:
                    distance_pixels = math.sqrt((cX - self.previous_center[0])**2 + (cY - self.previous_center[1])**2)
                    self.total_distance_pixels += distance_pixels
                    total_distance_cm = self.total_distance_pixels * self.pixel_to_cm  # Conversión
                    self.get_logger().info(f" Distancia recorrida en cm: {total_distance_cm:.2f}")

                    # Dibujar la trayectoria en la imagen
                    cv2.line(self.trajectory_image, self.previous_center, (cX, cY), (0, 255, 0), 2)

                self.previous_center = (cX, cY)

                # Superponer imagen con trayectoria
                overlay = cv2.addWeighted(cv_image, 0.7, self.trajectory_image, 0.3, 0)
                cv2.putText(overlay, f'Distancia: {total_distance_cm:.2f} cm', (20, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

                # Dibujar rectángulo alrededor del objeto
                cv2.rectangle(overlay, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(overlay, (cX, cY), 5, (255, 0, 0), -1)

                # Mostrar imagen con detección y trayectoria
                cv2.imshow('Seguimiento de Trayectoria', overlay)  
                cv2.waitKey(1)

                # **Publicar coordenadas**
                point_msg = Point()
                point_msg.x = float(cX)
                point_msg.y = float(cY)
                point_msg.z = 0.0  # No usamos z
                self.publisher.publish(point_msg)  

                self.get_logger().info(f' Publicando coordenada: ({cX}, {cY})')

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

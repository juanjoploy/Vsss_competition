import os
import rclpy
from rclpy.node import Node
import subprocess

class DisplayURDF(Node):
    def __init__(self):
        super().__init__('display_urdf')
        self.declare_parameter('urdf_file', 'urdf/caja_con_ruedas.urdf')
        urdf_file = self.get_parameter('urdf_file').get_parameter_value().string_value

        # Obtener la ruta completa del URDF
        urdf_path = os.path.join(
            os.path.dirname(__file__), '..', urdf_file
        )

        # Ejecutar robot_state_publisher
        self.get_logger().info(f"Cargando modelo URDF desde: {urdf_path}")
        subprocess.Popen(["ros2", "run", "robot_state_publisher", "robot_state_publisher", urdf_path])

        # Ejecutar Rviz
        subprocess.Popen(["ros2", "run", "rviz2", "rviz2"])

def main(args=None):
    rclpy.init(args=args)
    node = DisplayURDF()
    rclpy.spin(node)
    rclpy.shutdown()

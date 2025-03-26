import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt

class TrajectoryPlotter(Node):
    def __init__(self):
        super().__init__('trajectory_plotter_node')

        # Suscripción a los datos de trayectoria
        self.subscription = self.create_subscription(
            Point,  # Cambio de Float32MultiArray a Point
            'trajectory_data',
            self.listener_callback,
            10)

        self.trajectory_x = []
        self.trajectory_y = []

        # Configurar la gráfica
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_title("Trayectoria del Robot")
        self.line, = self.ax.plot([], [], 'g-', label="Trayectoria")
        plt.legend()
        plt.ion()  # Modo interactivo para actualizar en tiempo real
        plt.show()

    def listener_callback(self, msg):
        # Recibe los valores de x e y desde el mensaje Point
        x, y = msg.x, msg.y
        self.trajectory_x.append(x)
        self.trajectory_y.append(y)

        # Mostrar en consola las coordenadas recibidas
        self.get_logger().info(f"📍 Recibido punto: ({x}, {y})")

        self.update_plot()

    def update_plot(self):
        # Actualizar datos en la gráfica
        self.line.set_xdata(self.trajectory_x)
        self.line.set_ydata(self.trajectory_y)
        self.ax.relim()
        self.ax.autoscale_view()
        plt.draw()
        plt.pause(0.01)  # Pequeña pausa para permitir actualización en tiempo real

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlotter()
    rclpy.spin(node)  # Mantener el nodo en ejecución
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

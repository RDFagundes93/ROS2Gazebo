import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math

class OdometryPrinter(Node):
    def __init__(self):
        super().__init__('odometry_printer')
        # Assinatura no tópico /odom para receber mensagens Odometry
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.subscription  # evitar warning de variável não usada

    def odom_callback(self, msg):
        # Extrair posição x e y
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Extrair orientação em quaternion e converter para yaw (rotação em torno do eixo z)
        q = msg.pose.pose.orientation
        yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

        self.get_logger().info(f'Posição do robô: x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.1f}°')

    def quaternion_to_yaw(self, x, y, z, w):
        # Conversão quaternion para yaw (ângulo em radianos)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

def main(args=None):
    rclpy.init(args=args)
    node = OdometryPrinter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


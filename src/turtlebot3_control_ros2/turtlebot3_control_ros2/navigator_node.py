import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.forward_paths = [
            [  # 1º objetivo
                (-2.0, 2.0),
                (0.36, 1.84),
                (0.57, 0.91),
                (1.75, 0.98),
                (2.0, 2.0),
            ],
            [  # 2º objetivo
                (-2.0, 2.0),
                (0.42, 1.84),
                (1.37, -0.18),
                (2.10, -1.89),
            ],
            [  # 3º objetivo
                (-2.0, 2.0),
                (-0.55, 1.92),
                (-1.0, -0.09),
                (-1.0, -2.0), 
                (-2.16, -2.15), 
            ],
            [  # 4º objetivo
                (-2.0, 2.0),
                (-0.55, 1.92),
                (-0.55, -0.08),
                (-2.0, -0.08), 
                (-1.90, 1.20),
            ],             
        ]

        self.current_path_index = 0
        self.returning = False
        self.load_current_path()

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.tolerance = 0.25
        self.linear_speed = 0.2
        self.angular_speed = 0.5

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Waypoint navigator started.')

    def load_current_path(self):
        self.waypoints = self.forward_paths[self.current_path_index]
        self.return_path = list(reversed(self.waypoints))
        self.waypoint_index = 0
        self.get_logger().info(f"Iniciando caminho para objetivo {self.current_path_index + 1}.")

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        if self.waypoint_index >= len(self.waypoints):
            return

        target = self.waypoints[self.waypoint_index]
        dx = target[0] - self.x
        dy = target[1] - self.y
        distance = math.hypot(dx, dy)
        angle_to_target = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(angle_to_target - self.yaw)

        twist = Twist()

        if distance > self.tolerance:
            if abs(angle_diff) > 0.2:
                twist.angular.z = self.angular_speed * (angle_diff / abs(angle_diff))
                twist.linear.x = 0.0
            else:
                twist.linear.x = min(self.linear_speed, 0.5 * distance)
                twist.angular.z = 0.0
        else:
            self.get_logger().info(f'Alvo {self.waypoint_index} alcançado: {target}')
            self.waypoint_index += 1

            if self.waypoint_index == len(self.waypoints):
                if not self.returning:
                    self.get_logger().info("Item coletado. Retornando à origem.")
                    time.sleep(2)
                    self.returning = True
                    self.waypoints = self.return_path
                    self.waypoint_index = 1
                else:
                    self.get_logger().info("Retornou à origem.")

                    self.current_path_index += 1
                    if self.current_path_index < len(self.forward_paths):
                        self.returning = False
                        self.load_current_path()
                    else:
                        self.get_logger().info("Missão concluída com todos os objetivos.")
                        self.cmd_pub.publish(Twist())
                        rclpy.shutdown()
                        return

        self.cmd_pub.publish(twist)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2*math.pi
        while angle < -math.pi:
            angle += 2*math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


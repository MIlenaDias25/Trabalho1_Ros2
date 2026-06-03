#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math

TARGETS = [
    (-7.74, -8.87),  # Alvo 1
    ( 7.98, -7.47),  # Alvo 2
]

GOAL_TOLERANCE = 0.3  # distância mínima para considerar que chegou [m]
OBSTACLE_DIST  = 0.6  # distância mínima para considerar obstáculo [m]


class Publisher(Node):
    def __init__(self):
        super().__init__("publisher")

        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(Odometry,  '/odom',      self.odom_cb, 10)
        self.create_subscription(LaserScan, '/base_scan', self.scan_cb, 10)
        self.timer = self.create_timer(0.1, self.publish_hello)

        self.x   = -8.18 
        self.y   =  6.25
        self.yaw = 0.0

        self.ranges     = []
        self.target_idx = 0  # começa pelo alvo 1
        self.done       = False

        self.get_logger().info("Publisher started")

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Converte quaternion para yaw (ângulo de rotação do robô)
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def scan_cb(self, msg):
        self.ranges = list(msg.ranges)

    def publish_hello(self):
        msg = Twist()

        if self.done or not self.ranges:
            self.publisher.publish(msg)
            return

        tx, ty = TARGETS[self.target_idx]

        # ── Chegou ao alvo? ───────────────────────────────────────────────────
        if abs(self.x - tx) < GOAL_TOLERANCE and abs(self.y - ty) < GOAL_TOLERANCE:
            self.get_logger().info(f'Alvo {self.target_idx + 1} alcancado!')
            self.target_idx += 1
            if self.target_idx >= len(TARGETS):
                self.get_logger().info('Todos os alvos alcancados!')
                self.done = True
            self.publisher.publish(msg)  # para o robô
            return

        # ── Tem obstáculo na frente? ──────────────────────────────────────────
        # Pega apenas os raios centrais do laser (cone de ~50° à frente)
        mid    = len(self.ranges) // 2
        cone   = self.ranges[mid - 100 : mid + 100]
        validos = [r for r in cone if math.isfinite(r) and r > 0.01]
        frente  = min(validos) if validos else float('inf')

        if frente < OBSTACLE_DIST:
            # Gira à esquerda para desviar
            msg.angular.z = 0.5
        else:
            # Caminho livre: calcula ângulo até o alvo e avança
            angulo_alvo = math.atan2(ty - self.y, tx - self.x)
            erro        = angulo_alvo - self.yaw

            # Normaliza o erro para [-π, π]
            while erro >  math.pi: erro -= 2 * math.pi
            while erro < -math.pi: erro += 2 * math.pi

            msg.linear.x  = 0.3          # sempre anda para frente
            msg.angular.z = erro * 0.8   # corrige o ângulo proporcionalmente

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=None)
    node = Publisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

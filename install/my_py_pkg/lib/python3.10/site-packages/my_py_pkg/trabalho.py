#!/usr/bin/env python3

import math
import rclpy

from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from math import atan2


class RobotNavigator(Node):

    def __init__(self):
        super().__init__("robot_navigator")

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            "/base_scan",
            self.scan_callback,
            10
        )

        # Publisher
        self.cmd_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )

        # Timer
        self.timer = self.create_timer(0.1, self.control_loop)

        # Robot pose
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Laser
        self.front_distance = 10.0

        # Targets
        self.targets = [
            (-7.74, -8.87),
            (7.98, -7.47)
        ]

        self.current_target = 0

        self.get_logger().info("Robot Navigator Started")


    def odom_callback(self, msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        self.yaw = atan2(siny_cosp, cosy_cosp)

    def scan_callback(self, msg):

        # Região frontal do laser
        front_ranges = (
            msg.ranges[0:80] +
            msg.ranges[1000:1080]
        )

        valid_ranges = [
            r for r in front_ranges
            if not math.isinf(r)
        ]

        if len(valid_ranges) > 0:
            self.front_distance = min(valid_ranges)
        else:
            self.front_distance = 10.0


    def normalize_angle(self, angle):

        while angle > math.pi:
            angle -= 2 * math.pi

        while angle < -math.pi:
            angle += 2 * math.pi

        return angle


    def control_loop(self):

        if self.current_target >= len(self.targets):

            stop = Twist()
            self.cmd_pub.publish(stop)

            self.get_logger().info("All targets reached!")
            return

        target_x, target_y = self.targets[self.current_target]

        dx = target_x - self.x
        dy = target_y - self.y

        distance = math.sqrt(dx**2 + dy**2)

        # Verifica se chegou
        if abs(dx) < 0.3 and abs(dy) < 0.3:

            self.get_logger().info(
                f"Target {self.current_target + 1} reached!"
            )

            self.current_target += 1
            return

        cmd = Twist()

        # DESVIO DE OBSTÁCULO
        if self.front_distance < 0.8:

            cmd.linear.x = 0.0
            cmd.angular.z = 0.6

            self.get_logger().info("Obstacle detected!")

        else:

            desired_angle = math.atan2(dy, dx)

            angle_error = self.normalize_angle(
                desired_angle - self.yaw
            )

            # Gira primeiro
            if abs(angle_error) > 0.2:

                cmd.linear.x = 0.0
                cmd.angular.z = 0.5 * angle_error

            # Anda reto
            else:

                cmd.linear.x = 0.4
                cmd.angular.z = 0.3 * angle_error

        self.cmd_pub.publish(cmd)


def main(args=None):

    rclpy.init(args=args)

    node = RobotNavigator()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()


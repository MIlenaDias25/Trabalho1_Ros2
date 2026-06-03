#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math


INITIAL_X = -8.18
INITIAL_Y = 6.25

TARGETS = [
    (-7.74, -8.87),
    (7.98, -7.47),
]

GOAL_ERROR_XY = 0.3

GOAL_APPROACH_DISTANCE = 1.2
GOAL_OBSTACLE_THRESHOLD = 0.35

LINEAR_SPEED = 0.45
ANGULAR_SPEED = 0.85

OBSTACLE_THRESHOLD = 0.75
WALL_FOLLOW_DIST = 0.75

LASER_ANGLE_DEG = 270.0
M_LINE_TOLERANCE = 0.25


class State:
    GOTO_GOAL = 'GOTO_GOAL'
    WALL_FOLLOW = 'WALL_FOLLOW'


class RobotNavigator(Node):

    def __init__(self):
        super().__init__('robot_navigator')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/base_scan', self.scan_callback, 10
        )

        self.x = INITIAL_X
        self.y = INITIAL_Y
        self.yaw = 0.0
        self.ranges = []

        self.current_target_idx = 0
        self.state = State.GOTO_GOAL
        self.mission_complete = False

        self.odom_initialized = False
        self.init_odom_x = 0.0
        self.init_odom_y = 0.0
        self.init_odom_yaw = 0.0

        self.hit_x = 0.0
        self.hit_y = 0.0
        self.hit_dist = float('inf')

        self.follow_side = 'right'

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Robô Navegador iniciado!')

    def odom_callback(self, msg: Odometry):
        ox = msg.pose.pose.position.x
        oy = msg.pose.pose.position.y
        q = msg.pose.pose.orientation

        odom_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        if not self.odom_initialized:
            self.init_odom_x = ox
            self.init_odom_y = oy
            self.init_odom_yaw = odom_yaw
            self.odom_initialized = True

            self.get_logger().info(
                f'Odom init: ({ox:.2f}, {oy:.2f}), yaw={math.degrees(odom_yaw):.1f}°'
            )

        dx = ox - self.init_odom_x
        dy = oy - self.init_odom_y

        cos_a = math.cos(self.init_odom_yaw)
        sin_a = math.sin(self.init_odom_yaw)

        self.x = INITIAL_X + (cos_a * dx + sin_a * dy)
        self.y = INITIAL_Y + (-sin_a * dx + cos_a * dy)

        self.yaw = odom_yaw - self.init_odom_yaw
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

    def scan_callback(self, msg: LaserScan):
        self.ranges = list(msg.ranges)

    def sector_min(self, idx_start, idx_end):
        if not self.ranges:
            return float('inf')

        n = len(self.ranges)
        idx_start = max(0, idx_start)
        idx_end = min(n, idx_end)

        values = [
            r for r in self.ranges[idx_start:idx_end]
            if math.isfinite(r) and r > 0.01
        ]

        return min(values) if values else float('inf')

    def get_sectors(self):
        n = len(self.ranges)

        if n == 0:
            inf = float('inf')
            return {
                'front': inf,
                'front_left': inf,
                'front_right': inf,
                'left': inf,
                'right': inf,
            }

        c = n // 2
        step = n / LASER_ANGLE_DEG

        f20 = int(20 * step)
        f50 = int(50 * step)
        f90 = int(90 * step)

        return {
            'front': self.sector_min(c - f20, c + f20),
            'front_left': self.sector_min(c, c + f50),
            'front_right': self.sector_min(c - f50, c),
            'left': self.sector_min(c, c + f90),
            'right': self.sector_min(c - f90, c),
        }

    def dist_to_m_line(self, tx, ty):
        sx, sy = INITIAL_X, INITIAL_Y
        dx = tx - sx
        dy = ty - sy

        length = math.hypot(dx, dy)

        if length < 1e-6:
            return float('inf')

        return abs(dy * self.x - dx * self.y + tx * sy - ty * sx) / length

    def target_direction_is_free(self, target_x, target_y):
        if not self.ranges:
            return False

        desired_yaw = math.atan2(target_y - self.y, target_x - self.x)

        desired_rel = math.atan2(
            math.sin(desired_yaw - self.yaw),
            math.cos(desired_yaw - self.yaw)
        )

        angle_deg = math.degrees(desired_rel)

        if angle_deg < -120 or angle_deg > 120:
            return False

        n = len(self.ranges)
        center = n // 2
        step = n / LASER_ANGLE_DEG

        idx = int(center - angle_deg * step)

        idx_start = max(0, idx - int(12 * step))
        idx_end = min(n, idx + int(12 * step))

        values = [
            r for r in self.ranges[idx_start:idx_end]
            if math.isfinite(r) and r > 0.01
        ]

        if not values:
            return True

        dist_to_goal = math.hypot(target_x - self.x, target_y - self.y)

        return min(values) > min(1.0, dist_to_goal)

    def control_loop(self):
        if not self.odom_initialized or self.mission_complete:
            return

        target_x, target_y = TARGETS[self.current_target_idx]

        error_x = abs(target_x - self.x)
        error_y = abs(target_y - self.y)

        dist_to_goal = math.hypot(
            target_x - self.x,
            target_y - self.y
        )

        if error_x <= GOAL_ERROR_XY and error_y <= GOAL_ERROR_XY:
            self.stop_robot()

            self.get_logger().info(
                f'Alvo {self.current_target_idx + 1} atingido! '
                f'Erro x={error_x:.2f}, erro y={error_y:.2f}'
            )

            self.current_target_idx += 1
            self.state = State.GOTO_GOAL

            if self.current_target_idx >= len(TARGETS):
                self.get_logger().info('Missão completa!')
                self.mission_complete = True
            else:
                self.get_logger().info(
                    f'Próximo alvo: {TARGETS[self.current_target_idx]}'
                )

            return

        obs = self.get_sectors()

        if dist_to_goal < GOAL_APPROACH_DISTANCE:
            front_blocked = obs['front'] < GOAL_OBSTACLE_THRESHOLD
        else:
            front_blocked = obs['front'] < OBSTACLE_THRESHOLD

        cmd = Twist()

        if self.state == State.GOTO_GOAL:

            desired_yaw = math.atan2(
                target_y - self.y,
                target_x - self.x
            )

            angle_error = math.atan2(
                math.sin(desired_yaw - self.yaw),
                math.cos(desired_yaw - self.yaw)
            )

            if dist_to_goal < GOAL_APPROACH_DISTANCE:
                cmd.linear.x = LINEAR_SPEED * 0.45
                cmd.angular.z = 1.2 * angle_error

                if obs['front'] < GOAL_OBSTACLE_THRESHOLD:
                    cmd.linear.x = 0.0
                    cmd.angular.z = ANGULAR_SPEED

            elif front_blocked:
                self.hit_x = self.x
                self.hit_y = self.y
                self.hit_dist = dist_to_goal

                self.follow_side = 'right' if obs['right'] >= obs['left'] else 'left'
                self.state = State.WALL_FOLLOW

                self.get_logger().info(
                    f'Obstáculo! Contornando pela '
                    f'{"direita" if self.follow_side == "right" else "esquerda"}'
                )

            else:
                if abs(angle_error) > 0.15:
                    cmd.linear.x = LINEAR_SPEED * max(
                        0.35,
                        1.0 - abs(angle_error)
                    )
                    cmd.angular.z = ANGULAR_SPEED * math.copysign(
                        1.0,
                        angle_error
                    )
                else:
                    cmd.linear.x = LINEAR_SPEED
                    cmd.angular.z = 0.8 * angle_error

        elif self.state == State.WALL_FOLLOW:

            on_m_line = self.dist_to_m_line(target_x, target_y) < M_LINE_TOLERANCE

            closer_to_goal = dist_to_goal < self.hit_dist - GOAL_ERROR_XY * 0.5

            away_from_hit = math.hypot(
                self.x - self.hit_x,
                self.y - self.hit_y
            ) > 0.8

            if dist_to_goal < GOAL_APPROACH_DISTANCE:
                self.state = State.GOTO_GOAL
                self.get_logger().info('Perto do alvo: retomando aproximação direta')

            elif on_m_line and closer_to_goal and away_from_hit and not front_blocked:
            	self.state = State.GOTO_GOAL
            	self.get_logger().info(
            		'Retomando navegação direta ao alvo'
    		)

            elif on_m_line and closer_to_goal and away_from_hit and not front_blocked:
                self.state = State.GOTO_GOAL
                self.get_logger().info('Retomando navegação direta ao alvo')

            else:
                cmd = self.wall_follow(obs)

        self.cmd_pub.publish(cmd)

    def wall_follow(self, obs):
        cmd = Twist()

        if self.follow_side == 'right':
            side_dist = obs['right']

            forward_blocked = (
                obs['front'] < OBSTACLE_THRESHOLD or
                obs['front_right'] < OBSTACLE_THRESHOLD * 0.75
            )

            if forward_blocked:
                cmd.linear.x = LINEAR_SPEED * 0.25
                cmd.angular.z = ANGULAR_SPEED

            elif side_dist > WALL_FOLLOW_DIST * 1.5:
                cmd.linear.x = LINEAR_SPEED * 0.75
                cmd.angular.z = -ANGULAR_SPEED * 0.35

            elif side_dist < WALL_FOLLOW_DIST * 0.6:
                cmd.linear.x = LINEAR_SPEED * 0.55
                cmd.angular.z = ANGULAR_SPEED * 0.6

            else:
                cmd.linear.x = LINEAR_SPEED * 0.85
                cmd.angular.z = 0.0

        else:
            side_dist = obs['left']

            forward_blocked = (
                obs['front'] < OBSTACLE_THRESHOLD or
                obs['front_left'] < OBSTACLE_THRESHOLD * 0.75
            )

            if forward_blocked:
                cmd.linear.x = LINEAR_SPEED * 0.25
                cmd.angular.z = -ANGULAR_SPEED

            elif side_dist > WALL_FOLLOW_DIST * 1.5:
                cmd.linear.x = LINEAR_SPEED * 0.75
                cmd.angular.z = ANGULAR_SPEED * 0.35

            elif side_dist < WALL_FOLLOW_DIST * 0.6:
                cmd.linear.x = LINEAR_SPEED * 0.55
                cmd.angular.z = -ANGULAR_SPEED * 0.6

            else:
                cmd.linear.x = LINEAR_SPEED * 0.85
                cmd.angular.z = 0.0

        return cmd

    def stop_robot(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)

    node = RobotNavigator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrompido.')
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

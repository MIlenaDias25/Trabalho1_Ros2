#!/usr/bin/env python3
"""
Trabalho 1 - Robô Navegador
Algoritmo: VFH + Trap Detection + Wall Follow de escape
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
import time

# ───────────────────────── Parâmetros ──────────────────────────────────────

INITIAL_X = -8.18
INITIAL_Y  =  6.25

TARGETS = [
    (-7.74, -8.87),
    ( 7.98, -7.47),
]

GOAL_TOLERANCE  = 0.3
LINEAR_SPEED    = 0.4
ANGULAR_SPEED   = 0.7
OBSTACLE_DIST   = 1.0   # m — distância de alerta do VFH
LASER_ANGLE_DEG = 270.0

# Trap detection
STUCK_TIME      = 4.0   # segundos parado no mesmo lugar = preso
STUCK_DIST      = 0.3   # se não andou mais que isso em STUCK_TIME, está preso

# Escape: quanto tempo contorna antes de tentar ir ao alvo de novo
ESCAPE_TIME     = 3.5   # segundos

# ───────────────────────────────────────────────────────────────────────────


class RobotNavigator(Node):

    def __init__(self):
        super().__init__('robot_navigator')

        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry,  '/odom',      self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/base_scan', self.scan_callback, 10)

        self.x   = INITIAL_X
        self.y   = INITIAL_Y
        self.yaw = 0.0

        self.ranges    = []
        self.angle_min = 0.0
        self.angle_inc = 0.0

        self.current_target_idx = 0
        self.mission_complete   = False

        self.odom_initialized = False
        self.init_odom_x      = 0.0
        self.init_odom_y      = 0.0
        self.init_odom_yaw    = 0.0

        # Trap detection
        self.last_check_time = time.time()
        self.last_check_x    = INITIAL_X
        self.last_check_y    = INITIAL_Y

        # Escape state
        self.escaping      = False
        self.escape_start  = 0.0
        self.escape_side   = 1.0   # +1 = esquerda, -1 = direita

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Robô Navegador (VFH + Trap Escape) iniciado!')

    # ──────────────────────── Callbacks ────────────────────────────────────

    def odom_callback(self, msg: Odometry):
        ox = msg.pose.pose.position.x
        oy = msg.pose.pose.position.y
        q  = msg.pose.pose.orientation
        odom_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        if not self.odom_initialized:
            self.init_odom_x   = ox
            self.init_odom_y   = oy
            self.init_odom_yaw = odom_yaw
            self.odom_initialized = True

        dx    = ox - self.init_odom_x
        dy    = oy - self.init_odom_y
        cos_a = math.cos(self.init_odom_yaw)
        sin_a = math.sin(self.init_odom_yaw)
        self.x   = INITIAL_X + ( cos_a * dx + sin_a * dy)
        self.y   = INITIAL_Y + (-sin_a * dx + cos_a * dy)
        self.yaw = math.atan2(
            math.sin(odom_yaw - self.init_odom_yaw),
            math.cos(odom_yaw - self.init_odom_yaw)
        )

    def scan_callback(self, msg: LaserScan):
        self.ranges    = list(msg.ranges)
        self.angle_min = msg.angle_min
        self.angle_inc = msg.angle_increment

    # ──────────────────────── Utilitários laser ─────────────────────────────

    def sector_min(self, i_start, i_end):
        if not self.ranges:
            return float('inf')
        v = [r for r in self.ranges[i_start:i_end] if math.isfinite(r) and r > 0.01]
        return min(v) if v else float('inf')

    def get_sectors(self):
        n = len(self.ranges)
        if n == 0:
            return {k: float('inf') for k in ('front','left','right','front_left','front_right')}
        c    = n // 2
        step = n / LASER_ANGLE_DEG
        f20  = int(20 * step)
        f60  = int(60 * step)
        f90  = int(90 * step)
        return {
            'front':       self.sector_min(c - f20, c + f20),
            'front_left':  self.sector_min(c,       c + f60),
            'front_right': self.sector_min(c - f60, c),
            'left':        self.sector_min(c,       c + f90),
            'right':       self.sector_min(c - f90, c),
        }

    # ──────────────────────── VFH ──────────────────────────────────────────

    def vfh_direction(self, desired_world_angle: float):
        """
        Retorna (ângulo_mundial, livre) onde:
        - ângulo_mundial: melhor direção livre mais próxima do alvo
        - livre: False se nenhum raio está livre à frente
        """
        if not self.ranges or self.angle_inc == 0:
            return desired_world_angle, False

        desired_rel = math.atan2(
            math.sin(desired_world_angle - self.yaw),
            math.cos(desired_world_angle - self.yaw)
        )

        n      = len(self.ranges)
        window = 5   # suavização

        free_angles = []
        for i in range(window, n - window):
            local_min = min(
                (r for r in self.ranges[i - window: i + window + 1]
                 if math.isfinite(r) and r > 0.01),
                default=float('inf')
            )
            if local_min > OBSTACLE_DIST:
                ray_angle = self.angle_min + i * self.angle_inc
                free_angles.append(ray_angle)

        if not free_angles:
            return desired_world_angle, False

        best_rel = min(free_angles, key=lambda a: abs(
            math.atan2(math.sin(a - desired_rel), math.cos(a - desired_rel))
        ))
        world_angle = math.atan2(
            math.sin(self.yaw + best_rel),
            math.cos(self.yaw + best_rel)
        )
        return world_angle, True

    # ──────────────────────── Trap detection ───────────────────────────────

    def check_stuck(self) -> bool:
        now = time.time()
        if now - self.last_check_time >= STUCK_TIME:
            moved = math.hypot(self.x - self.last_check_x, self.y - self.last_check_y)
            self.last_check_time = now
            self.last_check_x    = self.x
            self.last_check_y    = self.y
            if moved < STUCK_DIST:
                return True
        return False

    # ──────────────────────── Loop de controle ─────────────────────────────

    def control_loop(self):
        if not self.odom_initialized or self.mission_complete:
            return

        target_x, target_y = TARGETS[self.current_target_idx]
        dist = math.hypot(target_x - self.x, target_y - self.y)

        if dist < GOAL_TOLERANCE:
            self.stop_robot()
            self.get_logger().info(
                f'✔ Alvo {self.current_target_idx + 1} atingido! ({self.x:.2f}, {self.y:.2f})')
            self.current_target_idx += 1
            self.escaping = False
            if self.current_target_idx >= len(TARGETS):
                self.get_logger().info('🎉 Missão completa!')
                self.mission_complete = True
            else:
                self.get_logger().info(f'➡ Próximo alvo: {TARGETS[self.current_target_idx]}')
            return

        obs = self.get_sectors()
        cmd = Twist()

        # ── Detectar armadilha ───────────────────────────────────────────
        if not self.escaping and self.check_stuck():
            self.escaping     = True
            self.escape_start = time.time()
            # Escolhe o lado com mais espaço para escapar
            self.escape_side  = 1.0 if obs['left'] >= obs['right'] else -1.0
            self.get_logger().info(
                f'🚨 Preso detectado! Escapando pela {"esquerda" if self.escape_side > 0 else "direita"}')

        # ── Modo escape: contorna a parede ativamente ────────────────────
        if self.escaping:
            elapsed = time.time() - self.escape_start
            if elapsed > ESCAPE_TIME:
                self.escaping = False
                self.get_logger().info('↩ Fim do escape, retomando VFH')
            else:
                cmd = self._escape_move(obs)
                self.cmd_pub.publish(cmd)
                return

        # ── VFH normal ───────────────────────────────────────────────────
        desired_yaw  = math.atan2(target_y - self.y, target_x - self.x)
        nav_yaw, ok  = self.vfh_direction(desired_yaw)

        if not ok:
            # Sem direção livre — gira no lugar
            cmd.linear.x  = 0.0
            cmd.angular.z = ANGULAR_SPEED
            self.cmd_pub.publish(cmd)
            return

        angle_error = math.atan2(
            math.sin(nav_yaw - self.yaw),
            math.cos(nav_yaw - self.yaw)
        )

        if abs(angle_error) > 0.8:
            cmd.linear.x  = LINEAR_SPEED * 0.1
            cmd.angular.z = ANGULAR_SPEED * math.copysign(1.0, angle_error)
        elif abs(angle_error) > 0.15:
            cmd.linear.x  = LINEAR_SPEED * (1.0 - 0.5 * abs(angle_error))
            cmd.angular.z = ANGULAR_SPEED * 0.8 * math.copysign(1.0, angle_error)
        else:
            cmd.linear.x  = LINEAR_SPEED
            cmd.angular.z = 1.2 * angle_error

        self.cmd_pub.publish(cmd)

    # ── Manobra de escape: avança contornando a parede ───────────────────
    def _escape_move(self, obs):
        """
        Contorna a parede pelo lado escolhido (escape_side).
        escape_side = +1 → segue pela esquerda (parede à direita)
                    = -1 → segue pela direita (parede à esquerda)
        """
        cmd = Twist()
        side = self.escape_side

        front_blocked = obs['front'] < OBSTACLE_DIST

        if side > 0:   # parede à direita, avança virando à esquerda se necessário
            wall_dist = obs['right']
            if front_blocked or obs['front_right'] < OBSTACLE_DIST * 0.8:
                cmd.linear.x  = 0.05
                cmd.angular.z =  ANGULAR_SPEED
            elif wall_dist > 1.3:
                cmd.linear.x  = LINEAR_SPEED * 0.5
                cmd.angular.z = -ANGULAR_SPEED * 0.4
            elif wall_dist < 0.6:
                cmd.linear.x  = LINEAR_SPEED * 0.5
                cmd.angular.z =  ANGULAR_SPEED * 0.4
            else:
                cmd.linear.x  = LINEAR_SPEED * 0.8
                cmd.angular.z = 0.0
        else:           # parede à esquerda
            wall_dist = obs['left']
            if front_blocked or obs['front_left'] < OBSTACLE_DIST * 0.8:
                cmd.linear.x  = 0.05
                cmd.angular.z = -ANGULAR_SPEED
            elif wall_dist > 1.3:
                cmd.linear.x  = LINEAR_SPEED * 0.5
                cmd.angular.z =  ANGULAR_SPEED * 0.4
            elif wall_dist < 0.6:
                cmd.linear.x  = LINEAR_SPEED * 0.5
                cmd.angular.z = -ANGULAR_SPEED * 0.4
            else:
                cmd.linear.x  = LINEAR_SPEED * 0.8
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

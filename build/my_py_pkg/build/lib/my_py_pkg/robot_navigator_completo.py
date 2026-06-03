#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math

# ─────────────────────────── Parâmetros ────────────────────────────────────

INITIAL_X = -8.18
INITIAL_Y  =  6.25

TARGETS = [
    (-7.74, -8.87),
    ( 7.98, -7.47),
]

GOAL_TOLERANCE      = 0.3
LINEAR_SPEED        = 0.60
ANGULAR_SPEED       = 0.90
OBSTACLE_THRESHOLD  = 0.70 # distância de alerta (m)
WALL_FOLLOW_DIST    = 0.70 # distância desejada da parede no contorno
LASER_RAYS          = 1080
LASER_ANGLE_DEG     = 270.0

# Bug2: abandona o contorno se chegou mais perto do alvo do que quando começou
# e está de volta na linha M (linha reta start→goal)
M_LINE_TOLERANCE    = 0.25  # metros de tolerância para considerar "na linha M"

# ───────────────────────────────────────────────────────────────────────────

class State:
    GOTO_GOAL    = 'GOTO_GOAL'
    WALL_FOLLOW  = 'WALL_FOLLOW'
    DONE         = 'DONE'


class RobotNavigator(Node):

    def __init__(self):
        super().__init__('robot_navigator')

        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry,  '/odom',      self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/base_scan', self.scan_callback, 10)

        self.x   = INITIAL_X
        self.y   = INITIAL_Y
        self.yaw = 0.0

        self.ranges = []

        self.current_target_idx = 0
        self.state              = State.GOTO_GOAL
        self.mission_complete   = False

        self.odom_initialized = False
        self.init_odom_x   = 0.0
        self.init_odom_y   = 0.0
        self.init_odom_yaw = 0.0

        # Bug2: ponto onde colidiu e distância ao alvo nesse momento
        self.hit_x    = 0.0
        self.hit_y    = 0.0
        self.hit_dist = float('inf')

        # Lado do contorno (left/right wall follow)
        self.follow_side = 'right'

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Robô Navegador iniciado!')

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
            self.get_logger().info(f'Odom init: ({ox:.2f}, {oy:.2f}), yaw={math.degrees(odom_yaw):.1f}°')

        dx = ox - self.init_odom_x
        dy = oy - self.init_odom_y
        cos_a = math.cos(self.init_odom_yaw)
        sin_a = math.sin(self.init_odom_yaw)

        self.x   = INITIAL_X + ( cos_a * dx + sin_a * dy)
        self.y   = INITIAL_Y + (-sin_a * dx + cos_a * dy)
        self.yaw = odom_yaw - self.init_odom_yaw
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

    def scan_callback(self, msg: LaserScan):
        self.ranges = list(msg.ranges)

    # ──────────────────────── Utilitários laser ─────────────────────────────

    def sector_min(self, idx_start: int, idx_end: int) -> float:
        if not self.ranges:
            return float('inf')
        s = self.ranges[idx_start:idx_end]
        v = [r for r in s if math.isfinite(r) and r > 0.01]
        return min(v) if v else float('inf')

    def get_sectors(self):
        n    = len(self.ranges)
        if n == 0:
            inf = float('inf')
            return {k: inf for k in ('front','front_left','front_right','left','right')}

        c    = n // 2
        step = n / LASER_ANGLE_DEG

        f20  = int(20  * step)
        f50  = int(50  * step)
        f90  = int(90  * step)

        return {
            'front':        self.sector_min(c - f20, c + f20),
            'front_left':   self.sector_min(c,       c + f50),
            'front_right':  self.sector_min(c - f50, c),
            'left':         self.sector_min(c,       c + f90),
            'right':        self.sector_min(c - f90, c),
        }

    def best_free_direction(self):
        """
        Varre os setores do laser e retorna o ângulo (relativo ao robô)
        da direção com mais espaço livre, priorizando o ângulo mais próximo
        do alvo.
        """
        if not self.ranges:
            return 0.0

        n     = len(self.ranges)
        step  = LASER_ANGLE_DEG / n          # graus por raio
        offset= LASER_ANGLE_DEG / 2.0        # 135°  →  raio 0 aponta para a direita

        target_x, target_y = TARGETS[self.current_target_idx]
        desired_yaw = math.atan2(target_y - self.y, target_x - self.x)
        desired_rel = math.degrees(desired_yaw - self.yaw)
        desired_rel = (desired_rel + 180) % 360 - 180  # [-180, 180]

        # Janela deslizante: média de 5 raios, escolhe a mais afastada de obstáculos
        window = 5
        best_angle  = 0.0
        best_score  = -float('inf')

        for i in range(window, n - window):
            d = min(self.ranges[i - window: i + window + 1],
                    key=lambda r: r if math.isfinite(r) else float('inf'))
            if not math.isfinite(d):
                d = 10.0

            # Ângulo relativo deste raio (direita = negativo, esquerda = positivo)
            ray_angle = offset - i * step   # graus, relativo ao robô

            # Penalizar por diferença ao ângulo desejado
            diff  = abs(ray_angle - desired_rel)
            score = d - 0.3 * diff

            if score > best_score:
                best_score  = score
                best_angle  = ray_angle

        return math.radians(best_angle)

    # ──────────────────────── Bug2: linha M ─────────────────────────────────

    def dist_to_m_line(self, tx, ty):
        """Distância perpendicular do robô à linha M (start→goal)."""
        sx, sy = INITIAL_X, INITIAL_Y
        dx, dy = tx - sx, ty - sy
        length = math.hypot(dx, dy)
        if length < 1e-6:
            return float('inf')
        return abs(dy * self.x - dx * self.y + tx * sy - ty * sx) / length

    # ──────────────────────── Loop de controle ──────────────────────────────

    def control_loop(self):
        if not self.odom_initialized or self.mission_complete:
            return

        target_x, target_y = TARGETS[self.current_target_idx]
        dist_to_goal = math.hypot(target_x - self.x, target_y - self.y)

        # ── Chegou? ─────────────────────────────────────────────────────
        if dist_to_goal < GOAL_TOLERANCE:
            self.stop_robot()
            self.get_logger().info(
                f'Alvo {self.current_target_idx + 1} atingido! ({self.x:.2f}, {self.y:.2f})')
            self.current_target_idx += 1
            self.state = State.GOTO_GOAL
            if self.current_target_idx >= len(TARGETS):
                self.get_logger().info('Missão completa!')
                self.mission_complete = True
            else:
                self.get_logger().info(f'Próximo alvo: {TARGETS[self.current_target_idx]}')
            return

        obs = self.get_sectors()
        front_blocked = obs['front'] < OBSTACLE_THRESHOLD

        cmd = Twist()

        # ════════════════════════════════════════════════════════════════
        if self.state == State.GOTO_GOAL:
            if front_blocked:
                # ── Entrar no modo contorno ──────────────────────────
                self.hit_x    = self.x
                self.hit_y    = self.y
                self.hit_dist = dist_to_goal

                # Escolhe o lado com mais espaço
                self.follow_side = 'right' if obs['right'] >= obs['left'] else 'left'
                self.state = State.WALL_FOLLOW
                self.get_logger().info(
                    f'⚠ Obstáculo! Contornando pela {"direita" if self.follow_side=="right" else "esquerda"}')
            else:
                # ── Navegar em direção ao alvo ───────────────────────
                desired_yaw = math.atan2(target_y - self.y, target_x - self.x)
                angle_error = math.atan2(
                    math.sin(desired_yaw - self.yaw),
                    math.cos(desired_yaw - self.yaw)
                )

                if abs(angle_error) > 0.15:
                    cmd.linear.x  = LINEAR_SPEED * max(0.3, 1.0 - abs(angle_error))
                    cmd.angular.z = ANGULAR_SPEED * math.copysign(1.0, angle_error)
                else:
                    cmd.linear.x  = LINEAR_SPEED
                    cmd.angular.z = 0.8 * angle_error

        # ════════════════════════════════════════════════════════════════
        elif self.state == State.WALL_FOLLOW:

            # ── Condição de saída Bug2 ───────────────────────────────
            on_m_line     = self.dist_to_m_line(target_x, target_y) < M_LINE_TOLERANCE
            closer_to_goal= dist_to_goal < self.hit_dist - GOAL_TOLERANCE * 0.5
            away_from_hit = math.hypot(self.x - self.hit_x, self.y - self.hit_y) > 0.8

            if on_m_line and closer_to_goal and away_from_hit and not front_blocked:
                self.state = State.GOTO_GOAL
                self.get_logger().info('Retomando navegação direta ao alvo')
            else:
                # ── Wall following ───────────────────────────────────
                cmd = self._wall_follow(obs)

        self.cmd_pub.publish(cmd)

    def _wall_follow(self, obs):
        """
        Segue a parede mantendo WALL_FOLLOW_DIST de distância.
        """
        cmd = Twist()

        if self.follow_side == 'right':
            side_dist = obs['right']
            forward_blocked = obs['front'] < OBSTACLE_THRESHOLD or \
                              obs['front_right'] < OBSTACLE_THRESHOLD * 0.8

            if forward_blocked:
                # Vira à esquerda (afasta da parede à direita que está bloqueando)
                cmd.linear.x  = 0.0
                cmd.angular.z =  ANGULAR_SPEED
            elif side_dist > WALL_FOLLOW_DIST * 1.3:
                # Muito longe da parede → curva suave à direita
                cmd.linear.x  = LINEAR_SPEED * 0.6
                cmd.angular.z = -ANGULAR_SPEED * 0.5
            elif side_dist < WALL_FOLLOW_DIST * 0.7:
                # Muito perto → curva suave à esquerda
                cmd.linear.x  = LINEAR_SPEED * 0.6
                cmd.angular.z =  ANGULAR_SPEED * 0.5
            else:
                # Na faixa correta
                cmd.linear.x  = LINEAR_SPEED
                cmd.angular.z = 0.0

        else:  # follow_side == 'left'
            side_dist = obs['left']
            forward_blocked = obs['front'] < OBSTACLE_THRESHOLD or \
                              obs['front_left'] < OBSTACLE_THRESHOLD * 0.8

            if forward_blocked:
                cmd.linear.x  = 0.0
                cmd.angular.z = -ANGULAR_SPEED
            elif side_dist > WALL_FOLLOW_DIST * 1.3:
                cmd.linear.x  = LINEAR_SPEED * 0.6
                cmd.angular.z =  ANGULAR_SPEED * 0.5
            elif side_dist < WALL_FOLLOW_DIST * 0.7:
                cmd.linear.x  = LINEAR_SPEED * 0.6
                cmd.angular.z = -ANGULAR_SPEED * 0.5
            else:
                cmd.linear.x  = LINEAR_SPEED
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

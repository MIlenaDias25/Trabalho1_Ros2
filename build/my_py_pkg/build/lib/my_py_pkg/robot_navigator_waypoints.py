#!/usr/bin/env python3
"""
Trabalho 1 - Robô Navegador

Estratégia usada:
- Navegação por waypoints para encurtar o caminho no mapa conhecido.
- Desvio local com LaserScan usando histograma de direção livre, estilo VFH simplificado.
- Odometria corrigida por matriz de rotação, usando a posição inicial exigida no trabalho.

Tópicos usados:
- /odom      -> posição do robô
- /base_scan -> sensor laser
- /cmd_vel   -> comando de velocidade
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


# ─────────────────────────── Parâmetros do trabalho ─────────────────────────

INITIAL_X = -8.18
INITIAL_Y = 6.25

# Alvos obrigatórios do enunciado
TARGET_1 = (-7.74, -8.87)
TARGET_2 = (7.98, -7.47)

# Waypoints: incluem os dois alvos obrigatórios e pontos intermediários
# para diminuir o caminho e evitar contornos muito longos.
WAYPOINTS = [
    TARGET_1,        # alvo 1 obrigatório
    (-5.5, -8.8),
    (-3.2, -8.5),
    (-2.0, -6.0),
    (-1.2, -3.8),
    (1.0, -3.2),
    (3.8, -3.0),
    (6.8, -4.2),
    TARGET_2,        # alvo 2 obrigatório
]

# Índices dos alvos obrigatórios dentro de WAYPOINTS
REQUIRED_TARGET_INDEXES = {0: 1, 8: 2}

# Tolerâncias
WAYPOINT_TOLERANCE = 0.45
GOAL_TOLERANCE = 0.30

# Velocidades
MAX_LINEAR_SPEED = 0.60
MIN_LINEAR_SPEED = 0.55
MAX_ANGULAR_SPEED = 0.85

# Laser
LASER_ANGLE_DEG = 270.0
OBSTACLE_THRESHOLD = 0.65      # abaixo disso, começa a desviar
DANGER_THRESHOLD = 0.35        # abaixo disso, para de avançar e gira
CLEAR_DISTANCE = 1.40          # distância considerada bem livre


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

        self.odom_initialized = False
        self.init_odom_x = 0.0
        self.init_odom_y = 0.0
        self.init_odom_yaw = 0.0

        self.current_waypoint_idx = 0
        self.reached_target_1 = False
        self.reached_target_2 = False
        self.mission_complete = False

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Robô Navegador com waypoints + VFH iniciado!')

    # ─────────────────────────── Callbacks ───────────────────────────

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
                f'Odom inicializada: ({ox:.2f}, {oy:.2f}), '
                f'yaw={math.degrees(odom_yaw):.1f}°'
            )

        # Diferença medida pela odometria
        dx = ox - self.init_odom_x
        dy = oy - self.init_odom_y

        # Matriz de rotação para corrigir o referencial inicial
        cos_a = math.cos(self.init_odom_yaw)
        sin_a = math.sin(self.init_odom_yaw)

        self.x = INITIAL_X + (cos_a * dx + sin_a * dy)
        self.y = INITIAL_Y + (-sin_a * dx + cos_a * dy)
        self.yaw = odom_yaw - self.init_odom_yaw
        self.yaw = self.normalize_angle(self.yaw)

    def scan_callback(self, msg: LaserScan):
        self.ranges = list(msg.ranges)

    # ─────────────────────────── Funções auxiliares ─────────────────────────

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def clamp(value, min_value, max_value):
        return max(min_value, min(max_value, value))

    def valid_range(self, r):
        return math.isfinite(r) and r > 0.02

    def sector_min(self, start_idx, end_idx):
        if not self.ranges:
            return float('inf')

        n = len(self.ranges)
        start_idx = max(0, min(n, start_idx))
        end_idx = max(0, min(n, end_idx))

        if start_idx >= end_idx:
            return float('inf')

        values = [r for r in self.ranges[start_idx:end_idx] if self.valid_range(r)]
        return min(values) if values else float('inf')

    def get_sectors(self):
        if not self.ranges:
            inf = float('inf')
            return {
                'front': inf,
                'front_left': inf,
                'front_right': inf,
                'left': inf,
                'right': inf,
            }

        n = len(self.ranges)
        center = n // 2
        step = n / LASER_ANGLE_DEG

        f20 = int(20 * step)
        f45 = int(45 * step)
        f90 = int(90 * step)

        return {
            'front': self.sector_min(center - f20, center + f20),
            'front_left': self.sector_min(center, center + f45),
            'front_right': self.sector_min(center - f45, center),
            'left': self.sector_min(center, center + f90),
            'right': self.sector_min(center - f90, center),
        }

    def current_waypoint(self):
        return WAYPOINTS[self.current_waypoint_idx]

    def distance_to_current_waypoint(self):
        wx, wy = self.current_waypoint()
        return math.hypot(wx - self.x, wy - self.y)

    def reached_waypoint(self):
        wx, wy = self.current_waypoint()
        dx = abs(wx - self.x)
        dy = abs(wy - self.y)

        if self.current_waypoint_idx in REQUIRED_TARGET_INDEXES:
            # Para os alvos obrigatórios, respeita erro máximo de 0.3 em x e y.
            return dx <= GOAL_TOLERANCE and dy <= GOAL_TOLERANCE

        return math.hypot(wx - self.x, wy - self.y) <= WAYPOINT_TOLERANCE

    def desired_angle_to_waypoint(self):
        wx, wy = self.current_waypoint()
        return math.atan2(wy - self.y, wx - self.x)

    def best_free_direction(self, desired_rel_angle):
        """
        Escolhe a melhor direção livre no laser.
        O score favorece:
        - maior distância de obstáculo;
        - menor diferença em relação ao rumo desejado.
        """
        if not self.ranges:
            return desired_rel_angle

        n = len(self.ranges)
        step_deg = LASER_ANGLE_DEG / n
        offset_deg = LASER_ANGLE_DEG / 2.0
        desired_deg = math.degrees(desired_rel_angle)

        window = max(3, int(n * 0.008))
        best_angle_deg = 0.0
        best_score = -float('inf')

        # Limita a busca para evitar escolher direções muito para trás.
        max_side_angle = 105.0

        for i in range(window, n - window):
            ray_angle_deg = offset_deg - i * step_deg

            if abs(ray_angle_deg) > max_side_angle:
                continue

            local_values = []
            for r in self.ranges[i - window:i + window + 1]:
                if self.valid_range(r):
                    local_values.append(min(r, CLEAR_DISTANCE))

            if not local_values:
                local_dist = CLEAR_DISTANCE
            else:
                local_dist = min(local_values)

            # Ignora direções muito apertadas, exceto se não houver opção melhor.
            if local_dist < DANGER_THRESHOLD:
                distance_score = -5.0
            else:
                distance_score = local_dist

            angle_diff = abs(self.normalize_angle(math.radians(ray_angle_deg - desired_deg)))

            # Peso do ângulo: mantém tendência de ir ao waypoint.
            score = distance_score - 0.75 * angle_diff

            if score > best_score:
                best_score = score
                best_angle_deg = ray_angle_deg

        return math.radians(best_angle_deg)

    # ─────────────────────────── Controle principal ─────────────────────────

    def control_loop(self):
        if not self.odom_initialized or self.mission_complete:
            return

        if self.current_waypoint_idx >= len(WAYPOINTS):
            self.stop_robot()
            self.mission_complete = True
            self.get_logger().info('Missão completa! Alvos 1 e 2 alcançados.')
            return

        if self.reached_waypoint():
            self.handle_waypoint_reached()
            return

        obs = self.get_sectors()
        desired_yaw = self.desired_angle_to_waypoint()
        desired_rel = self.normalize_angle(desired_yaw - self.yaw)

        # Se tiver obstáculo, troca o rumo desejado pelo melhor rumo livre.
        front_blocked = obs['front'] < OBSTACLE_THRESHOLD
        danger = obs['front'] < DANGER_THRESHOLD

        if front_blocked or obs['front_left'] < OBSTACLE_THRESHOLD * 0.85 or obs['front_right'] < OBSTACLE_THRESHOLD * 0.85:
            target_rel = self.best_free_direction(desired_rel)
        else:
            target_rel = desired_rel

        cmd = Twist()

        # Controle angular proporcional, limitado.
        cmd.angular.z = self.clamp(1.8 * target_rel, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED)

        # Velocidade linear adaptativa.
        angle_abs = abs(target_rel)
        dist_wp = self.distance_to_current_waypoint()

        if danger:
            cmd.linear.x = 0.0
        else:
            # Reduz quando está muito desalinhado, perto de obstáculo ou perto do waypoint.
            angle_factor = self.clamp(1.0 - angle_abs / 1.2, 0.0, 1.0)
            obstacle_factor = self.clamp((obs['front'] - DANGER_THRESHOLD) / (CLEAR_DISTANCE - DANGER_THRESHOLD), 0.25, 1.0)
            goal_factor = self.clamp(dist_wp / 1.5, 0.35, 1.0)

            speed = MAX_LINEAR_SPEED * angle_factor * obstacle_factor * goal_factor

            if angle_abs > 1.15:
                speed = 0.0
            elif angle_abs > 0.65:
                speed = min(speed, 0.18)

            cmd.linear.x = self.clamp(speed, MIN_LINEAR_SPEED if speed > 0 else 0.0, MAX_LINEAR_SPEED)

        self.cmd_pub.publish(cmd)

    def handle_waypoint_reached(self):
        idx = self.current_waypoint_idx

        if idx in REQUIRED_TARGET_INDEXES:
            target_number = REQUIRED_TARGET_INDEXES[idx]
            self.get_logger().info(
                f'Alvo {target_number} atingido! Posição atual: ({self.x:.2f}, {self.y:.2f})'
            )
            if target_number == 1:
                self.reached_target_1 = True
            elif target_number == 2:
                self.reached_target_2 = True

        else:
            self.get_logger().info(
                f'Waypoint {idx + 1}/{len(WAYPOINTS)} atingido: ({self.x:.2f}, {self.y:.2f})'
            )

        self.current_waypoint_idx += 1
        self.stop_robot()

        if self.current_waypoint_idx >= len(WAYPOINTS):
            self.mission_complete = True
            self.get_logger().info('Missão completa! Alvos 1 e 2 alcançados.')
        else:
            wx, wy = self.current_waypoint()
            self.get_logger().info(
                f'Próximo waypoint: ({wx:.2f}, {wy:.2f})'
            )

    def stop_robot(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = RobotNavigator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Execução interrompida pelo usuário.')
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

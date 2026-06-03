# Trabalho 1 — Robô Navegador

**Disciplina:** Introdução à Robótica Inteligente  
**Algoritmo:** Bug2 com seguimento de parede (Wall Following)

---

## Autora

Milena Dias de Oliveira

---

## Descrição

Nodo ROS 2 que controla um robô diferencial no simulador Stage. O robô parte da posição inicial **(x = -8.18, y = 6.25)** e deve alcançar dois alvos em sequência:

- **Alvo 1:** (x = -7.74, y = -8.87)
- **Alvo 2:** (x = 7.98, y = -7.47)

O algoritmo utilizado é o **Bug2**: o robô tenta seguir em linha reta até o alvo (M-line) e, ao encontrar um obstáculo, contorna a parede até retornar à M-line em um ponto mais próximo do objetivo.

O robô calcula continuamente o ângulo em direção ao alvo.

Quando um obstáculo é detectado: 
- O robô registra o ponto de colisão virtual (Hit Point);
- Entra no modo de seguimento de parede (Wall Following).

Durante o seguimento da parede, o algoritmo verifica:

- Se o robô voltou para a linha M;
- Se está mais próximo do alvo do que estava no ponto de colisão.

Quando essas condições são satisfeitas, o robô retorna ao objetivo principal.

---

## Requisitos

- ROS 2
- Simulador Stage
- Python 3
- Pacote `my_py_pkg` (desenvolvido nas vı́deo aulas sobre ROS2)

---

## Como executar

### 1. Clone esse repositório

```
https://github.com/MIlenaDias25/Trabalho1_Ros2.git
```
Acesse o diretório principal

```
ros2_ws
```
### 2. Compile no workspace ROS2

```
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### 2. Inicie o simulador Stage

```
ros2 launch stage_ros2 stage.launch.py
```

### 3. Execute o nodo nomeado como robot_navegador
Em outro terminal:

```cd ~/ros2_ws
ros2 run my_py_pkg robot_navigator
```

O robô iniciará automaticamente a navegação em direção ao Alvo 1 e, após atingi-lo, seguirá para o Alvo 2. A missão é encerrada e o robô para de navegar quando ambos os alvos são alcançados. 

---

## Tópicos ROS utilizados

| Tópico | Tipo | Função |
|---|---|---|
| `/odom` | `nav_msgs/Odometry` | Leitura da posição e orientação do robô |
| `/base_scan` | `sensor_msgs/LaserScan` | Leitura das distâncias do sensor LiDAR |
| `/cmd_vel` | `geometry_msgs/Twist` | Envio de comandos de velocidade |

---

## Parâmetros principais

| Parâmetro | Valor | Descrição |
|---|---|---|
| `GOAL_TOLERANCE` | 0.3 m | Distância mínima para considerar alvo atingido |
| `LINEAR_SPEED` | 0.60 m/s | Velocidade linear máxima |
| `ANGULAR_SPEED` | 0.90 rad/s | Velocidade angular máxima |
| `OBSTACLE_THRESHOLD` | 0.70 m | Distância mínima para detecção de obstáculo |
| `WALL_FOLLOW_DIST` | 0.70 m | Distância desejada de manutenção em relação à parede |
| `M_LINE_TOLERANCE` | 0.25 m | Tolerância para retorno à linha M |

---

## Vídeo de demonstração

[Ver no YouTube](<INSERIR_LINK_AQUI>)

---

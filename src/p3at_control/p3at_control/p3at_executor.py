#!/usr/bin/env python3
# --------------------------------------------------------------------------------------
# FILE: p3at_executor.py (Versão 2.4 - FULL ROBUST + STOP PRIORITY)
# --------------------------------------------------------------------------------------
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool, Empty
import json
import time
import math


def parse_bool(val):
    if isinstance(val, bool):
        return val
    if isinstance(val, (int, float)):
        return val != 0
    if isinstance(val, str):
        return val.strip().lower() in ('1', 'true', 'yes', 'on', 'enable', 'enabled', 'ativar', 'ligar')
    return False

class P3ATExecutor(Node):
    def __init__(self):
        super().__init__('p3at_executor_node')

        self.declare_parameter('direct_cmd_vel', True)
        self.declare_parameter('linear_timeout_per_meter', 12.0)
        self.declare_parameter('angular_timeout_per_90deg', 8.0)
        self.declare_parameter('linear_epsilon', 0.02)
        self.declare_parameter('angular_epsilon_deg', 2.0)
        self.declare_parameter('linear_slowdown_distance', 0.30)
        self.declare_parameter('angular_slowdown_deg', 25.0)
        self.declare_parameter('min_linear_approach_speed', 0.05)
        self.declare_parameter('min_angular_approach_speed', 0.12)
        self.declare_parameter('linear_target_bias_m', 0.09)
        self.declare_parameter('angular_target_bias_deg', 8.2)
        self.declare_parameter('explore_speed', 0.2)
        self.declare_parameter('turn_speed', 0.5)
        self.declare_parameter('avoid_gain', 1.0)
        self.declare_parameter('dist_stop', 0.70)
        self.declare_parameter('dist_avoid', 1.75)
        self.declare_parameter('turn_force', 1.0)
        self.declare_parameter('tail_cooldown', 0.4)
        self.declare_parameter('turn_duration', 0.7)
        self.declare_parameter('scan_timeout', 0.7)
        self.declare_parameter('lateral_avoid_factor', 1.25)
        self.declare_parameter('lateral_turn_hold', 0.9)
        self.declare_parameter('lateral_turn_cooldown', 1.2)
        self.declare_parameter('lateral_block_time', 0.8)
        self.declare_parameter('max_linear', 0.2)
        self.declare_parameter('max_angular', 0.5)

        # 1. SUBSCRIPTIONS
        self.create_subscription(String, '/p3at_target', self.cmd_callback, 10)
        self.create_subscription(Empty, '/reset_simulation', self.reset_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 20)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # 2. PUBLICATIONS
        cmd_vel_topic = '/cmd_vel' if self.get_parameter('direct_cmd_vel').value else '/high_level_cmd'
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.status_pub = self.create_publisher(Bool, '/executor_active', 10)

        # 3. ESTADO INTERNO
        self.current_task = None
        self.task_start_time = 0.0
        self.explore_active = False
        self.is_turning = False
        self.turn_end_time = 0.0
        self.tail_blind_time = 0.0
        self.locked_turn_z = 0.0
        self.last_scan_time = 0.0
        self.scan_front = None
        self.scan_left = None
        self.scan_right = None
        self.scan_lat_left = None
        self.scan_lat_right = None
        self.odom_pose = None
        self.last_odom_time = 0.0
        self.explore_lateral_lock_until = 0.0
        self.explore_lateral_lock_dir = 0.0
        self.explore_last_turn_dir = 0.0
        self.explore_last_turn_time = 0.0
        self.explore_lateral_blocked_since = 0.0
        self.explore_escape_until = 0.0
        self.explore_escape_v = 0.0
        self.explore_escape_w = 0.0
        self.explore_last_move_time = time.time()
        
        # Timer de controle (10Hz)
        self.timer = self.create_timer(0.1, self.control_loop) 

        self.get_logger().info("✅ Executor V2.4 (Robustez Máxima + Fix de Parada) pronto.")

    def _base_speed(self) -> float:
        return abs(float(self.get_parameter('explore_speed').value))

    def _turn_speed(self) -> float:
        return abs(float(self.get_parameter('turn_speed').value))

    def _clamp_cmd(self, lin: float, ang: float):
        max_linear = abs(float(self.get_parameter('max_linear').value))
        max_angular = abs(float(self.get_parameter('max_angular').value))
        lin_c = max(-max_linear, min(max_linear, float(lin)))
        ang_c = max(-max_angular, min(max_angular, float(ang)))
        return lin_c, ang_c

    def _yaw_from_quat(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _normalize_angle(self, angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def _distance_progress(self, start_pose, current_pose) -> float:
        dx = current_pose['x'] - start_pose['x']
        dy = current_pose['y'] - start_pose['y']
        return dx * math.cos(start_pose['yaw']) + dy * math.sin(start_pose['yaw'])

    def _rotation_progress(self, start_yaw: float, current_yaw: float) -> float:
        return self._normalize_angle(current_yaw - start_yaw)

    def _scaled_linear_speed(self, error: float) -> float:
        base = self._base_speed()
        min_speed = abs(float(self.get_parameter('min_linear_approach_speed').value))
        slowdown = max(abs(float(self.get_parameter('linear_slowdown_distance').value)), 1e-6)
        mag = abs(error)
        if mag >= slowdown:
            return base
        alpha = max(0.0, min(1.0, mag / slowdown))
        return min_speed + (base - min_speed) * alpha

    def _scaled_angular_speed(self, error_rad: float) -> float:
        base = self._turn_speed()
        min_speed = abs(float(self.get_parameter('min_angular_approach_speed').value))
        slowdown = math.radians(
            max(abs(float(self.get_parameter('angular_slowdown_deg').value)), 1e-6)
        )
        mag = abs(error_rad)
        if mag >= slowdown:
            return base
        alpha = max(0.0, min(1.0, mag / slowdown))
        return min_speed + (base - min_speed) * alpha

    def _apply_linear_bias(self, target_m: float) -> float:
        bias = abs(float(self.get_parameter('linear_target_bias_m').value))
        if abs(target_m) <= bias:
            return target_m
        return target_m - math.copysign(bias, target_m)

    def _apply_angular_bias_deg(self, target_deg: float) -> float:
        bias = abs(float(self.get_parameter('angular_target_bias_deg').value))
        if abs(target_deg) <= bias:
            return target_deg
        return target_deg - math.copysign(bias, target_deg)

    def reset_callback(self, msg):
        """Cancela tarefas pendentes se a simulação for resetada"""
        if self.current_task:
            self.get_logger().warn("♻️ Reset detectado! Cancelando tarefa atual.")
        self.current_task = None
        self.explore_active = False
        self._reset_explore_state()
        # Comando de parada imediata no reset
        self.cmd_vel_pub.publish(Twist()) 
        self.status_pub.publish(Bool(data=False))

    def odom_callback(self, msg: Odometry):
        self.odom_pose = {
            'x': float(msg.pose.pose.position.x),
            'y': float(msg.pose.pose.position.y),
            'yaw': self._yaw_from_quat(msg.pose.pose.orientation),
        }
        self.last_odom_time = time.time()

    def scan_callback(self, msg: LaserScan):
        ranges = list(msg.ranges) if msg.ranges else []
        if not ranges:
            return
        mid = len(ranges) // 2
        def clean(vals):
            out = []
            for v in vals:
                if not math.isfinite(v) or v <= 0.0:
                    continue
                out.append(v)
            return out

        front_vals = clean(ranges[mid-2:mid+3])
        left_vals = clean(ranges[mid+3:mid+8])
        right_vals = clean(ranges[mid-8:mid-3])
        lat_left_vals = clean(ranges[mid+8:mid+16])
        lat_right_vals = clean(ranges[mid-16:mid-8])

        if not front_vals or not left_vals or not right_vals:
            return

        self.scan_front = min(front_vals)
        self.scan_left = min(left_vals)
        self.scan_right = min(right_vals)
        self.scan_lat_left = min(lat_left_vals) if lat_left_vals else self.scan_left
        self.scan_lat_right = min(lat_right_vals) if lat_right_vals else self.scan_right
        self.last_scan_time = time.time()

    def cmd_callback(self, msg):
        try:
            data = json.loads(msg.data)
            cmd = data.get('command')
            val = data.get('value', 0.0)
            
            task = {'lin': 0.0, 'ang': 0.0, 'duration': 0.0}

            # Lógica de Distância (Andar X metros)
            if cmd == 'move_distance':
                self.explore_active = False
                self._reset_explore_state()
                raw_dist = float(val)
                dist = self._apply_linear_bias(raw_dist)
                if self.odom_pose is None:
                    self.get_logger().error("❌ Sem odometria para iniciar move_distance.")
                    self.cmd_vel_pub.publish(Twist())
                    self.status_pub.publish(Bool(data=False))
                    return
                task = {
                    'type': 'move_distance',
                    'target': dist,
                    'start_pose': dict(self.odom_pose),
                    'timeout_s': abs(dist) * float(self.get_parameter('linear_timeout_per_meter').value),
                }
            
            # Lógica de Ângulo (Girar X graus)
            elif cmd == 'turn_angle':
                self.explore_active = False
                self._reset_explore_state()
                raw_deg = float(val)
                deg = self._apply_angular_bias_deg(raw_deg)
                if self.odom_pose is None:
                    self.get_logger().error("❌ Sem odometria para iniciar turn_angle.")
                    self.cmd_vel_pub.publish(Twist())
                    self.status_pub.publish(Bool(data=False))
                    return
                task = {
                    'type': 'turn_angle',
                    'target_rad': math.radians(deg),
                    'start_yaw': float(self.odom_pose['yaw']),
                    'timeout_s': (abs(deg) / 90.0) * float(self.get_parameter('angular_timeout_per_90deg').value),
                }

            elif cmd == 'arc':
                self.explore_active = False
                self._reset_explore_state()
                task = {
                    'type': 'timed_cmd',
                    'lin': float(val.get('v', 0.0)),
                    'ang': float(val.get('w', 0.0)),
                    'duration': max(0.0, float(val.get('duration', 0.0))),
                }
            
            # Modo exploração simples (sem otimização)
            elif cmd == 'explore_mode':
                self.current_task = None
                self.explore_active = parse_bool(val)
                self._reset_explore_state()
                if not self.explore_active:
                    self.cmd_vel_pub.publish(Twist())
                    self.status_pub.publish(Bool(data=False))
                return

            # Comando de Parada Direta
            elif cmd == 'stop':
                self.current_task = None
                self.explore_active = False
                self._reset_explore_state()
                self.cmd_vel_pub.publish(Twist())
                self.status_pub.publish(Bool(data=False))
                return
            
            # Fallback: Comandos Diretos (Mantido como solicitado)
            else:
                self.explore_active = False
                task = {
                    'type': 'timed_cmd',
                    'lin': float(data.get('linear', 0.0)),
                    'ang': float(data.get('angular', 0.0)),
                    'duration': 0.5,
                }

            self.start_task(task)

        except Exception as e:
            self.get_logger().error(f"❌ Erro ao processar comando JSON: {e}")

    def start_task(self, task):
        self.current_task = task
        self.task_start_time = time.time()
        # Publica ocupado imediatamente
        self.status_pub.publish(Bool(data=True))

    def _finish_task(self, reason="done"):
        if self.current_task is not None:
            self.get_logger().info(f"⏹️ Tarefa finalizada ({reason}).")
        self.current_task = None
        self.cmd_vel_pub.publish(Twist())
        self.status_pub.publish(Bool(data=False))

    def _reset_explore_state(self):
        self.is_turning = False
        self.turn_end_time = 0.0
        self.tail_blind_time = 0.0
        self.locked_turn_z = 0.0
        self.explore_lateral_lock_until = 0.0
        self.explore_lateral_lock_dir = 0.0
        self.explore_last_turn_dir = 0.0
        self.explore_last_turn_time = 0.0
        self.explore_lateral_blocked_since = 0.0
        self.explore_escape_until = 0.0
        self.explore_escape_v = 0.0
        self.explore_escape_w = 0.0
        self.explore_last_move_time = time.time()

    def _explore_cmd(self):
        now = time.time()
        if self.last_scan_time == 0.0 or (now - self.last_scan_time) > float(self.get_parameter('scan_timeout').value):
            return None

        front = float(self.scan_front)
        left = float(self.scan_left)
        right = float(self.scan_right)
        lat_left = float(self.scan_lat_left) if self.scan_lat_left is not None else left
        lat_right = float(self.scan_lat_right) if self.scan_lat_right is not None else right

        dist_stop = float(self.get_parameter('dist_stop').value)
        dist_avoid = float(self.get_parameter('dist_avoid').value)
        turn_force = float(self.get_parameter('turn_force').value)
        avoid_gain = float(self.get_parameter('avoid_gain').value)
        tail_cooldown = float(self.get_parameter('tail_cooldown').value)
        turn_duration = float(self.get_parameter('turn_duration').value)
        speed_explore = self._base_speed()
        lateral_avoid = dist_avoid * float(self.get_parameter('lateral_avoid_factor').value)
        lateral_turn_hold = float(self.get_parameter('lateral_turn_hold').value)
        lateral_turn_cooldown = float(self.get_parameter('lateral_turn_cooldown').value)
        lateral_block_time = float(self.get_parameter('lateral_block_time').value)

        # Escape travado (anti-loop)
        if now < self.explore_escape_until:
            return (self.explore_escape_v, self.explore_escape_w)

        # Stuck: se não avançou, força escape
        if (now - self.explore_last_move_time) > 1.0:
            self._start_explore_escape(left, right)
            return (self.explore_escape_v, self.explore_escape_w)

        # Proteção lateral (antecipa e evita flip-flop)
        if now < self.explore_lateral_lock_until and self.explore_lateral_lock_dir != 0.0:
            return (speed_explore * 0.1, self.explore_lateral_lock_dir * turn_force * avoid_gain)

        if lat_left < lateral_avoid or lat_right < lateral_avoid:
            if self.explore_lateral_blocked_since == 0.0:
                self.explore_lateral_blocked_since = now
            if (now - self.explore_lateral_blocked_since) > lateral_block_time:
                self._start_explore_escape(left, right)
                return (self.explore_escape_v, self.explore_escape_w)

            if abs(lat_left - lat_right) < 0.05:
                if (now - self.explore_last_turn_time) < lateral_turn_cooldown and self.explore_last_turn_dir != 0.0:
                    turn_dir = self.explore_last_turn_dir
                else:
                    turn_dir = -1.0 if time.time_ns() % 2 == 0 else 1.0
            else:
                turn_dir = -1.0 if lat_left < lat_right else 1.0

            self.explore_last_turn_dir = turn_dir
            self.explore_last_turn_time = now
            self.explore_lateral_lock_dir = turn_dir
            self.explore_lateral_lock_until = now + lateral_turn_hold
            return (speed_explore * 0.05, turn_dir * turn_force * avoid_gain)
        else:
            self.explore_lateral_blocked_since = 0.0

        # Emergência
        if front < dist_stop:
            w = (-turn_force if left < right else turn_force) * avoid_gain
            self.tail_blind_time = now + tail_cooldown
            return (0.0, w)

        # Cooldown de cauda
        if now < self.tail_blind_time:
            self.explore_last_move_time = now
            return (speed_explore, 0.0)

        # Exploração com giro travado
        if self.is_turning and now < self.turn_end_time:
            return (speed_explore * 0.4, self.locked_turn_z)
        if left < dist_avoid or right < dist_avoid:
            self.is_turning = True
            self.turn_end_time = now + turn_duration
            speed = 0.9 * turn_force * avoid_gain
            self.locked_turn_z = -speed if left < right else speed
            return (0.0, self.locked_turn_z)

        self.is_turning = False
        self.explore_last_move_time = now
        return (speed_explore, 0.0)

    def _start_explore_escape(self, left, right):
        turn_force = float(self.get_parameter('turn_force').value)
        avoid_gain = float(self.get_parameter('avoid_gain').value)
        speed_explore = self._base_speed()
        if abs(left - right) < 0.05:
            turn_dir = -1.0 if time.time_ns() % 2 == 0 else 1.0
        else:
            turn_dir = -1.0 if left < right else 1.0
        self.explore_escape_v = -abs(speed_explore) * 0.6
        self.explore_escape_w = turn_dir * turn_force * avoid_gain
        self.explore_escape_until = time.time() + 1.2

    def control_loop(self):
        """Loop de alta frequência para execução de comandos"""
        if not self.current_task:
            if self.explore_active:
                cmd = self._explore_cmd()
                if cmd is None:
                    self.cmd_vel_pub.publish(Twist())
                    self.status_pub.publish(Bool(data=False))
                    return
                msg = Twist()
                lin, ang = self._clamp_cmd(float(cmd[0]), float(cmd[1]))
                msg.linear.x = lin
                msg.angular.z = ang
                self.cmd_vel_pub.publish(msg)
                self.status_pub.publish(Bool(data=True))
                return
            # ESTADO IDLE: Força o Brain a parar publicando ZEROS constantemente
            self.cmd_vel_pub.publish(Twist()) 
            self.status_pub.publish(Bool(data=False))
            return
            
        elapsed = time.time() - self.task_start_time
        task_type = self.current_task.get('type', 'timed_cmd')

        if task_type == 'move_distance':
            if self.odom_pose is None:
                self._finish_task("missing_odom")
                return
            if elapsed > max(float(self.current_task.get('timeout_s', 0.0)), 0.5):
                self._finish_task("timeout")
                return
            target = float(self.current_task['target'])
            progress = self._distance_progress(self.current_task['start_pose'], self.odom_pose)
            error = target - progress
            eps = abs(float(self.get_parameter('linear_epsilon').value))
            if abs(error) <= eps:
                self._finish_task("target_reached")
                return
            cmd_speed = self._scaled_linear_speed(error)
            cmd = Twist()
            cmd.linear.x, cmd.angular.z = self._clamp_cmd(
                math.copysign(cmd_speed, error),
                0.0,
            )
            self.cmd_vel_pub.publish(cmd)
            self.status_pub.publish(Bool(data=True))
            return

        if task_type == 'turn_angle':
            if self.odom_pose is None:
                self._finish_task("missing_odom")
                return
            if elapsed > max(float(self.current_task.get('timeout_s', 0.0)), 0.5):
                self._finish_task("timeout")
                return
            target = float(self.current_task['target_rad'])
            progress = self._rotation_progress(self.current_task['start_yaw'], self.odom_pose['yaw'])
            error = self._normalize_angle(target - progress)
            eps = math.radians(abs(float(self.get_parameter('angular_epsilon_deg').value)))
            if abs(error) <= eps:
                self._finish_task("target_reached")
                return
            cmd_speed = self._scaled_angular_speed(error)
            cmd = Twist()
            cmd.linear.x, cmd.angular.z = self._clamp_cmd(
                0.0,
                math.copysign(cmd_speed, error),
            )
            self.cmd_vel_pub.publish(cmd)
            self.status_pub.publish(Bool(data=True))
            return

        if elapsed < self.current_task['duration']:
            msg = Twist()
            lin, ang = self._clamp_cmd(float(self.current_task['lin']), float(self.current_task['ang']))
            msg.linear.x = lin
            msg.angular.z = ang
            self.cmd_vel_pub.publish(msg)
            self.status_pub.publish(Bool(data=True))
            return

        self._finish_task("duration_elapsed")

def main(args=None):
    rclpy.init(args=args)
    node = P3ATExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

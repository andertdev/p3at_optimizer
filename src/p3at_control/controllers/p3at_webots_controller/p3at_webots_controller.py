#!/usr/bin/env python3
# ---------------------------------------------------------
# P3AT Webots Controller – FINAL (Validator + Optimizer READY)
# ---------------------------------------------------------

from controller import Supervisor
import socket
import json
import math
import time
import os
import random

# =========================
# CONFIG
# =========================
TIME_STEP = 64

UDP_CMD_IP = "0.0.0.0"
UDP_CMD_PORT = 20001

UDP_STATUS_IP = "127.0.0.1"
UDP_STATUS_PORT = 20002

WHEEL_RADIUS = 0.11
WHEEL_BASE = 0.33
MAX_WHEEL_SPEED = 6.28

MOVE_SPEED = 0.30
TURN_SPEED = 0.90

DIST_STOP = 0.70
DIST_AVOID = 1.40
LATERAL_AVOID_FACTOR = 1.25
LATERAL_TURN_COOLDOWN = 1.2
SENSOR_MAX_RANGE = 5.0
LATERAL_TRIGGER_FRAC = 0.35  # aciona desvio quando <= 35% do alcance
FRONT_TRIGGER_FRAC = 0.30
LATERAL_TURN_HOLD = 0.9
CALIB_LOG_INTERVAL = 0.10

MAX_CRASHES = 3
STUCK_TIME = 2.5
EXPLORE_STUCK_TIME = 1.0
EXPLORE_FRONT_BLOCKED_TIME = 0.6
EXPLORE_ESCAPE_TIME = 1.2
TASK_LATERAL_SOFT_MIN = 0.85
TASK_LATERAL_HARD_MIN = 0.55
TASK_ESCAPE_TIME = 1.1
TASK_ESCAPE_COOLDOWN = 0.7
TASK_SPIN_LOCK_TIME = 0.45
TURN_BLOCKED_TIMEOUT = 1.2

# =========================
def clamp(x, a, b):
    return max(a, min(b, x))

def now():
    return time.time()

def deg2rad(d):
    return d * math.pi / 180.0

def parse_bool(v):
    if isinstance(v, bool):
        return v
    if isinstance(v, (int, float)):
        return v != 0
    if isinstance(v, str):
        return v.strip().lower() in ("1", "true", "yes", "on")
    return False

# =========================
class P3ATController(Supervisor):

    def __init__(self):
        super().__init__()
        self.timestep = int(self.getBasicTimeStep())
        
        self.boot_t0 = now()
        self.BOOT_GRACE_S = 2.0

        # --- Status UDP ---
        self.status_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.validator_addr = (UDP_STATUS_IP, UDP_STATUS_PORT)

        # --- Command UDP ---
        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cmd_sock.bind((UDP_CMD_IP, UDP_CMD_PORT))
        self.cmd_sock.setblocking(False)
        self.connection_established = False
        print(f"[P3AT] Aguardando comandos UDP em {UDP_CMD_IP}:{UDP_CMD_PORT}")

        # --- Devices ---
        self.motors = []
        for name in [
            "front left wheel","front right wheel",
            "back left wheel","back right wheel"
        ]:
            m = self.getDevice(name)
            m.setPosition(float("inf"))
            m.setVelocity(0.0)
            self.motors.append(m)

        self.enc_l = self.getDevice("front left wheel sensor")
        self.enc_r = self.getDevice("front right wheel sensor")
        self.enc_l.enable(self.timestep)
        self.enc_r.enable(self.timestep)
        self.prev_l = self.enc_l.getValue()
        self.prev_r = self.enc_r.getValue()

        self.sonars = []
        for i in range(16):
            s = self.getDevice(f"so{i}")
            s.enable(self.timestep)
            self.sonars.append(s)

        # --- Genes ---
        self.genes = {
            "dist_stop": DIST_STOP,
            "dist_avoid": DIST_AVOID,
            "turn_force": 1.2,

            # NOVOS GENES OTIMIZÁVEIS
            "base_speed": MOVE_SPEED,     # velocidade linear base
            "turn_speed": TURN_SPEED,     # velocidade angular base
            "avoid_gain": 1.2,            # ganho do desvio
        }

        # --- State ---
        self.task = None
        self.explore = False
        self.is_turning = False
        self.turn_end_time = 0.0
        self.tail_blind_time = 0.0
        self.tail_shielding_enabled = parse_bool(os.environ.get("P3AT_TAIL_SHIELDING", "1"))
        self.locked_turn_z = 0.0
        self.TAIL_COOLDOWN = 0.4
        self.TURN_DURATION = 0.7
        self.explore_escape_until = 0.0
        self.explore_escape_v = 0.0
        self.explore_escape_w = 0.0
        self.explore_front_blocked_since = 0.0
        self.explore_last_turn_dir = 0.0
        self.explore_last_turn_time = 0.0
        self.explore_lateral_lock_until = 0.0
        self.explore_lateral_lock_dir = 0.0
        self.explore_lateral_blocked_since = 0.0
        self._last_sensor_log_time = 0.0
        self._last_sonar_vals = None
        self._last_sonar_raws = None
        # Default ligado para facilitar calibracao; pode desativar com P3AT_SENSOR_CALIB_LOG=0.
        self.sensor_calib_log = parse_bool(os.environ.get("P3AT_SENSOR_CALIB_LOG", "1"))
        self._calib_last_log_time = 0.0
        self._calib_front_ref = None
        self._calib_marks_hit = set()
        self.crash_count = 0
        self.collision_hits = 0
        self.last_motion_time = now()
        self.booting = True
        self.boot_t0 = now()
        self.task_metrics = None
        self.validation_mode = False  # True para BEFORE/AFTER (sem desvio durante task)
        self.stop_requested = False
        self.task_escape_until = 0.0
        self.task_escape_cooldown_until = 0.0
        self.task_escape_v = 0.0
        self.task_escape_w = 0.0
        self.task_lateral_blocked_since = 0.0
        self.task_spin_lock_since = 0.0
        self.task_corner_blocked_since = 0.0
        self.task_corner_clear_since = 0.0
        self.task_escape_count = 0
        self.task_force_reset = False
        self.task_turn_blocked_since = 0.0

        self.robot_node = self.getFromDef("PIONEER_3AT")
        self.start_translation = self.robot_node.getField("translation").getSFVec3f()
        self.start_rotation = self.robot_node.getField("rotation").getSFRotation()

        self.start_rotation = self.robot_node.getField("rotation").getSFRotation()
        self.explore_prev_pos = list(self.start_translation)
        self.explore_prev_yaw = self._get_yaw()
        self.explore_last_move_time = now()

        # --- STUCK por pose REAL (Supervisor) ---
        self.prev_pose_t = now()
        self.prev_pos = list(self.start_translation)  # [x,y,z]
        self.prev_yaw = 0.0

        # thresholds (ajuste fino se quiser)
        self.STUCK_POS_EPS = 0.005    # considera parado mesmo com pequena oscilação
        self.STUCK_YAW_EPS = 0.01     # ~0.57 deg por step (rad)

        print(f"[P3AT] Controller iniciado ({__file__})")
        print(f"[P3AT] tail_shielding={'on' if self.tail_shielding_enabled else 'off'}")
        if self.sensor_calib_log:
            print("[P3AT][CALIB] sensor calibration log enabled (P3AT_SENSOR_CALIB_LOG=1)")

    def _params_yaml_path(self):
        base_dir = os.path.dirname(os.path.abspath(__file__))
        return os.path.normpath(os.path.join(base_dir, "..", "..", "config", "params.yaml"))

    def _persist_genes_to_params(self):
        path = self._params_yaml_path()
        try:
            with open(path, "r", encoding="utf-8") as f:
                lines = f.readlines()
        except Exception as e:
            print(f"[P3AT] Falha ao ler params.yaml: {e}")
            return

        def fmt(v):
            try:
                return f"{float(v):.2f}"
            except Exception:
                return str(v)

        in_executor = False
        in_ros_params = False
        for i, line in enumerate(lines):
            stripped = line.strip()
            if stripped.endswith(":") and not line.startswith(" "):
                in_executor = stripped == "executor:"
                in_ros_params = False
                continue
            if in_executor and stripped.startswith("ros__parameters:"):
                in_ros_params = True
                continue
            if in_executor and in_ros_params and stripped and not line.startswith("    "):
                in_ros_params = False

            if not (in_executor and in_ros_params):
                continue

            if stripped.startswith("dist_stop:"):
                lines[i] = f"    dist_stop: {fmt(self.genes['dist_stop'])}\n"
            elif stripped.startswith("dist_avoid:"):
                lines[i] = f"    dist_avoid: {fmt(self.genes['dist_avoid'])}\n"
            elif stripped.startswith("turn_force:"):
                lines[i] = f"    turn_force: {fmt(self.genes['turn_force'])}\n"
            elif stripped.startswith("explore_speed:"):
                lines[i] = f"    explore_speed: {fmt(self.genes['base_speed'])}\n"

        try:
            with open(path, "w", encoding="utf-8") as f:
                f.writelines(lines)
            print(f"[P3AT] params.yaml atualizado com genes otimizados ({path})")
        except Exception as e:
            print(f"[P3AT] Falha ao escrever params.yaml: {e}")

    # =========================
    def send_status(self, state, extra=None):
        msg = {
            "type": "status",
            "task_id": self.task["task_id"] if self.task else None,
            "state": state,
            "collision": self.crash_count > 0,
        }
        if extra:
            msg.update(extra)
        self.status_sock.sendto(json.dumps(msg).encode(), self.validator_addr)

    def send_reset_event(self):
        if not self.task:
            if not self.connection_established:
                print(f"[P3AT] Conexão UDP estabelecida ({UDP_CMD_IP}:{UDP_CMD_PORT})")
                self.connection_established = True
            return
        msg = {
            "type": "event",
            "event": "reset",
            "task_id": self.task["task_id"],
        }
        self.status_sock.sendto(json.dumps(msg).encode(), self.validator_addr)

    def _get_pos(self):
        return self.robot_node.getField("translation").getSFVec3f()

    def _get_yaw(self):
        # rotation = [ax, ay, az, angle]; para robô planar, yaw ~ angle * az
        r = self.robot_node.getField("rotation").getSFRotation()
        ax, ay, az, ang = float(r[0]), float(r[1]), float(r[2]), float(r[3])
        return ang * az

    def _wrap_pi(self, a):
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    # =========================
    def apply(self, v, w):
        left = clamp((v - w*WHEEL_BASE/2)/WHEEL_RADIUS, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED)
        right = clamp((v + w*WHEEL_BASE/2)/WHEEL_RADIUS, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED)

        self.motors[0].setVelocity(left)
        self.motors[2].setVelocity(left)
        self.motors[1].setVelocity(right)
        self.motors[3].setVelocity(right)

    def odom(self):
        l = self.enc_l.getValue()
        r = self.enc_r.getValue()
        dl = (l - self.prev_l) * WHEEL_RADIUS
        dr = (r - self.prev_r) * WHEEL_RADIUS
        self.prev_l = l
        self.prev_r = r
        return (dl+dr)/2, (dr-dl)/WHEEL_BASE

    def read_sonars(self):
        vals = []
        raws = []
        for s in self.sonars:
            raw = float(s.getValue())
            raws.append(raw)
            maxv = float(s.getMaxValue()) if hasattr(s, "getMaxValue") else 0.0

            if maxv > 0.0:
                raw = clamp(raw, 0.0, maxv)
                # converte leitura em distância (assume 0=longe, maxv=perto)
                dist = SENSOR_MAX_RANGE * (1.0 - (raw / maxv))
                vals.append(clamp(dist, 0.02, SENSOR_MAX_RANGE))
            else:
                # fallback: assume leitura já está em metros
                if raw <= 0.0:
                    raw = SENSOR_MAX_RANGE
                vals.append(clamp(raw, 0.02, SENSOR_MAX_RANGE))
    
        self._last_sonar_vals = vals
        self._last_sonar_raws = raws
        front = min(vals[3], vals[4])
        left  = min(vals[1], vals[2])
        right = min(vals[5], vals[6])
        # Laterais externas (s0/s15 esquerda, s7/s8 direita)
        lat_left = min(vals[0], vals[15])
        lat_right = min(vals[7], vals[8])
        back = None
        if len(vals) >= 16:
            back = min(vals[10], vals[11], vals[12], vals[13])
        return front, left, right, back, lat_left, lat_right

    def avoid(self, front, left, right, lat_left=None, lat_right=None):
        gain = self.genes["avoid_gain"]
        dist_stop_eff = self.genes["dist_stop"]
        dist_avoid_eff = self.genes["dist_avoid"]

        # Guardrail lateral durante task de movimento:
        # evita raspagem prolongada em parede/cilindro quando a frente "abre".
        if lat_left is not None and lat_right is not None:
            lat_soft = max(dist_stop_eff, TASK_LATERAL_SOFT_MIN)
            lat_hard = max(dist_stop_eff * 0.8, TASK_LATERAL_HARD_MIN)
            min_lat = min(lat_left, lat_right)
            if min_lat < lat_soft:
                away_dir = 1.0 if lat_right < lat_left else -1.0  # +w afasta da direita, -w afasta da esquerda
                w_cmd = self.genes["turn_force"] * gain
                if min_lat < lat_hard:
                    # muito perto: recua curto + gira forte para descolar.
                    return (-0.10 * self.genes["base_speed"], away_dir * 1.25 * w_cmd)
                # perto, mas não crítico: avança devagar e corrige heading.
                return (0.15 * self.genes["base_speed"], away_dir * 0.95 * w_cmd)

        if front < dist_stop_eff:
            w = self.genes["turn_force"] * gain
            return 0.0, -w if left < right else w

        if left < dist_avoid_eff:
            return (
                self.genes["base_speed"] * 0.5,
                -0.6 * self.genes["turn_force"] * gain
            )

        if right < dist_avoid_eff:
            return (
                self.genes["base_speed"] * 0.5,
                0.6 * self.genes["turn_force"] * gain
            )
        return None

    def explore_cmd(self, front, left, right, back, lat_left, lat_right):
        now_t = now()
        dist_stop = self.genes["dist_stop"]
        dist_avoid = self.genes["dist_avoid"]
        turn_force = self.genes["turn_force"]
        speed_explore = self.genes["base_speed"]
        lateral_avoid = dist_avoid * LATERAL_AVOID_FACTOR
        lateral_trigger = min(lateral_avoid, SENSOR_MAX_RANGE * LATERAL_TRIGGER_FRAC)
        front_trigger = min(dist_stop, SENSOR_MAX_RANGE * FRONT_TRIGGER_FRAC)

        # Escape travado (anti-loop)
        if now_t < self.explore_escape_until:
            return (self.explore_escape_v, self.explore_escape_w)

        # Stuck detection durante explore
        if (now_t - self.explore_last_move_time) > EXPLORE_STUCK_TIME:
            self._start_explore_escape(front, left, right, back)
            return (self.explore_escape_v, self.explore_escape_w)

        # Proteção lateral para evitar raspadas/flip-flop
        if now_t < self.explore_lateral_lock_until and self.explore_lateral_lock_dir != 0.0:
            return (speed_explore * 0.1, self.explore_lateral_lock_dir * turn_force)

        if lat_left < lateral_trigger or lat_right < lateral_trigger:
            if self.explore_lateral_blocked_since == 0.0:
                self.explore_lateral_blocked_since = now_t
            if (now_t - self.explore_lateral_blocked_since) > 0.8:
                self._start_explore_escape(front, left, right, back)
                return (self.explore_escape_v, self.explore_escape_w)

            if abs(lat_left - lat_right) < 0.05:
                if (now_t - self.explore_last_turn_time) < LATERAL_TURN_COOLDOWN and self.explore_last_turn_dir != 0.0:
                    turn_dir = self.explore_last_turn_dir
                else:
                    turn_dir = random.choice([-1.0, 1.0])
            else:
                turn_dir = -1.0 if lat_left < lat_right else 1.0

            self.explore_last_turn_dir = turn_dir
            self.explore_last_turn_time = now_t
            self.explore_lateral_lock_dir = turn_dir
            self.explore_lateral_lock_until = now_t + LATERAL_TURN_HOLD
            return (speed_explore * 0.05, turn_dir * turn_force)
        else:
            self.explore_lateral_blocked_since = 0.0

        # Emergência
        if front < max(dist_stop, front_trigger):
            if self.explore_front_blocked_since == 0.0:
                self.explore_front_blocked_since = now_t
            if (now_t - self.explore_front_blocked_since) > EXPLORE_FRONT_BLOCKED_TIME:
                self._start_explore_escape(front, left, right, back)
                return (self.explore_escape_v, self.explore_escape_w)

            if abs(left - right) < 0.05:
                w = turn_force * random.choice([-1.0, 1.0])
            else:
                w = -turn_force if left < right else turn_force
            if self.tail_shielding_enabled:
                self.tail_blind_time = now_t + self.TAIL_COOLDOWN
            return (0.0, w)
        else:
            self.explore_front_blocked_since = 0.0

        # Cooldown de cauda
        if self.tail_shielding_enabled and now_t < self.tail_blind_time:
            return (speed_explore, 0.0)

        # Exploração com giro travado
        if self.is_turning and now_t < self.turn_end_time:
            return (speed_explore * 0.4, self.locked_turn_z)
        if left < dist_avoid or right < dist_avoid:
            self.is_turning = True
            self.turn_end_time = now_t + self.TURN_DURATION
            speed = 0.9 * turn_force
            if abs(left - right) < 0.05:
                self.locked_turn_z = speed * random.choice([-1.0, 1.0])
            else:
                self.locked_turn_z = -speed if left < right else speed
            return (0.0, self.locked_turn_z)

        self.is_turning = False
        return (speed_explore, 0.0)

    def _reset_explore_state(self):
        self.is_turning = False
        self.turn_end_time = 0.0
        self.tail_blind_time = 0.0
        self.locked_turn_z = 0.0
        self.explore_escape_until = 0.0
        self.explore_escape_v = 0.0
        self.explore_escape_w = 0.0
        self.explore_front_blocked_since = 0.0
        self.explore_prev_pos = list(self._get_pos())
        self.explore_prev_yaw = self._get_yaw()
        self.explore_last_move_time = now()
        self.explore_lateral_lock_until = 0.0
        self.explore_lateral_lock_dir = 0.0
        self.explore_lateral_blocked_since = 0.0

    def _update_explore_motion(self):
        pos = self._get_pos()
        yaw = self._get_yaw()

        dx = float(pos[0]) - float(self.explore_prev_pos[0])
        dy = float(pos[1]) - float(self.explore_prev_pos[1])
        dpos = math.hypot(dx, dy)

        self.explore_prev_pos = list(pos)
        self.explore_prev_yaw = yaw

        moved_linear = dpos > self.STUCK_POS_EPS
        if moved_linear:
            self.explore_last_move_time = now()
        return moved_linear

    def _start_explore_escape(self, front, left, right, back):
        now_t = now()
        dist_stop = self.genes["dist_stop"]
        turn_force = self.genes["turn_force"]
        speed_explore = self.genes["base_speed"]

        if abs(left - right) < 0.05:
            turn_dir = random.choice([-1.0, 1.0])
        else:
            turn_dir = -1.0 if left < right else 1.0

        back_clear = (back is None) or (back > dist_stop)
        if back_clear:
            self.explore_escape_v = -abs(speed_explore) * 0.6
            self.explore_escape_w = turn_dir * turn_force * (0.7 + random.random() * 0.4)
        else:
            self.explore_escape_v = 0.0
            self.explore_escape_w = turn_dir * turn_force * (0.9 + random.random() * 0.4)

        self.explore_escape_until = now_t + EXPLORE_ESCAPE_TIME

    def reset_pose(self):
        print("🔄 RESET automático")
        
        # Só avisa o validator se havia uma task em execução
        if self.task:
            self.send_reset_event()
            self.task = None   # 🔴 importante: cancela a task atual

        # Reset físico do robô
        self.robot_node.getField("translation").setSFVec3f(self.start_translation)
        self.robot_node.getField("rotation").setSFRotation(self.start_rotation)
        self.simulationResetPhysics()

        # reancora referência de pose real após reset
        self.prev_pos = list(self.start_translation)
        self.prev_yaw = self._get_yaw()

        # Zera estado interno
        self.crash_count = 0
        self.last_motion_time = now()
        self.task_metrics = None
        self._reset_task_escape_state()

    # =========================
    def poll_udp(self):
        while True:
            try:
                data, _ = self.cmd_sock.recvfrom(4096)
                cmd = json.loads(data.decode())
            except:
                return

            c = cmd.get("command")
            v = cmd.get("value")
            tid = cmd.get("task_id")
            print(f"[P3AT] CMD recebido: {c} value={v}")

            if c == "stop":
                self.task = None
                self._reset_calib_session()
                self._reset_task_escape_state()
                self.explore = False
                self._reset_explore_state()
                self.stop_requested = True
                self.send_status("stopped")
                # continua drenando para priorizar stop
                continue

            if c == "set_genes":
                for k in self.genes:
                    if k in v:
                        self.genes[k] = float(v[k])
                self._persist_genes_to_params()
                continue

            if c in ("explore_mode", "explore"):
                # aceita value ausente, bool, string ou numero
                self.explore = parse_bool(True if v is None else v)
                self.task = None
                self._reset_calib_session()
                self._reset_task_escape_state()
                self._reset_explore_state()
                if self.explore:
                    # garantir que explore não fique bloqueado por modo de validação
                    self.validation_mode = False
                    self.send_status("explore_on")
                else:
                    self.send_status("explore_off")
                continue

            if c == "tail_shielding":
                self.tail_shielding_enabled = parse_bool(v)
                self.tail_blind_time = 0.0
                print(f"[P3AT] tail_shielding set to {'on' if self.tail_shielding_enabled else 'off'}")
                continue

            if c == "validation_mode":
                # True para BEFORE/AFTER; False para otimização (genes)
                self.validation_mode = parse_bool(v)
                continue

            if c == "move_distance":
                self.task = {"type":"move","target":float(v),"progress":0.0,"task_id":tid}
                self.collision_hits = 0
                self._reset_calib_session()
                self._reset_task_escape_state()
                self._start_task_metrics()
                self.send_status("running")
                continue

            if c == "turn_angle":
                self.task = {"type":"turn","target":deg2rad(v),"progress":0.0,"task_id":tid}
                self.collision_hits = 0
                self._reset_calib_session()
                self._reset_task_escape_state()
                self._start_task_metrics()
                self.send_status("running")
                continue

            if c == "arc":
                # valor esperado: {"v": .., "w": .., "duration": ..}
                if isinstance(v, dict):
                    self.task = {
                        "type": "arc",
                        "v": float(v.get("v", 0.0)),
                        "w": float(v.get("w", 0.0)),
                        "duration": float(v.get("duration", 0.0)),
                        "elapsed": 0.0,
                        "task_id": tid,
                    }
                    self.collision_hits = 0
                    self._reset_calib_session()
                    self._reset_task_escape_state()
                    self._start_task_metrics()
                self.send_status("running")
                continue

    # =========================
    def _start_task_metrics(self):
        self.task_metrics = {
            "t_start": now(),
            "t_last": now(),
            "stopped_time": 0.0,
            "avoid_time": 0.0,
            "min_front": 5.0,
            "min_left": 5.0,
            "min_right": 5.0,
        }

    def _reset_calib_session(self):
        self._calib_front_ref = None
        self._calib_marks_hit = set()

    def _reset_task_escape_state(self):
        self.task_escape_until = 0.0
        self.task_escape_cooldown_until = 0.0
        self.task_escape_v = 0.0
        self.task_escape_w = 0.0
        self.task_lateral_blocked_since = 0.0
        self.task_spin_lock_since = 0.0
        self.task_corner_blocked_since = 0.0
        self.task_corner_clear_since = 0.0
        self.task_escape_count = 0
        self.task_force_reset = False
        self.task_turn_blocked_since = 0.0

    def _start_task_escape(self, left, right, lat_left, lat_right, back, dist_stop_eff):
        now_t = now()
        turn_force = self.genes["turn_force"]
        base_speed = self.genes["base_speed"]

        # Direção para afastar da lateral bloqueada.
        if abs(lat_left - lat_right) > 0.03:
            turn_dir = 1.0 if lat_right < lat_left else -1.0
        elif abs(left - right) > 0.03:
            turn_dir = 1.0 if right < left else -1.0
        else:
            turn_dir = random.choice([-1.0, 1.0])

        back_close = (back is not None) and (back < max(dist_stop_eff * 0.8, 0.25))
        # Se traseira está bloqueada, evita insistir em ré e aplica avanço curto com giro forte.
        if back_close:
            self.task_escape_v = abs(base_speed) * 0.18
        else:
            self.task_escape_v = -abs(base_speed) * 0.45
        self.task_escape_w = turn_dir * turn_force * 1.35
        self.task_escape_until = now_t + TASK_ESCAPE_TIME
        self.task_escape_cooldown_until = self.task_escape_until + TASK_ESCAPE_COOLDOWN
        self.task_lateral_blocked_since = 0.0
        self.task_spin_lock_since = 0.0
        self.task_corner_blocked_since = 0.0
        self.task_corner_clear_since = 0.0
        self.task_escape_count += 1
        print(f"⚠️ TASK_ESCAPE {self.task_escape_count}/{MAX_CRASHES} (back_close={back_close})")
        # Se ficar "engatando" em sequência, reseta a pose para não contaminar avaliação.
        if self.task_escape_count >= MAX_CRASHES:
            self.task_force_reset = True

    def _log_sensor_calib(self, front, left, right, back, lat_left, lat_right):
        if not self.sensor_calib_log:
            return
        if not self.task or self.task.get("type") != "move":
            return
        now_t = now()
        if (now_t - self._calib_last_log_time) < CALIB_LOG_INTERVAL:
            return
        self._calib_last_log_time = now_t

        vals = self._last_sonar_vals or []
        raws = self._last_sonar_raws or []
        if not vals:
            return

        def _min_idx(idxs):
            best_i = idxs[0]
            best_v = vals[best_i]
            for i in idxs[1:]:
                if vals[i] < best_v:
                    best_v = vals[i]
                    best_i = i
            return best_i, best_v

        def _raw(i):
            try:
                return raws[i]
            except Exception:
                return 0.0

        fi, fv = _min_idx([3, 4])
        li, lv = _min_idx([1, 2])
        ri, rv = _min_idx([5, 6])
        if self._calib_front_ref is None:
            self._calib_front_ref = max(fv, 0.02)

        ref = max(self._calib_front_ref, 0.02)
        consumed = clamp(1.0 - (fv / ref), 0.0, 1.0)
        x, y, _ = self._get_pos()

        back_v = back if back is not None else -1.0
        print(
            f"[P3AT][CALIB] front=s{fi}:{fv:.3f}m(raw={_raw(fi):.1f}) "
            f"left=s{li}:{lv:.3f}m right=s{ri}:{rv:.3f}m "
            f"latL={lat_left:.3f}m latR={lat_right:.3f}m back={back_v:.3f}m "
            f"ref={ref:.3f}m consumed={consumed*100:.1f}% "
            f"dist_stop={self.genes['dist_stop']:.3f} dist_avoid={self.genes['dist_avoid']:.3f} "
            f"pose=({x:.3f},{y:.3f})"
        )

        # 25/50/75% da aproximacao consumida do valor inicial de referencia.
        for frac in (0.25, 0.50, 0.75):
            if consumed >= frac and frac not in self._calib_marks_hit:
                self._calib_marks_hit.add(frac)
                print(
                    f"[P3AT][CALIB][MARK] consumed={int(frac*100)}% "
                    f"remaining_front={fv:.3f}m"
                )

    def _update_task_metrics(self, front, left, right, v_cmd, avoid_active):
        if not self.task_metrics:
            return
        t_now = now()
        dt = max(0.0, t_now - self.task_metrics["t_last"])
        self.task_metrics["t_last"] = t_now

        self.task_metrics["min_front"] = min(self.task_metrics["min_front"], front)
        self.task_metrics["min_left"] = min(self.task_metrics["min_left"], left)
        self.task_metrics["min_right"] = min(self.task_metrics["min_right"], right)

        if abs(v_cmd) < 0.02:
            self.task_metrics["stopped_time"] += dt
        if avoid_active:
            self.task_metrics["avoid_time"] += dt

    def _finalize_task_metrics(self):
        if not self.task_metrics or not self.task:
            return {}
        total = max(0.001, self.task_metrics["t_last"] - self.task_metrics["t_start"])
        return {
            "min_front": self.task_metrics["min_front"],
            "min_left": self.task_metrics["min_left"],
            "min_right": self.task_metrics["min_right"],
            "stopped_time_frac": self.task_metrics["stopped_time"] / total,
            "avoid_time_frac": self.task_metrics["avoid_time"] / total,
            "duration_s": total,
        }

    # =========================
    def step_logic(self):
        self.poll_udp()
        if self.stop_requested:
            self.apply(0.0, 0.0)
            self.stop_requested = False
            return
        ds, dth = self.odom()
        front, left, right, back, lat_left, lat_right = self.read_sonars()
        dist_stop_eff = self.genes["dist_stop"]
        self._log_sensor_calib(front, left, right, back, lat_left, lat_right)
        if self.explore and self._last_sonar_vals and self._last_sonar_raws:
            now_t = now()
            if (now_t - self._last_sensor_log_time) >= 0.10:
                self._last_sensor_log_time = now_t
                vals = self._last_sonar_vals
                raws = self._last_sonar_raws
                def _min_idx(idxs):
                    best_i = idxs[0]
                    best_v = vals[best_i]
                    for i in idxs[1:]:
                        if vals[i] < best_v:
                            best_v = vals[i]
                            best_i = i
                    return best_i, best_v
                def _raw(i):
                    try:
                        return raws[i]
                    except Exception:
                        return 0.0
                fi, fv = _min_idx([3, 4])
                li, lv = _min_idx([1, 2])
                ri, rv = _min_idx([5, 6])
                lli, llv = _min_idx([0, 15])
                rri, rrv = _min_idx([7, 8])
                bi, bv = _min_idx([10, 11, 12, 13]) if len(vals) >= 16 else (-1, -1.0)
                print(
                    f"[P3AT][SENS] F:s{fi}={fv:.2f}(raw={_raw(fi):.1f}) "
                    f"L:s{li}={lv:.2f}(raw={_raw(li):.1f}) R:s{ri}={rv:.2f}(raw={_raw(ri):.1f}) "
                    f"LatL:s{lli}={llv:.2f}(raw={_raw(lli):.1f}) "
                    f"LatR:s{rri}={rrv:.2f}(raw={_raw(rri):.1f}) "
                    f"B:s{bi}={bv:.2f}(raw={_raw(bi):.1f})"
                )
        v_cmd = 0.0
        avoid_active = front < self.genes["dist_avoid"]

        # ----------------------------------
        # BOOT GUARD (anti-reset no startup)
        # ----------------------------------
        if self.booting:
            self.last_motion_time = now()
            if (now() - self.boot_t0) > 1.0:
                self.booting = False
            # Durante boot: não faz mais nada
            self.apply(0.0, 0.0)
            return
    
        # ----------------------------------
        # ANTI-DEADLOCK (após boot)
        # ----------------------------------
        moved = False  # assume parado até provar movimento
        dpos = 0.0
        dyaw = 0.0

        if self.task is not None:
            # STUCK baseado em deslocamento REAL (não encoder)
            pos = self._get_pos()
            yaw = self._get_yaw()

            dx = float(pos[0]) - float(self.prev_pos[0])
            dy = float(pos[1]) - float(self.prev_pos[1])
            dpos = math.hypot(dx, dy)

            dyaw = self._wrap_pi(yaw - self.prev_yaw)

            self.prev_pos = list(pos)
            self.prev_yaw = yaw

            moved = (dpos > self.STUCK_POS_EPS) or (abs(dyaw) > self.STUCK_YAW_EPS)
            # se frente bloqueada e deslocamento irrisório, força considerar "parado"
            if front < dist_stop_eff and dpos < (self.STUCK_POS_EPS * 2) and abs(dyaw) < (self.STUCK_YAW_EPS * 2):
                moved = False

            if moved:
                self.last_motion_time = now()
            elif now() - self.last_motion_time > STUCK_TIME:
                self.crash_count += 1
                print(f"⚠️ STUCK {self.crash_count}/{MAX_CRASHES}")
                self.last_motion_time = now()

            if self.crash_count >= MAX_CRASHES:
                self.reset_pose()
                return
        else:
            # fora de task, robô pode ficar parado sem penalização
            self.last_motion_time = now()
            self.crash_count = 0
            # mantém referência de pose atual para quando entrar em task
            self.prev_pos = list(self._get_pos())
            self.prev_yaw = self._get_yaw()
            self.collision_hits = 0

        # ----------------------------------
        # CONTADOR DE HITS (somente em task move, sem desvio)
        # ----------------------------------
        if self.task and self.task["type"] == "move":
            # considera hit se frente bloqueada e não houve movimento real no step
            front_blocked = front < dist_stop_eff
            if front_blocked and not moved:
                self.collision_hits += 1
                if self.collision_hits >= MAX_CRASHES:
                    print(f"🛑 Colisão repetida ({self.collision_hits}) -> reset tarefa")
                    self.reset_pose()
                    return
            else:
                # soltou do obstáculo
                self.collision_hits = 0

        # ----------------------------------
        # ESCAPE ANTI-LOOP EM TASK MOVE
        # ----------------------------------
        if self.task and self.task["type"] == "move" and (not self.validation_mode) and (not self.explore):
            now_t = now()

            # mantém comando de escape pelo tempo configurado
            if now_t < self.task_escape_until:
                self.apply(self.task_escape_v, self.task_escape_w)
                self._update_task_metrics(front, left, right, self.task_escape_v, True)
                return

            lateral_soft = max(dist_stop_eff, TASK_LATERAL_SOFT_MIN)
            lateral_hard = max(dist_stop_eff * 0.8, TASK_LATERAL_HARD_MIN)
            min_lat = min(lat_left, lat_right)
            min_side = min(lat_left, lat_right, left, right)
            back_close = (back is not None) and (back < max(dist_stop_eff * 0.8, 0.25))

            # bloqueio persistente de canto (lateral e/ou traseira) com histerese.
            corner_blocked = (min_side < lateral_hard) or back_close
            if corner_blocked:
                if self.task_corner_blocked_since == 0.0:
                    self.task_corner_blocked_since = now_t
                self.task_corner_clear_since = 0.0
                if min_side < lateral_hard and self.task_lateral_blocked_since == 0.0:
                    self.task_lateral_blocked_since = now_t
            else:
                self.task_lateral_blocked_since = 0.0
                if self.task_corner_clear_since == 0.0:
                    self.task_corner_clear_since = now_t
                elif (now_t - self.task_corner_clear_since) > 0.60:
                    self.task_corner_blocked_since = 0.0

            # gira sem sair do lugar + lateral próxima => loop em cilindro/parede
            spinning_in_place = ((min_side < lateral_soft) or back_close) and (dpos < (self.STUCK_POS_EPS * 1.2)) and (abs(dyaw) > (self.STUCK_YAW_EPS * 2.5))
            if spinning_in_place:
                if self.task_spin_lock_since == 0.0:
                    self.task_spin_lock_since = now_t
            else:
                self.task_spin_lock_since = 0.0

            trigger_by_lateral = (self.task_lateral_blocked_since > 0.0) and ((now_t - self.task_lateral_blocked_since) > 0.30)
            trigger_by_corner = (self.task_corner_blocked_since > 0.0) and ((now_t - self.task_corner_blocked_since) > 0.35)
            trigger_by_spin = (self.task_spin_lock_since > 0.0) and ((now_t - self.task_spin_lock_since) > TASK_SPIN_LOCK_TIME)
            out_of_cooldown = now_t >= self.task_escape_cooldown_until

            # Recuperou deslocamento e folga lateral: limpa contagem de engates.
            if moved and min_side > (lateral_soft + 0.10) and not back_close:
                self.task_escape_count = 0

            if out_of_cooldown and (trigger_by_lateral or trigger_by_corner or trigger_by_spin):
                self._start_task_escape(left, right, lat_left, lat_right, back, dist_stop_eff)
                if self.task_force_reset:
                    print("🛑 Task lateral loop persistente -> reset tarefa")
                    self.reset_pose()
                    return
                self.apply(self.task_escape_v, self.task_escape_w)
                self._update_task_metrics(front, left, right, self.task_escape_v, True)
                return

        # ----------------------------------
        # ANTI-TRAVA PARA TASK TURN
        # ----------------------------------
        if self.task and self.task["type"] == "turn" and (not self.explore):
            now_t = now()
            min_side = min(left, right, lat_left, lat_right)
            back_v = back if back is not None else SENSOR_MAX_RANGE
            proximity_blocked = min(front, min_side, back_v) < max(self.genes["dist_stop"] * 0.9, 0.45)
            poor_turn_progress = abs(dth) < 0.002

            if proximity_blocked and poor_turn_progress:
                if self.task_turn_blocked_since == 0.0:
                    self.task_turn_blocked_since = now_t
                elif (now_t - self.task_turn_blocked_since) > TURN_BLOCKED_TIMEOUT:
                    print("🛑 Turn task travada em obstáculo/canto -> reset tarefa")
                    self.reset_pose()
                    return
            else:
                self.task_turn_blocked_since = 0.0

        # ----------------------------------
        # DESVIO TEM PRIORIDADE
        # ----------------------------------
        # Desvio:
        # - permitido sempre quando fora de task
        # - permitido dentro da task APENAS se não estiver em modo de validação
        av = self.avoid(front, left, right, lat_left=lat_left, lat_right=lat_right)
        if av:
            allow_avoid = False
            if self.task is None:
                allow_avoid = True
            elif self.task.get("type") == "move" and (not self.validation_mode):
                # Em move, desvio continua ativo fora do validation_mode estrito.
                allow_avoid = True
            if allow_avoid and not self.explore:
                self.apply(*av)
                # se estamos em task e desviando, não queremos resetar crash_count por movimento falso
                self._update_task_metrics(front, left, right, av[0], True)
                return

        # ----------------------------------
        # TASK (move / turn)
        # ----------------------------------
        if self.task:
            if self.task["type"] == "move":
                self.task["progress"] += ds
                if abs(self.task["progress"]) >= abs(self.task["target"]):
                    extra = {
                        "distance_error_m": self.task["progress"] - self.task["target"],
                    }
                    extra.update(self._finalize_task_metrics())
                    self.send_status(
                        "done",
                        extra,
                    )
                    self.task = None
                    self._reset_task_escape_state()
                    self.task_metrics = None
                    self.apply(0.0, 0.0)
                else:
                    v_cmd = math.copysign(self.genes["base_speed"], self.task["target"])
                    self.apply(v_cmd, 0.0)

            elif self.task["type"] == "turn":
                self.task["progress"] += dth
                if abs(self.task["progress"]) >= abs(self.task["target"]):
                    extra = {
                        "angle_error_rad": self.task["progress"] - self.task["target"],
                    }
                    extra.update(self._finalize_task_metrics())
                    self.send_status(
                        "done",
                        extra,
                    )
                    self.task = None
                    self._reset_task_escape_state()
                    self.task_metrics = None
                    self.apply(0.0, 0.0)
                else:
                    w_cmd = math.copysign(self.genes["turn_speed"], self.task["target"])
                    self.apply(0.0, w_cmd)
            elif self.task["type"] == "arc":
                self.task["elapsed"] += (now() - self.task_metrics["t_last"]) if self.task_metrics else 0.0
                if self.task["elapsed"] >= self.task["duration"]:
                    extra = {}
                    extra.update(self._finalize_task_metrics())
                    self.send_status("done", extra)
                    self.task = None
                    self._reset_task_escape_state()
                    self.task_metrics = None
                    self.apply(0.0, 0.0)
                else:
                    v_cmd = clamp(self.task["v"], -self.genes["base_speed"] * 1.2, self.genes["base_speed"] * 1.2)
                    w_cmd = clamp(self.task["w"], -self.genes["turn_speed"] * 1.5, self.genes["turn_speed"] * 1.5)
                    self.apply(v_cmd, w_cmd)
            self._update_task_metrics(front, left, right, v_cmd, avoid_active)
            return
    
        # ----------------------------------
        # EXPLORE / IDLE
        # ----------------------------------
        if self.explore:
            self._update_explore_motion()
            v_cmd, w_cmd = self.explore_cmd(front, left, right, back, lat_left, lat_right)
            self.apply(v_cmd, w_cmd)
        else:
            self.apply(0.0, 0.0)

        # Atualiza métricas por task (no final do step para garantir dt correto)
        self._update_task_metrics(front, left, right, v_cmd, avoid_active)

# =========================
def main():
    ctl=P3ATController()
    while ctl.step(TIME_STEP)!=-1:
        ctl.step_logic()

if __name__=="__main__":
    main()

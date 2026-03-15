#!/usr/bin/env python3
# -------------------------------------------------------
# FILE: p3at_validator.py (FINAL - Controller-Centric)
# -------------------------------------------------------
# - Publica comandos em /p3at_target (std_msgs/String JSON)
# - Aguarda status UDP do controller (porta 20002)
# - Reenvia comando se houver RESET automático durante o teste
# - Continua do ponto onde falhou (não reinicia a suíte toda)
# - Agrega métricas e salva validation_{label}.json
# -------------------------------------------------------

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import argparse
import json
import math
import os
import socket
import statistics
import threading
import time
import uuid
import signal
from datetime import datetime
from typing import Dict, Any, Optional, Tuple, List

# =========================
# SHUTDOWN CONTROL
# =========================
SHUTDOWN_REQUESTED = False

# =========================
# CONFIG (alinhado ao controller)
# =========================
STATUS_UDP_BIND_IP = "0.0.0.0"
STATUS_UDP_PORT = 20002
STATUS_SOCKET_TIMEOUT_S = 0.25

ROS_TOPIC_TARGET = "/p3at_target"


# =========================
# Helpers
# =========================
def _signal_handler(sig, frame):
    global SHUTDOWN_REQUESTED
    SHUTDOWN_REQUESTED = True
    print("\n🛑 Shutdown solicitado (signal)")

signal.signal(signal.SIGINT, _signal_handler)
signal.signal(signal.SIGTERM, _signal_handler)

def now_s() -> float:
    return time.time()


def safe_float(x, default=0.0) -> float:
    try:
        return float(x)
    except Exception:
        return default


def rad2deg(r: float) -> float:
    return r * 180.0 / math.pi


# =========================
# Validator
# =========================
class P3ATValidator(Node):
    def __init__(
        self,
        environment: str = "webots",
        max_resets_per_attempt: int = 3,
        start_timeout_s: float = 5.0,
        exec_timeout_s: float = 60.0,
    ):
        super().__init__("p3at_validator_controller")

        self.environment = environment
        self.max_resets_per_attempt = int(max_resets_per_attempt)
        self.start_timeout_s = float(start_timeout_s)
        self.exec_timeout_s = float(exec_timeout_s)

        # ROS publisher
        self.pub = self.create_publisher(String, ROS_TOPIC_TARGET, 10)

        # UDP socket (recebe status do controller)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((STATUS_UDP_BIND_IP, STATUS_UDP_PORT))
        self.sock.settimeout(STATUS_SOCKET_TIMEOUT_S)

        self.get_logger().info(
            f"[VALIDATOR] Publicando em {ROS_TOPIC_TARGET} e ouvindo status UDP em {STATUS_UDP_BIND_IP}:{STATUS_UDP_PORT}"
        )


    # -------------------------
    # ROS send
    # -------------------------
    def send_cmd(self, command: str, value, task_id: str) -> None:
        payload = {"command": command, "value": value, "task_id": task_id}
        msg = String()
        msg.data = json.dumps(payload)
        self.pub.publish(msg)

    def send_control(self, command: str, value) -> None:
        payload = {"command": command, "value": value}
        msg = String()
        msg.data = json.dumps(payload)
        self.pub.publish(msg)

    def send_stop(self) -> None:
        # stop global (sem task_id obrigatório)
        msg = String()
        msg.data = json.dumps({"command": "stop", "value": 0})
        self.pub.publish(msg)

    # -------------------------
    # UDP receive
    # -------------------------
    def _recv_one(self) -> Optional[Dict[str, Any]]:
        if SHUTDOWN_REQUESTED:
            return None
        try:
            data, _ = self.sock.recvfrom(65535)
        except socket.timeout:
            return None
        except OSError:
            return None  # socket fechado
        except Exception:
            return None

        try:
            txt = data.decode("utf-8", errors="ignore").strip()
            if not txt:
                return None
            return json.loads(txt)
        except Exception:
            return None

    # -------------------------
    # Shutdown limpo
    # -------------------------
    def close(self):
        try:
            self.send_stop()
        except Exception:
            pass

        try:
            self.sock.close()
        except Exception:
            pass

    # -------------------------
    # Wait for task result
    # -------------------------
    def run_one_attempt(
        self,
        test_name: str,
        command: str,
        value,
        per_attempt_timeout_s: Optional[float] = None,
    ) -> Dict[str, Any]:
        """
        Executa UMA tentativa.
        Reenvia o comando se receber evento 'reset' do controller para este task_id.
        Retorna dict com:
          ok, state, duration_s, resets, collisions, distance_error_m, angle_error_deg
        """
        task_id = uuid.uuid4().hex
        t0 = now_s()
        deadline_start = t0 + self.start_timeout_s
        deadline_exec = t0 + (per_attempt_timeout_s or self.exec_timeout_s)

        resets = 0
        collisions = 0
        got_any_status = False

        # Envia o comando inicial
        self.send_cmd(command, value, task_id)

        # Loop de espera
        last_state = None
        last_metrics: Dict[str, Any] = {}

        while True:
            if SHUTDOWN_REQUESTED:
                self.send_stop()
                return {
                    "ok": False,
                    "state": "shutdown",
                    "duration_s": now_s() - t0,
                    "resets": resets,
                    "collisions": collisions,
                    "distance_error_m": None,
                    "angle_error_deg": None,
                    "task_id": task_id,
                }

            t = now_s()
            if t > deadline_exec:
                # timeout duro
                self.send_stop()
                return {
                    "ok": False,
                    "state": "timeout",
                    "duration_s": t - t0,
                    "resets": resets,
                    "collisions": collisions,
                    "distance_error_m": None,
                    "angle_error_deg": None,
                    "task_id": task_id,
                }

            msg = self._recv_one()

            if msg is None:
                # Se não chegou nada ainda, dá um tempo para "começar"
                if (not got_any_status) and (now_s() > deadline_start):
                    # Sem status do controller -> algo desconectado
                    self.send_stop()
                    return {
                        "ok": False,
                        "state": "no_status",
                        "duration_s": now_s() - t0,
                        "resets": resets,
                        "collisions": collisions,
                        "distance_error_m": None,
                        "angle_error_deg": None,
                        "task_id": task_id,
                    }
                continue

            mtype = msg.get("type")
            mid = msg.get("task_id")

            # Evento de reset
            if mtype == "event" and msg.get("event") == "reset" and mid == task_id:
                resets += 1
                if resets <= self.max_resets_per_attempt:
                    # Reenvia o mesmo comando e continua aguardando
                    self.get_logger().warn(
                        f"[{test_name}] RESET {resets}/{self.max_resets_per_attempt} -> reenviando comando (task_id={task_id})"
                    )
                    self.send_cmd(command, value, task_id)
                    continue
                else:
                    self.get_logger().error(
                        f"[{test_name}] Excedeu resets ({resets}) -> abortando tentativa (task_id={task_id})"
                    )
                    self.send_stop()
                    return {
                        "ok": False,
                        "state": "too_many_resets",
                        "duration_s": now_s() - t0,
                        "resets": resets,
                        "collisions": collisions,
                        "distance_error_m": None,
                        "angle_error_deg": None,
                        "task_id": task_id,
                    }

            # Status periódico
            if mtype == "status" and mid == task_id:
                got_any_status = True

                # colisão (sinal do controller)
                if bool(msg.get("collision", False)):
                    collisions += 1

                state = (msg.get("state") or "").lower()
                last_state = state
                last_metrics = msg

                # terminou?
                if state in ("done", "crashed", "aborted", "stopped"):
                    duration = now_s() - t0

                    # extrai métricas (o controller manda em metros/rad)
                    dist_err_m = msg.get("distance_error_m", None)
                    ang_err_rad = msg.get("angle_error_rad", None)

                    ang_err_deg = None
                    if ang_err_rad is not None:
                        ang_err_deg = abs(rad2deg(float(ang_err_rad)))

                    ok = (state == "done")

                    extra = {
                        "ok": ok,
                        "state": state,
                        "duration_s": duration,
                        "resets": resets,
                        "collisions": collisions,
                        "distance_error_m": dist_err_m if dist_err_m is not None else None,
                        "angle_error_deg": ang_err_deg,
                        "task_id": task_id,
                    }
                    # métricas extras do controller (opcionais)
                    for k in ("min_front", "min_left", "min_right", "stopped_time_frac", "avoid_time_frac"):
                        if k in msg:
                            extra[k] = msg.get(k)
                    return extra

            # ignora status/event de outras tarefas

    # -------------------------
    # Suite
    # -------------------------
    def run_suite(
        self,
        repeats: int,
        validation_mode: bool = False,
        tail_shielding_mode: str = "keep",
    ) -> Dict[str, Any]:
        """
        Retorna JSON no formato:
        {
          "meta": {...},
          "tests": {
              "linear_1.0m": {...},
              "turn_90deg": {...},
              ...
          }
        }
        """
        repeats = int(repeats)
        validation_mode = bool(validation_mode)
        tail_shielding_mode = str(tail_shielding_mode).strip().lower()
        if tail_shielding_mode not in ("keep", "on", "off"):
            tail_shielding_mode = "keep"

        # Normaliza estado do controller para não herdar configuração antiga.
        self.send_control("validation_mode", validation_mode)
        if tail_shielding_mode in ("on", "off"):
            self.send_control("tail_shielding", tail_shielding_mode == "on")
        time.sleep(0.1)

        # Definição da suíte (você pode editar aqui sem mexer no compare)
        suite: List[Tuple[str, str, Any]] = []
        # lineares (frente e ré)
        suite.append(("linear_3m", "move_distance", 3.0))
        suite.append(("linear_back_3m", "move_distance", -3.0))
        suite.append(("linear_5m", "move_distance", 5.0))

        # giros
        for a in (90.0, -90.0, 180.0, -180.0):
            suite.append((f"turn_{int(a)}deg", "turn_angle", float(a)))

        # arcos suaves/fortes (duração em segundos)
        suite.append(("arc_soft_left", "arc", {"v": 0.25, "w": 0.6, "duration": 3.0}))
        suite.append(("arc_soft_right", "arc", {"v": 0.25, "w": -0.6, "duration": 3.0}))
        suite.append(("arc_hard_left", "arc", {"v": 0.18, "w": 1.2, "duration": 2.0}))
        suite.append(("arc_hard_right", "arc", {"v": 0.18, "w": -1.2, "duration": 2.0}))

        results: Dict[str, Any] = {"meta": {}, "tests": {}}
        results["meta"] = {
            "timestamp": datetime.now().isoformat(),
            "environment": self.environment,
            "repeats": repeats,
            "status_udp_port": STATUS_UDP_PORT,
            "topic_target": ROS_TOPIC_TARGET,
            "max_resets_per_attempt": self.max_resets_per_attempt,
            "timeouts": {
                "start_timeout_s": self.start_timeout_s,
                "exec_timeout_s": self.exec_timeout_s,
            },
            "validation_mode": validation_mode,
            "tail_shielding_mode": tail_shielding_mode,
            # 🔥 NOVO: snapshot dos genes ativos no controller
            "genes": {
                "base_speed": None,
                "turn_speed": None,
                "dist_stop": None,
                "dist_avoid": None,
                "avoid_gain": None,
                "turn_force": None,
            }
        }

        for test_name, cmd, value in suite:
            self.get_logger().info(f"[SUITE] Teste '{test_name}' ({cmd}={value}) x{repeats}")

            attempts = 0
            success = 0
            collisions_total = 0
            resets_total = 0

            dist_err_samples: List[float] = []
            ang_err_samples: List[float] = []
            duration_samples: List[float] = []
            dist_err_max = None
            ang_err_max = None

            attempt_details: List[Dict[str, Any]] = []

            for i in range(repeats):
                attempts += 1
                r = self.run_one_attempt(test_name, cmd, value)
                attempt_details.append(r)

                collisions_total += int(r.get("collisions", 0) or 0)
                resets_total += int(r.get("resets", 0) or 0)
                duration_samples.append(float(r.get("duration_s", 0.0) or 0.0))

                if r.get("ok", False):
                    success += 1
                    # Métricas só entram se ok=True e valor existir
                    if r.get("distance_error_m") is not None:
                        dist_err_samples.append(abs(float(r["distance_error_m"])))
                        val = abs(float(r["distance_error_m"]))
                        dist_err_max = val if dist_err_max is None else max(dist_err_max, val)
                    if r.get("angle_error_deg") is not None:
                        ang_err_samples.append(abs(float(r["angle_error_deg"])))
                        val_a = abs(float(r["angle_error_deg"]))
                        ang_err_max = val_a if ang_err_max is None else max(ang_err_max, val_a)
                else:
                    # Se a tentativa excedeu o limite de resets, falha o teste atual
                    # e avança para o próximo item da suíte (sem abortar a suíte inteira).
                    if str(r.get("state", "")) == "too_many_resets":
                        self.get_logger().warn(
                            f"[{test_name}] too_many_resets -> pulando restante deste teste e seguindo suíte"
                        )
                        break

                # pequeno respiro entre tentativas
                time.sleep(0.3)

            # Agregação (no formato do compare_controller_centric_v2)
            entry: Dict[str, Any] = {
                "attempts": attempts,
                "success": success,
                "collisions": collisions_total,
                "resets": resets_total,
                "duration_avg": statistics.mean(duration_samples) if duration_samples else 0.0,
                "duration_std": statistics.stdev(duration_samples) if len(duration_samples) > 1 else 0.0,
                "details": attempt_details,
            }

            # Para testes lineares, registrar distance_error_avg/std
            if cmd == "move_distance":
                entry["distance_error_avg"] = statistics.mean(dist_err_samples) if dist_err_samples else None
                entry["distance_error_std"] = statistics.stdev(dist_err_samples) if len(dist_err_samples) > 1 else 0.0
                entry["distance_error_max"] = dist_err_max

            # Para giros, registrar angle_error_avg (em graus)
            if cmd == "turn_angle":
                entry["angle_error_avg"] = statistics.mean(ang_err_samples) if ang_err_samples else None
                entry["angle_error_std"] = statistics.stdev(ang_err_samples) if len(ang_err_samples) > 1 else 0.0
                entry["angle_error_max"] = ang_err_max

            # Extras do controller (médias e std das frações e distâncias mínimas)
            min_front_vals = [a.get("min_front") for a in attempt_details if a.get("min_front") is not None]
            min_left_vals = [a.get("min_left") for a in attempt_details if a.get("min_left") is not None]
            min_right_vals = [a.get("min_right") for a in attempt_details if a.get("min_right") is not None]
            stopped_frac_vals = [a.get("stopped_time_frac") for a in attempt_details if a.get("stopped_time_frac") is not None]
            avoid_frac_vals = [a.get("avoid_time_frac") for a in attempt_details if a.get("avoid_time_frac") is not None]

            if min_front_vals:
                entry["min_front_avg"] = statistics.mean(min_front_vals)
                entry["min_front_std"] = statistics.stdev(min_front_vals) if len(min_front_vals) > 1 else 0.0
            if min_left_vals:
                entry["min_left_avg"] = statistics.mean(min_left_vals)
                entry["min_left_std"] = statistics.stdev(min_left_vals) if len(min_left_vals) > 1 else 0.0
            if min_right_vals:
                entry["min_right_avg"] = statistics.mean(min_right_vals)
                entry["min_right_std"] = statistics.stdev(min_right_vals) if len(min_right_vals) > 1 else 0.0
            if stopped_frac_vals:
                entry["stopped_time_frac_avg"] = statistics.mean(stopped_frac_vals)
                entry["stopped_time_frac_std"] = statistics.stdev(stopped_frac_vals) if len(stopped_frac_vals) > 1 else 0.0
            if avoid_frac_vals:
                entry["avoid_time_frac_avg"] = statistics.mean(avoid_frac_vals)
                entry["avoid_time_frac_std"] = statistics.stdev(avoid_frac_vals) if len(avoid_frac_vals) > 1 else 0.0

            results["tests"][test_name] = entry

        # garante robô parado e modo normal ao final
        self.send_stop()
        self.send_control("validation_mode", False)
        return results


# =========================
# MAIN
# =========================
def main():
    parser = argparse.ArgumentParser(description="P3AT Validator (Controller-Centric FINAL)")
    parser.add_argument("--label", type=str, default="before")
    parser.add_argument("--repeats", type=int, default=5)
    parser.add_argument("--env", type=str, default="webots_60x60_obstacles")
    parser.add_argument("--max-resets", type=int, default=3)
    parser.add_argument("--start-timeout", type=float, default=5.0)
    parser.add_argument("--exec-timeout", type=float, default=90.0)
    parser.add_argument(
        "--validation-mode",
        type=int,
        choices=[0, 1],
        default=0,
        help="Controller validation_mode: 1 disables in-task avoidance; 0 keeps normal avoidance.",
    )
    parser.add_argument(
        "--tail-shielding",
        choices=["keep", "on", "off"],
        default="keep",
        help="Set controller tail shielding mode for this run.",
    )
    parser.add_argument(
        "--genes-file",
        type=str,
        default="",
        help="Optional JSON file with genes to inject before running the suite.",
    )
    parser.add_argument(
        "--outdir",
        type=str,
        default=".",
        help="Directory where validation_<label>.json will be written.",
    )
    args = parser.parse_args()

    rclpy.init()
    node = P3ATValidator(
        environment=args.env,
        max_resets_per_attempt=args.max_resets,
        start_timeout_s=args.start_timeout,
        exec_timeout_s=args.exec_timeout,
    )

    spin_thread = threading.Thread(
        target=rclpy.spin,
        args=(node,),
        daemon=True
    )
    spin_thread.start()

    try:
        if args.genes_file:
            with open(args.genes_file, "r", encoding="utf-8") as f:
                genes = json.load(f)
            node.send_control("set_genes", genes)
            time.sleep(0.1)

        results = node.run_suite(
            repeats=args.repeats,
            validation_mode=bool(args.validation_mode),
            tail_shielding_mode=args.tail_shielding,
        )

        os.makedirs(args.outdir, exist_ok=True)
        out = os.path.join(args.outdir, f"validation_{args.label}.json")
        with open(out, "w") as f:
            json.dump(results, f, indent=2)

        print(f"\n✅ Validation salvo em: {out}\n")

    except KeyboardInterrupt:
        print("\n🛑 Validator interrompido pelo usuário")

    finally:
        global SHUTDOWN_REQUESTED
        SHUTDOWN_REQUESTED = True

        try:
            node.close()
        except Exception:
            pass

        try:
            node.destroy_node()
        except Exception:
            pass

        if rclpy.ok():
            rclpy.shutdown()

        spin_thread.join(timeout=1.0)

if __name__ == "__main__":
    main()

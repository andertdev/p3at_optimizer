#!/usr/bin/env python3
import argparse
import json
import math
import os
import signal
import statistics
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


SHUTDOWN_REQUESTED = False
ROS_TOPIC_TARGET = "/p3at_target"
EXECUTOR_NODE = "/executor"


def _signal_handler(sig, frame):
    global SHUTDOWN_REQUESTED
    SHUTDOWN_REQUESTED = True


signal.signal(signal.SIGINT, _signal_handler)
signal.signal(signal.SIGTERM, _signal_handler)


def now_s() -> float:
    return time.time()


def _mean(vals: List[float]) -> Optional[float]:
    return statistics.mean(vals) if vals else None


def _stdev(vals: List[float]) -> float:
    return statistics.stdev(vals) if len(vals) > 1 else 0.0


def _normalize_angle(rad: float) -> float:
    return math.atan2(math.sin(rad), math.cos(rad))


def _yaw_from_quat(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


class SuiteStopRequested(Exception):
    pass


class RealRobotValidator(Node):
    def __init__(
        self,
        environment: str,
        start_timeout_s: float,
        exec_timeout_s: float,
        collision_threshold_m: float,
    ):
        super().__init__("real_robot_validator")
        self.environment = environment
        self.start_timeout_s = float(start_timeout_s)
        self.exec_timeout_s = float(exec_timeout_s)
        self.collision_threshold_m = float(collision_threshold_m)

        self.cmd_pub = self.create_publisher(String, ROS_TOPIC_TARGET, 10)
        self.create_subscription(Odometry, "/odom", self._odom_cb, 20)
        self.create_subscription(LaserScan, "/scan", self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(Bool, "/executor_active", self._executor_cb, 10)

        self.executor_set_params = self.create_client(
            SetParameters, f"{EXECUTOR_NODE}/set_parameters"
        )
        self.executor_get_params = self.create_client(
            GetParameters, f"{EXECUTOR_NODE}/get_parameters"
        )

        self.latest_pose: Optional[Pose2D] = None
        self.latest_linear_speed = 0.0
        self.latest_angular_speed = 0.0
        self.latest_scan: Optional[Dict[str, float]] = None
        self.executor_active = False
        self.executor_seen = False
        self.last_odom_time = 0.0
        self.last_scan_time = 0.0

        self.get_logger().info(
            f"[REAL_VALIDATOR] topic={ROS_TOPIC_TARGET} executor={EXECUTOR_NODE} env={environment}"
        )

    def _odom_cb(self, msg: Odometry):
        self.latest_pose = Pose2D(
            x=float(msg.pose.pose.position.x),
            y=float(msg.pose.pose.position.y),
            yaw=_yaw_from_quat(msg.pose.pose.orientation),
        )
        self.latest_linear_speed = float(msg.twist.twist.linear.x)
        self.latest_angular_speed = float(msg.twist.twist.angular.z)
        self.last_odom_time = now_s()

    def _scan_cb(self, msg: LaserScan):
        ranges = list(msg.ranges)
        if not ranges:
            return
        mid = len(ranges) // 2

        def _clean(vals):
            out = []
            for v in vals:
                if not math.isfinite(v) or v <= 0.0:
                    continue
                out.append(float(v))
            return out

        front = _clean(ranges[max(0, mid - 2): min(len(ranges), mid + 3)])
        left = _clean(ranges[min(len(ranges), mid + 3): min(len(ranges), mid + 8)])
        right = _clean(ranges[max(0, mid - 8): max(0, mid - 3)])
        if not front:
            return
        self.latest_scan = {
            "min_front": min(front),
            "min_left": min(left) if left else min(front),
            "min_right": min(right) if right else min(front),
        }
        self.last_scan_time = now_s()

    def _executor_cb(self, msg: Bool):
        self.executor_active = bool(msg.data)
        self.executor_seen = True

    def wait_for_inputs(self, timeout_s: float = 5.0) -> None:
        deadline = now_s() + timeout_s
        while now_s() < deadline:
            if self.latest_pose is not None and self.latest_scan is not None and self.executor_seen:
                return
            time.sleep(0.05)
        missing = []
        if self.latest_pose is None:
            missing.append("/odom")
        if self.latest_scan is None:
            missing.append("/scan")
        if not self.executor_seen:
            missing.append("/executor_active")
        raise RuntimeError("Missing required topics: " + ", ".join(missing))

    def wait_for_executor_service(self, timeout_s: float = 5.0) -> None:
        ok_set = self.executor_set_params.wait_for_service(timeout_sec=timeout_s)
        ok_get = self.executor_get_params.wait_for_service(timeout_sec=timeout_s)
        if ok_set and ok_get:
            return
        raise RuntimeError(f"Executor parameter service unavailable: {EXECUTOR_NODE}")

    def send_control(self, command: str, value: Any) -> None:
        msg = String()
        msg.data = json.dumps({"command": command, "value": value})
        self.cmd_pub.publish(msg)

    def send_cmd(self, command: str, value: Any) -> None:
        msg = String()
        msg.data = json.dumps({"command": command, "value": value})
        self.cmd_pub.publish(msg)

    def send_stop(self) -> None:
        self.send_control("stop", 0)

    def apply_genes(self, genes: Dict[str, float]) -> None:
        params = {
            "explore_speed": float(genes["base_speed"]),
            "turn_speed": float(genes["turn_speed"]),
            "avoid_gain": float(genes["avoid_gain"]),
            "dist_stop": float(genes["dist_stop"]),
            "dist_avoid": float(genes["dist_avoid"]),
            "turn_force": float(genes["turn_force"]),
        }
        req = []
        for name, value in params.items():
            req.append(
                ParameterMsg(
                    name=name,
                    value=ParameterValue(
                        type=ParameterType.PARAMETER_DOUBLE,
                        double_value=float(value),
                    ),
                )
            )
        request = SetParameters.Request()
        request.parameters = req
        future = self.executor_set_params.call_async(request)
        while rclpy.ok() and not future.done():
            time.sleep(0.05)
        response = future.result()
        if response is None:
            raise RuntimeError("Executor parameter update returned no result")
        failed = [r.reason for r in response.results if not r.successful]
        if failed:
            raise RuntimeError("Executor parameter update failed: " + "; ".join(failed))

    def read_active_genes(self, source_genes: Dict[str, float]) -> Dict[str, float]:
        return {
            "base_speed": float(source_genes["base_speed"]),
            "turn_speed": float(source_genes["turn_speed"]),
            "dist_stop": float(source_genes["dist_stop"]),
            "dist_avoid": float(source_genes["dist_avoid"]),
            "avoid_gain": float(source_genes["avoid_gain"]),
            "turn_force": float(source_genes["turn_force"]),
        }

    def read_current_genes(self) -> Dict[str, float]:
        request = GetParameters.Request()
        names = [
            "explore_speed",
            "turn_speed",
            "avoid_gain",
            "dist_stop",
            "dist_avoid",
            "turn_force",
        ]
        request.names = names
        future = self.executor_get_params.call_async(request)
        while rclpy.ok() and not future.done():
            time.sleep(0.05)
        response = future.result()
        if response is None or len(response.values) != len(names):
            raise RuntimeError("Executor get_parameters returned incomplete data")
        values = {}
        for name, val in zip(names, response.values):
            values[name] = float(val.double_value)
        return {
            "base_speed": values["explore_speed"],
            "turn_speed": values["turn_speed"],
            "avoid_gain": values["avoid_gain"],
            "dist_stop": values["dist_stop"],
            "dist_avoid": values["dist_avoid"],
            "turn_force": values["turn_force"],
        }

    def _estimate_avoid_active(self, scan_min: float, dist_avoid: float) -> bool:
        return scan_min <= max(dist_avoid, self.collision_threshold_m)

    def _planned_duration(self, command: str, value: Any, genes: Dict[str, float]) -> float:
        if command == "move_distance":
            base_speed = max(abs(float(genes["base_speed"])), 1e-6)
            return abs(float(value)) / base_speed
        if command == "turn_angle":
            turn_speed = max(abs(float(genes["turn_speed"])), 1e-6)
            return abs(math.radians(float(value))) / turn_speed
        if command == "arc":
            return float(value.get("duration", 0.0))
        return 0.0

    def _prompt_attempt_start(self, test_name: str, rep_idx: int, repeats: int) -> None:
        prompt = (
            f"\n[MANUAL RESET] Posicione o robo para {test_name} "
            f"({rep_idx}/{repeats}). Enter=iniciar | stop=encerrar suite: "
        )
        while True:
            cmd = input(prompt).strip().lower()
            if cmd == "":
                return
            if cmd == "stop":
                raise SuiteStopRequested()
            print("Comando invalido. Use Enter para iniciar ou 'stop' para encerrar.")

    def _prompt_retry_same_attempt(self, test_name: str, rep_idx: int, state: str) -> bool:
        prompt = (
            f"\n[ATTEMPT STOPPED] {test_name} ({rep_idx}) terminou como '{state}'. "
            "Reposicione o robo. Enter=repetir mesma tentativa | stop=encerrar suite: "
        )
        while True:
            cmd = input(prompt).strip().lower()
            if cmd == "":
                return True
            if cmd == "stop":
                return False
            print("Comando invalido. Use Enter para repetir ou 'stop' para encerrar.")

    def run_one_attempt(self, test_name: str, command: str, value: Any, genes: Dict[str, float]) -> Dict[str, Any]:
        self.send_stop()
        time.sleep(0.2)
        start_pose = self.latest_pose
        if start_pose is None:
            raise RuntimeError("No odom available before attempt")

        t0 = now_s()
        planned_duration = self._planned_duration(command, value, genes)
        started = False
        ended = False
        active_prev = self.executor_active
        min_front = float("inf")
        min_left = float("inf")
        min_right = float("inf")
        avoid_time = 0.0
        stopped_time = 0.0
        collision_events = 0
        in_collision = False
        last_sample_t = t0

        self.send_cmd(command, value)

        while True:
            if SHUTDOWN_REQUESTED:
                self.send_stop()
                return {
                    "ok": False,
                    "state": "shutdown",
                    "duration_s": now_s() - t0,
                    "resets": 0,
                    "collisions": collision_events,
                    "distance_error_m": None,
                    "angle_error_deg": None,
                }

            t = now_s()
            if not started and (t - t0) > self.start_timeout_s:
                self.send_stop()
                return {
                    "ok": False,
                    "state": "no_start",
                    "duration_s": t - t0,
                    "resets": 0,
                    "collisions": collision_events,
                    "distance_error_m": None,
                    "angle_error_deg": None,
                }

            if started and (t - t0) > self.exec_timeout_s:
                self.send_stop()
                return {
                    "ok": False,
                    "state": "timeout",
                    "duration_s": t - t0,
                    "resets": 0,
                    "collisions": collision_events,
                    "distance_error_m": None,
                    "angle_error_deg": None,
                    "min_front": min_front if math.isfinite(min_front) else None,
                    "min_left": min_left if math.isfinite(min_left) else None,
                    "min_right": min_right if math.isfinite(min_right) else None,
                }

            active_now = self.executor_active
            if active_now and not started:
                started = True

            if started and (not active_now) and active_prev:
                ended = True

            if started and self.latest_scan is not None and self.latest_pose is not None:
                dt = max(0.0, t - last_sample_t)
                last_sample_t = t
                min_front = min(min_front, self.latest_scan["min_front"])
                min_left = min(min_left, self.latest_scan["min_left"])
                min_right = min(min_right, self.latest_scan["min_right"])
                scan_min = min(
                    self.latest_scan["min_front"],
                    self.latest_scan["min_left"],
                    self.latest_scan["min_right"],
                )
                if self._estimate_avoid_active(scan_min, float(genes["dist_avoid"])):
                    avoid_time += dt
                if abs(self.latest_linear_speed) < 0.01 and abs(self.latest_angular_speed) < 0.05:
                    stopped_time += dt
                if scan_min <= self.collision_threshold_m:
                    if not in_collision:
                        collision_events += 1
                        in_collision = True
                else:
                    in_collision = False

            active_prev = active_now
            if ended:
                break
            time.sleep(0.05)

        end_pose = self.latest_pose
        if end_pose is None:
            raise RuntimeError("No odom available after attempt")

        duration = now_s() - t0
        total = max(duration, 1e-6)
        distance_error = None
        angle_error_deg = None

        if command == "move_distance":
            dx = end_pose.x - start_pose.x
            dy = end_pose.y - start_pose.y
            progress = dx * math.cos(start_pose.yaw) + dy * math.sin(start_pose.yaw)
            distance_error = progress - float(value)
        elif command == "turn_angle":
            yaw_delta = _normalize_angle(end_pose.yaw - start_pose.yaw)
            target_rad = math.radians(float(value))
            angle_error_deg = abs(math.degrees(_normalize_angle(yaw_delta - target_rad)))

        state = "done"
        if planned_duration > 0.0 and duration < (0.8 * planned_duration):
            state = "operator_abort"

        return {
            "ok": state == "done",
            "state": state,
            "duration_s": duration,
            "resets": 0,
            "collisions": collision_events,
            "distance_error_m": distance_error,
            "angle_error_deg": angle_error_deg,
            "min_front": min_front if math.isfinite(min_front) else None,
            "min_left": min_left if math.isfinite(min_left) else None,
            "min_right": min_right if math.isfinite(min_right) else None,
            "stopped_time_frac": stopped_time / total,
            "avoid_time_frac": avoid_time / total,
        }

    def run_suite(
        self,
        repeats: int,
        genes: Dict[str, float],
        suite: List[Tuple[str, str, Any]],
        label: str,
    ) -> Dict[str, Any]:
        results: Dict[str, Any] = {
            "meta": {
                "timestamp": datetime.now().isoformat(),
                "environment": self.environment,
                "label": label,
                "repeats": int(repeats),
                "topic_target": ROS_TOPIC_TARGET,
                "executor_node": EXECUTOR_NODE,
                "timeouts": {
                    "start_timeout_s": self.start_timeout_s,
                    "exec_timeout_s": self.exec_timeout_s,
                },
                "measurement_mode": {
                    "distance_error": "odom_projection",
                    "angle_error": "odom_yaw_delta",
                    "collision": f"scan_threshold<{self.collision_threshold_m:.3f}m",
                    "avoid_time_frac": "scan_proxy_vs_dist_avoid",
                },
                "genes": self.read_active_genes(genes),
                "suite_state": "running",
            },
            "tests": {},
        }

        try:
            for test_name, cmd, value in suite:
                attempts = 0
                success = 0
                collisions_total = 0
                resets_total = 0
                operator_aborts = 0
                duration_samples: List[float] = []
                dist_samples: List[float] = []
                ang_samples: List[float] = []
                attempt_details: List[Dict[str, Any]] = []

                self.get_logger().info(f"[REAL_SUITE] {test_name} ({cmd}={value}) x{repeats}")

                rep_idx = 1
                while rep_idx <= int(repeats):
                    self._prompt_attempt_start(test_name, rep_idx, int(repeats))
                    r = self.run_one_attempt(test_name, cmd, value, genes)

                    if r.get("state") == "operator_abort":
                        operator_aborts += 1
                        if not self._prompt_retry_same_attempt(test_name, rep_idx, str(r.get("state"))):
                            raise SuiteStopRequested()
                        continue

                    attempts += 1
                    attempt_details.append(r)
                    collisions_total += int(r.get("collisions", 0) or 0)
                    resets_total += int(r.get("resets", 0) or 0)
                    duration_samples.append(float(r.get("duration_s", 0.0) or 0.0))
                    if r.get("ok"):
                        success += 1
                        if r.get("distance_error_m") is not None:
                            dist_samples.append(abs(float(r["distance_error_m"])))
                        if r.get("angle_error_deg") is not None:
                            ang_samples.append(abs(float(r["angle_error_deg"])))
                    rep_idx += 1
                    time.sleep(0.3)

                entry: Dict[str, Any] = {
                    "attempts": attempts,
                    "success": success,
                    "collisions": collisions_total,
                    "resets": resets_total,
                    "operator_aborts": operator_aborts,
                    "duration_avg": _mean(duration_samples) or 0.0,
                    "duration_std": _stdev(duration_samples),
                    "details": attempt_details,
                }

                if cmd == "move_distance":
                    entry["distance_error_avg"] = _mean(dist_samples)
                    entry["distance_error_std"] = _stdev(dist_samples)
                    entry["distance_error_max"] = max(dist_samples) if dist_samples else None
                if cmd == "turn_angle":
                    entry["angle_error_avg"] = _mean(ang_samples)
                    entry["angle_error_std"] = _stdev(ang_samples)
                    entry["angle_error_max"] = max(ang_samples) if ang_samples else None

                for key in ("min_front", "min_left", "min_right", "stopped_time_frac", "avoid_time_frac"):
                    vals = [float(a[key]) for a in attempt_details if a.get(key) is not None]
                    if vals:
                        entry[f"{key}_avg"] = _mean(vals)
                        entry[f"{key}_std"] = _stdev(vals)

                results["tests"][test_name] = entry
        except SuiteStopRequested:
            results["meta"]["suite_state"] = "operator_stopped"
            self.send_stop()
            return results

        self.send_stop()
        results["meta"]["suite_state"] = "completed"
        return results


def _default_suite() -> List[Tuple[str, str, Any]]:
    return [
        ("linear_3m", "move_distance", 3.0),
        ("linear_back_3m", "move_distance", -3.0),
        ("linear_5m", "move_distance", 5.0),
        ("turn_90deg", "turn_angle", 90.0),
        ("turn_-90deg", "turn_angle", -90.0),
        ("turn_180deg", "turn_angle", 180.0),
        ("turn_-180deg", "turn_angle", -180.0),
        ("arc_soft_left", "arc", {"v": 0.25, "w": 0.6, "duration": 3.0}),
        ("arc_soft_right", "arc", {"v": 0.25, "w": -0.6, "duration": 3.0}),
        ("arc_hard_left", "arc", {"v": 0.18, "w": 1.2, "duration": 2.0}),
        ("arc_hard_right", "arc", {"v": 0.18, "w": -1.2, "duration": 2.0}),
    ]


def main():
    parser = argparse.ArgumentParser(description="Real-robot validator for P3AT optimized genes")
    parser.add_argument("--label", default="real_validation")
    parser.add_argument("--genes-file", default="")
    parser.add_argument("--repeats", type=int, default=5)
    parser.add_argument("--env", default="p3at_real_lab")
    parser.add_argument("--start-timeout", type=float, default=4.0)
    parser.add_argument("--exec-timeout", type=float, default=90.0)
    parser.add_argument("--collision-threshold", type=float, default=0.20)
    parser.add_argument("--outdir", default=".")
    args = parser.parse_args()

    rclpy.init()
    node = RealRobotValidator(
        environment=args.env,
        start_timeout_s=args.start_timeout,
        exec_timeout_s=args.exec_timeout,
        collision_threshold_m=args.collision_threshold,
    )

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.wait_for_executor_service()
        node.wait_for_inputs()
        if args.genes_file:
            with open(args.genes_file, "r", encoding="utf-8") as f:
                genes = json.load(f)
            node.apply_genes(genes)
            time.sleep(0.2)
            genes = node.read_current_genes()
        else:
            genes = node.read_current_genes()
        results = node.run_suite(
            repeats=args.repeats,
            genes=genes,
            suite=_default_suite(),
            label=args.label,
        )
        os.makedirs(args.outdir, exist_ok=True)
        out = os.path.join(args.outdir, f"validation_{args.label}.json")
        with open(out, "w", encoding="utf-8") as f:
            json.dump(results, f, indent=2)
        print(f"[OK] Real validation written to: {out}")
    finally:
        global SHUTDOWN_REQUESTED
        SHUTDOWN_REQUESTED = True
        try:
            node.send_stop()
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

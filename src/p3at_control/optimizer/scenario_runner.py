#!/usr/bin/env python3
"""
Scenario runner for 4-paper scenarios:

Block A (parsing):
  A1 llm parser (n8n/LLM)
  A2 baseline parser (CaP)

Block B (execution/generalization):
  B1 llm parser + {default, pygad, moead}
  B2 cap parser + {default, pygad, moead}

Outputs are written as JSON files under --outdir.
"""

import argparse
import json
import os
import socket
import statistics
import time
import uuid
from datetime import datetime
from typing import Any, Dict, List, Optional, Tuple
from urllib.error import URLError, HTTPError
from urllib.request import Request, urlopen

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    from p3at_control.cap_parser import parse_cap
except Exception:
    import sys

    here = os.path.dirname(os.path.abspath(__file__))
    pkg_dir = os.path.normpath(os.path.join(here, "..", "p3at_control"))
    if pkg_dir not in sys.path:
        sys.path.append(pkg_dir)
    from cap_parser import parse_cap


STATUS_UDP_BIND_IP = "0.0.0.0"
STATUS_UDP_PORT = 20002
STATUS_SOCKET_TIMEOUT_S = 0.25
ROS_TOPIC_TARGET = "/p3at_target"

KNOWN_COMMANDS = {
    "move_distance",
    "turn_angle",
    "arc",
    "stop",
    "explore_mode",
    "validation_mode",
    "set_genes",
}


def now_s() -> float:
    return time.time()


def _safe_float(v: Any, default: float = 0.0) -> float:
    try:
        return float(v)
    except Exception:
        return default


def _rad2deg(r: float) -> float:
    return r * 180.0 / 3.141592653589793


def _task_suite() -> List[Tuple[str, str, Any, str]]:
    # (test_id, command, value, natural-language prompt)
    return [
        ("linear_3m", "move_distance", 3.0, "andar 3 metros"),
        ("linear_back_3m", "move_distance", -3.0, "voltar 3 metros"),
        ("linear_5m", "move_distance", 5.0, "andar 5 metros"),
        ("turn_90deg", "turn_angle", 90.0, "girar 90 graus esquerda"),
        ("turn_-90deg", "turn_angle", -90.0, "girar 90 graus direita"),
        ("turn_180deg", "turn_angle", 180.0, "girar 180 graus esquerda"),
        ("turn_-180deg", "turn_angle", -180.0, "girar 180 graus direita"),
        ("arc_soft_left", "arc", {"v": 0.25, "w": 0.6, "duration": 3.0}, "arco v=0.25 w=0.6 d=3"),
        ("arc_soft_right", "arc", {"v": 0.25, "w": -0.6, "duration": 3.0}, "arco v=0.25 w=-0.6 d=3"),
        ("arc_hard_left", "arc", {"v": 0.18, "w": 1.2, "duration": 2.0}, "arco v=0.18 w=1.2 d=2"),
        ("arc_hard_right", "arc", {"v": 0.18, "w": -1.2, "duration": 2.0}, "arco v=0.18 w=-1.2 d=2"),
    ]


def _load_json(path: Optional[str]) -> Dict[str, Any]:
    if not path:
        return {}
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def _request_json(url: str, payload: Dict[str, Any], timeout_s: float) -> Dict[str, Any]:
    data = json.dumps(payload).encode("utf-8")
    req = Request(url, data=data, headers={"Content-Type": "application/json"}, method="POST")
    with urlopen(req, timeout=timeout_s) as resp:
        raw = resp.read().decode("utf-8")
    return json.loads(raw)


def _extract_command_candidate(resp_obj: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    if not isinstance(resp_obj, dict):
        return None
    # API shape: {"status":"received","data":{"parsed":{...}, ...}}
    data = resp_obj.get("data")
    if isinstance(data, dict):
        if isinstance(data.get("parsed"), dict):
            return data["parsed"]
        if "command" in data or "linear" in data or "angular" in data:
            return data
    if "command" in resp_obj or "linear" in resp_obj or "angular" in resp_obj:
        return resp_obj
    for k in ("command_json", "json", "llm_json"):
        v = resp_obj.get(k)
        if isinstance(v, dict):
            return v
    return None


def _normalize_structured(candidate: Dict[str, Any]) -> Dict[str, Any]:
    cmd, _ = parse_cap(candidate)
    return cmd


def _command_valid(cmd: Dict[str, Any]) -> Tuple[bool, List[str]]:
    issues: List[str] = []
    if not isinstance(cmd, dict):
        return False, ["not_dict"]
    if "command" not in cmd and ("linear" in cmd or "angular" in cmd):
        return True, issues
    c = cmd.get("command")
    if c not in KNOWN_COMMANDS:
        issues.append("unknown_command")
        return False, issues
    if c in ("move_distance", "turn_angle", "explore_mode", "validation_mode", "arc", "set_genes") and "value" not in cmd:
        issues.append("missing_value")
    if c == "arc":
        val = cmd.get("value")
        if not isinstance(val, dict) or any(k not in val for k in ("v", "w", "duration")):
            issues.append("arc_missing_fields")
    if c == "set_genes":
        if not isinstance(cmd.get("value"), dict):
            issues.append("genes_not_dict")
    return len(issues) == 0, issues


class ScenarioRunner(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("scenario_runner")
        self.args = args
        self.pub = self.create_publisher(String, ROS_TOPIC_TARGET, 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((STATUS_UDP_BIND_IP, STATUS_UDP_PORT))
        self.sock.settimeout(STATUS_SOCKET_TIMEOUT_S)
        self.methods_order = [m.strip() for m in str(args.method_order).split(",") if m.strip()]
        alias = {"current": "llm"}
        self.methods_order = [alias.get(m, m) for m in self.methods_order]
        valid_methods = {"cap", "llm"}
        if not self.methods_order or any(m not in valid_methods for m in self.methods_order):
            raise ValueError(
                f"Invalid --method-order={args.method_order}. Use a comma list with 'cap' and/or 'llm'."
            )
        self.profiles = [p.strip() for p in str(args.profiles).split(",") if p.strip()]
        valid_profiles = {"default", "pygad", "moead"}
        if not self.profiles or any(p not in valid_profiles for p in self.profiles):
            raise ValueError(
                f"Invalid --profiles={args.profiles}. Use comma list among default,pygad,moead."
            )

        self.current_map = _load_json(args.current_map)
        self.genes_pygad = _load_json(args.pygad_genes) if "pygad" in self.profiles else {}
        self.genes_moead = _load_json(args.moead_genes) if "moead" in self.profiles else {}

        if "pygad" in self.profiles and not self.genes_pygad:
            raise ValueError(
                f"--profiles includes pygad, but genes file is missing/empty: {args.pygad_genes}"
            )
        if "moead" in self.profiles and not self.genes_moead:
            raise ValueError(
                f"--profiles includes moead, but genes file is missing/empty: {args.moead_genes}"
            )

    def close(self):
        try:
            self.sock.close()
        except Exception:
            pass

    def send_cmd(self, command: str, value: Any, task_id: Optional[str] = None):
        payload: Dict[str, Any] = {"command": command, "value": value}
        if task_id:
            payload["task_id"] = task_id
        msg = String()
        msg.data = json.dumps(payload)
        self.pub.publish(msg)

    def parse_with_cap(self, text: str) -> Tuple[Optional[Dict[str, Any]], Dict[str, Any], float]:
        t0 = now_s()
        cmd, meta = parse_cap({"text": text})
        latency_ms = (now_s() - t0) * 1000.0
        return cmd, {"source": "cap", "meta": meta}, latency_ms

    def parse_with_llm(self, text: str) -> Tuple[Optional[Dict[str, Any]], Dict[str, Any], float]:
        t0 = now_s()
        if text in self.current_map and isinstance(self.current_map[text], dict):
            cmd = self.current_map[text]
            latency_ms = (now_s() - t0) * 1000.0
            return cmd, {"source": "llm_map"}, latency_ms

        if self.args.current_webhook:
            try:
                resp = _request_json(
                    self.args.current_webhook,
                    {"text": text},
                    timeout_s=self.args.current_timeout,
                )
                cand = _extract_command_candidate(resp)
                latency_ms = (now_s() - t0) * 1000.0
                if isinstance(cand, dict):
                    return cand, {"source": "llm_webhook", "raw_response": resp}, latency_ms
                return None, {"source": "llm_webhook", "error": "no_command_in_response", "raw_response": resp}, latency_ms
            except (URLError, HTTPError, TimeoutError) as e:
                latency_ms = (now_s() - t0) * 1000.0
                return None, {"source": "llm_webhook", "error": str(e)}, latency_ms
            except Exception as e:
                latency_ms = (now_s() - t0) * 1000.0
                return None, {"source": "llm_webhook", "error": str(e)}, latency_ms

        latency_ms = (now_s() - t0) * 1000.0
        return None, {"source": "llm", "error": "no_llm_parser_source"}, latency_ms

    def parse_method(self, method: str, text: str) -> Tuple[Optional[Dict[str, Any]], Dict[str, Any], float]:
        if method == "cap":
            return self.parse_with_cap(text)
        if method == "llm":
            return self.parse_with_llm(text)
        return None, {"error": f"unknown parser method={method}"}, 0.0

    def run_block_a(self) -> Dict[str, Any]:
        records = []
        prompts = [p[3] for p in _task_suite()]
        for text in prompts:
            row: Dict[str, Any] = {"text": text}
            for method in self.methods_order:
                raw_cmd, meta, latency_ms = self.parse_method(method, text)
                normalized = _normalize_structured(raw_cmd) if isinstance(raw_cmd, dict) else None
                valid, issues = _command_valid(normalized if normalized else {})
                row[method] = {
                    "latency_ms": latency_ms,
                    "raw_cmd": raw_cmd,
                    "normalized_cmd": normalized,
                    "valid": valid,
                    "issues": issues,
                    "parser_meta": meta,
                    "clamp_or_fix_applied": (raw_cmd != normalized) if isinstance(raw_cmd, dict) else False,
                }
            cur = row.get("llm", {}).get("normalized_cmd")
            cap = row.get("cap", {}).get("normalized_cmd")
            row["consistency_same_output"] = (cur == cap) if (cur is not None and cap is not None) else None
            records.append(row)

        summary: Dict[str, Any] = {
            "methods_order": self.methods_order,
        }
        if "llm" in self.methods_order:
            summary["llm_valid_rate"] = _ratio(sum(1 for r in records if r.get("llm", {}).get("valid")), len(records))
            summary["llm_avg_latency_ms"] = _mean([r.get("llm", {}).get("latency_ms", 0.0) for r in records])
        if "cap" in self.methods_order:
            summary["cap_valid_rate"] = _ratio(sum(1 for r in records if r.get("cap", {}).get("valid")), len(records))
            summary["cap_avg_latency_ms"] = _mean([r.get("cap", {}).get("latency_ms", 0.0) for r in records])
        if "llm" in self.methods_order and "cap" in self.methods_order:
            cons_vals = [r["consistency_same_output"] for r in records if r["consistency_same_output"] is not None]
            summary["consistency_rate"] = _ratio(sum(1 for v in cons_vals if v), len(cons_vals))
        return {
            "meta": {
                "timestamp": datetime.now().isoformat(),
                "block": "A",
                "description": "Parsing comparison (llm vs cap)",
            },
            "summary": summary,
            "records": records,
        }

    def _recv_one(self) -> Optional[Dict[str, Any]]:
        try:
            data, _ = self.sock.recvfrom(65535)
            txt = data.decode("utf-8", errors="ignore").strip()
            if not txt:
                return None
            return json.loads(txt)
        except Exception:
            return None

    def _run_one_attempt(self, method: str, prompt: str) -> Dict[str, Any]:
        task_id = uuid.uuid4().hex
        parse_cmd, parse_meta, parse_latency_ms = self.parse_method(method, prompt)
        if not parse_cmd:
            return {
                "ok": False,
                "state": "parse_error",
                "task_id": task_id,
                "duration_s": 0.0,
                "resets": 0,
                "collisions": 0,
                "distance_error_m": None,
                "angle_error_deg": None,
                "parse_meta": parse_meta,
                "parse_latency_ms": parse_latency_ms,
            }

        cmd = _normalize_structured(parse_cmd)
        valid, issues = _command_valid(cmd)
        if not valid:
            return {
                "ok": False,
                "state": "invalid_command",
                "task_id": task_id,
                "duration_s": 0.0,
                "resets": 0,
                "collisions": 0,
                "distance_error_m": None,
                "angle_error_deg": None,
                "parse_meta": parse_meta,
                "parse_latency_ms": parse_latency_ms,
                "issues": issues,
                "command": cmd,
            }

        self.send_cmd(cmd["command"], cmd.get("value"), task_id=task_id)
        t0 = now_s()
        resets = 0
        collisions = 0
        deadline = t0 + self.args.exec_timeout

        while now_s() < deadline:
            m = self._recv_one()
            if not m:
                continue
            if m.get("task_id") != task_id:
                continue
            if m.get("type") == "event" and m.get("event") == "reset":
                resets += 1
                if resets <= self.args.max_resets:
                    self.send_cmd(cmd["command"], cmd.get("value"), task_id=task_id)
                    continue
                return {
                    "ok": False,
                    "state": "too_many_resets",
                    "task_id": task_id,
                    "duration_s": now_s() - t0,
                    "resets": resets,
                    "collisions": collisions,
                    "distance_error_m": None,
                    "angle_error_deg": None,
                    "parse_meta": parse_meta,
                    "parse_latency_ms": parse_latency_ms,
                }
            if m.get("type") == "status":
                if bool(m.get("collision", False)):
                    collisions += 1
                state = str(m.get("state", "")).lower()
                if state in ("done", "crashed", "aborted", "stopped"):
                    ang_deg = None
                    if m.get("angle_error_rad") is not None:
                        ang_deg = abs(_rad2deg(float(m["angle_error_rad"])))
                    return {
                        "ok": state == "done",
                        "state": state,
                        "task_id": task_id,
                        "duration_s": now_s() - t0,
                        "resets": resets,
                        "collisions": collisions,
                        "distance_error_m": m.get("distance_error_m"),
                        "angle_error_deg": ang_deg,
                        "min_front": m.get("min_front"),
                        "min_left": m.get("min_left"),
                        "min_right": m.get("min_right"),
                        "stopped_time_frac": m.get("stopped_time_frac"),
                        "avoid_time_frac": m.get("avoid_time_frac"),
                        "parse_meta": parse_meta,
                        "parse_latency_ms": parse_latency_ms,
                    }

        return {
            "ok": False,
            "state": "timeout",
            "task_id": task_id,
            "duration_s": now_s() - t0,
            "resets": resets,
            "collisions": collisions,
            "distance_error_m": None,
            "angle_error_deg": None,
            "parse_meta": parse_meta,
            "parse_latency_ms": parse_latency_ms,
        }

    def _apply_gene_profile(self, profile: str):
        if profile == "default":
            # no set_genes; keep controller defaults
            return
        genes = self.genes_pygad if profile == "pygad" else self.genes_moead
        if genes:
            self.send_cmd("set_genes", genes)
            time.sleep(0.15)

    def _run_execution_suite(self, method: str, gene_profile: str) -> Dict[str, Any]:
        self.send_cmd("validation_mode", True)
        time.sleep(0.10)
        self._apply_gene_profile(gene_profile)
        time.sleep(0.10)

        out: Dict[str, Any] = {
            "meta": {
                "timestamp": datetime.now().isoformat(),
                "block": "B",
                "parser_method": method,
                "gene_profile": gene_profile,
                "repeats": self.args.repeats,
                "exec_timeout": self.args.exec_timeout,
            },
            "tests": {},
        }

        for test_name, _cmd, _value, prompt in _task_suite():
            attempts = 0
            success = 0
            collisions = 0
            resets = 0
            durations: List[float] = []
            dist_samples: List[float] = []
            ang_samples: List[float] = []
            details: List[Dict[str, Any]] = []

            for _ in range(self.args.repeats):
                attempts += 1
                r = self._run_one_attempt(method, prompt)
                details.append(r)
                success += 1 if r.get("ok") else 0
                collisions += int(r.get("collisions", 0) or 0)
                resets += int(r.get("resets", 0) or 0)
                durations.append(float(r.get("duration_s", 0.0) or 0.0))
                if r.get("ok") and r.get("distance_error_m") is not None:
                    dist_samples.append(abs(float(r["distance_error_m"])))
                if r.get("ok") and r.get("angle_error_deg") is not None:
                    ang_samples.append(abs(float(r["angle_error_deg"])))
                time.sleep(0.25)

            entry = {
                "attempts": attempts,
                "success": success,
                "collisions": collisions,
                "resets": resets,
                "duration_avg": _mean(durations),
                "duration_std": _stdev(durations),
                "details": details,
            }
            if dist_samples:
                entry["distance_error_avg"] = _mean(dist_samples)
                entry["distance_error_std"] = _stdev(dist_samples)
                entry["distance_error_max"] = max(dist_samples)
            if ang_samples:
                entry["angle_error_avg"] = _mean(ang_samples)
                entry["angle_error_std"] = _stdev(ang_samples)
                entry["angle_error_max"] = max(ang_samples)
            out["tests"][test_name] = entry

        self.send_cmd("stop", 0)
        return out

    def run_all(self) -> Dict[str, str]:
        os.makedirs(self.args.outdir, exist_ok=True)
        saved: Dict[str, str] = {}
        block_b_docs: Dict[str, Dict[str, Any]] = {}

        # Block A
        if self.args.run_block_a:
            a = self.run_block_a()
            a_path = os.path.join(self.args.outdir, "cap_vs_llm_parsing.json")
            _save_json(a_path, a)
            saved["cap_vs_llm_parsing"] = a_path

        # Block B
        if self.args.run_block_b:
            for method in self.methods_order:
                for profile in self.profiles:
                    doc = self._run_execution_suite(method, profile)
                    name = f"{method}_{profile}.json"
                    path = os.path.join(self.args.outdir, name)
                    _save_json(path, doc)
                    key = f"{method}_{profile}"
                    saved[key] = path
                    block_b_docs[key] = doc
            b_summary = _build_block_b_summary(block_b_docs)
            b_summary_path = os.path.join(self.args.outdir, "cap_vs_llm_summary.json")
            _save_json(b_summary_path, b_summary)
            saved["cap_vs_llm_summary"] = b_summary_path

        return saved


def _save_json(path: str, obj: Dict[str, Any]):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(obj, f, indent=2, ensure_ascii=False)


def _mean(vals: List[float]) -> float:
    return statistics.mean(vals) if vals else 0.0


def _stdev(vals: List[float]) -> float:
    return statistics.stdev(vals) if len(vals) > 1 else 0.0


def _ratio(a: int, b: int) -> float:
    if b <= 0:
        return 0.0
    return float(a) / float(b)


def _pct_improvement(before: float, after: float) -> float:
    if before == 0:
        return 0.0
    return (before - after) / before * 100.0


def _aggregate_block_b(doc: Dict[str, Any]) -> Dict[str, float]:
    tests = doc.get("tests", {})
    dist_vals: List[float] = []
    dist_worst: List[float] = []
    durations: List[float] = []
    collisions_total = 0.0
    resets_total = 0.0
    attempts_total = 0.0
    success_total = 0.0

    for tname, t in tests.items():
        durations.append(float(t.get("duration_avg", 0.0) or 0.0))
        collisions_total += float(t.get("collisions", 0.0) or 0.0)
        resets_total += float(t.get("resets", 0.0) or 0.0)
        attempts_total += float(t.get("attempts", 0.0) or 0.0)
        success_total += float(t.get("success", 0.0) or 0.0)

        # Keep distance metrics only for linear tasks.
        if "linear" in tname and t.get("distance_error_avg") is not None:
            dist_vals.append(float(t["distance_error_avg"]))
        if "linear" in tname and t.get("distance_error_max") is not None:
            dist_worst.append(float(t["distance_error_max"]))

    return {
        "distance_error_avg_linear": _mean(dist_vals),
        "distance_error_worst_linear": max(dist_worst) if dist_worst else 0.0,
        "duration_avg_all": _mean(durations),
        "collisions_total": collisions_total,
        "resets_total": resets_total,
        "success_rate": _ratio(int(success_total), int(attempts_total)) if attempts_total > 0 else 0.0,
    }


def _build_block_b_summary(docs: Dict[str, Dict[str, Any]]) -> Dict[str, Any]:
    agg: Dict[str, Dict[str, Any]] = {}
    for key, doc in docs.items():
        # key format: <method>_<profile> (e.g., cap_default, llm_moead)
        method, profile = key.split("_", 1)
        agg.setdefault(method, {})[profile] = _aggregate_block_b(doc)

    improvements: Dict[str, Dict[str, Any]] = {}
    for method in ("llm", "cap"):
        if method not in agg or "default" not in agg[method]:
            continue
        base = agg[method]["default"]
        improvements[method] = {}
        for profile in ("pygad", "moead"):
            if profile not in agg[method]:
                continue
            cur = agg[method][profile]
            improvements[method][profile] = {
                "distance_error_avg_linear_pct": _pct_improvement(
                    base["distance_error_avg_linear"], cur["distance_error_avg_linear"]
                ),
                "distance_error_worst_linear_pct": _pct_improvement(
                    base["distance_error_worst_linear"], cur["distance_error_worst_linear"]
                ),
                "duration_avg_all_pct": _pct_improvement(
                    base["duration_avg_all"], cur["duration_avg_all"]
                ),
            }

    method_compare: Dict[str, Any] = {}
    for profile in ("default", "pygad", "moead"):
        if "llm" in agg and "cap" in agg and profile in agg["llm"] and profile in agg["cap"]:
            cur = agg["llm"][profile]
            cap = agg["cap"][profile]
            method_compare[profile] = {
                "cap_vs_llm_distance_avg_linear_pct": _pct_improvement(
                    cur["distance_error_avg_linear"], cap["distance_error_avg_linear"]
                ),
                "cap_vs_llm_worst_linear_pct": _pct_improvement(
                    cur["distance_error_worst_linear"], cap["distance_error_worst_linear"]
                ),
                "cap_vs_llm_duration_avg_pct": _pct_improvement(
                    cur["duration_avg_all"], cap["duration_avg_all"]
                ),
            }

    return {
        "meta": {
            "timestamp": datetime.now().isoformat(),
            "description": "Block B summary: within-method generalization and cap-vs-llm comparison.",
        },
        "aggregates": agg,
        "improvements_vs_default": improvements,
        "method_comparison": method_compare,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run 4 scenarios (Block A + Block B).")
    parser.add_argument("--outdir", type=str, default="scenario_results")
    parser.add_argument(
        "--run-mode",
        type=str,
        default="all",
        choices=["all", "a", "b"],
        help="all: Block A + B, a: only Block A, b: only Block B",
    )
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--exec-timeout", type=float, default=90.0)
    parser.add_argument("--max-resets", type=int, default=5)
    parser.add_argument("--current-webhook", type=str, default="", help="Current parser URL (n8n/LLM).")
    parser.add_argument("--current-timeout", type=float, default=8.0)
    parser.add_argument(
        "--current-map",
        type=str,
        default="",
        help="Optional JSON map text->structured command for llm parser fallback.",
    )
    parser.add_argument(
        "--method-order",
        type=str,
        default="cap,llm",
        help="Execution order for parser methods, comma-separated. Example: cap,llm",
    )
    parser.add_argument(
        "--profiles",
        type=str,
        default="default,pygad,moead",
        help="Gene profiles to evaluate in Block B. Example: default,pygad",
    )
    parser.add_argument(
        "--pygad-genes",
        type=str,
        default="",
    )
    parser.add_argument(
        "--moead-genes",
        type=str,
        default="",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    args.run_block_a = args.run_mode in ("all", "a")
    args.run_block_b = args.run_mode in ("all", "b")
    rclpy.init()
    node = ScenarioRunner(args)
    try:
        results = node.run_all()
        print("\n=== Scenario outputs ===")
        for k, v in results.items():
            print(f"{k}: {v}")
        print("========================\n")
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
LLM parsing benchmark (CAP baseline vs LLM parser path).

Goal:
  Evaluate parser quality and latency with a fixed NL dataset and expected JSON.

Outputs (under --outdir/<run_id>/):
  - benchmark_results.json   (full records + summary)
  - benchmark_summary.csv    (global metrics)
  - benchmark_summary.md     (human-readable summary)
  - benchmark_cases.csv      (per-case details)
"""

import argparse
import csv
import json
import os
import statistics
import time
from datetime import datetime
from typing import Any, Dict, List, Optional, Tuple
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

try:
    from p3at_control.cap_parser import NUM_RANGES, parse_cap
except Exception:
    import sys

    here = os.path.dirname(os.path.abspath(__file__))
    pkg_dir = os.path.normpath(os.path.join(here, "..", "p3at_control"))
    if pkg_dir not in sys.path:
        sys.path.append(pkg_dir)
    from cap_parser import NUM_RANGES, parse_cap


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


def _request_json(url: str, payload: Dict[str, Any], timeout_s: float) -> Dict[str, Any]:
    data = json.dumps(payload).encode("utf-8")
    req = Request(url, data=data, headers={"Content-Type": "application/json"}, method="POST")
    with urlopen(req, timeout=timeout_s) as resp:
        raw = resp.read().decode("utf-8")
    if not raw.strip():
        raise ValueError("empty_response_body")
    return json.loads(raw)


def _extract_command_candidate(resp_obj: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    if not isinstance(resp_obj, dict):
        return None
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


def _command_valid(cmd: Optional[Dict[str, Any]]) -> Tuple[bool, List[str]]:
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
    if c == "set_genes" and not isinstance(cmd.get("value"), dict):
        issues.append("genes_not_dict")
    return len(issues) == 0, issues


def _approx_equal(a: Any, b: Any, tol: float) -> bool:
    if isinstance(a, bool) or isinstance(b, bool):
        return a is b
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        return abs(float(a) - float(b)) <= tol
    return a == b


def _json_match(got: Any, exp: Any, tol: float) -> bool:
    if type(got) != type(exp):
        # Allow int vs float comparisons
        if isinstance(got, (int, float)) and isinstance(exp, (int, float)):
            return _approx_equal(got, exp, tol)
        return False
    if isinstance(got, dict):
        if set(got.keys()) != set(exp.keys()):
            return False
        return all(_json_match(got[k], exp[k], tol) for k in got.keys())
    if isinstance(got, list):
        if len(got) != len(exp):
            return False
        return all(_json_match(g, e, tol) for g, e in zip(got, exp))
    return _approx_equal(got, exp, tol)


def _load_dataset(path: str) -> List[Dict[str, Any]]:
    with open(path, "r", encoding="utf-8") as f:
        obj = json.load(f)
    if isinstance(obj, list):
        cases = obj
    elif isinstance(obj, dict) and isinstance(obj.get("cases"), list):
        cases = obj["cases"]
    else:
        raise ValueError("Dataset must be a list or an object with key 'cases'.")
    for i, c in enumerate(cases):
        if not isinstance(c, dict) or not isinstance(c.get("text"), str):
            raise ValueError(f"Invalid case at index {i}: each case must contain string field 'text'.")
    return cases


def _pct(num: int, den: int) -> float:
    return 0.0 if den <= 0 else 100.0 * float(num) / float(den)


def _lat_stats(values_ms: List[float]) -> Dict[str, float]:
    if not values_ms:
        return {"mean_ms": 0.0, "p50_ms": 0.0, "p95_ms": 0.0, "p99_ms": 0.0}
    vals = sorted(values_ms)

    def q(p: float) -> float:
        idx = max(0, min(len(vals) - 1, int(round((len(vals) - 1) * p))))
        return vals[idx]

    return {
        "mean_ms": statistics.mean(vals),
        "p50_ms": q(0.50),
        "p95_ms": q(0.95),
        "p99_ms": q(0.99),
    }


def _normalize_candidate(candidate: Optional[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    if not isinstance(candidate, dict):
        return None
    normalized, _ = parse_cap(candidate)
    return normalized


def _call_llm_webhook(
    webhook_url: str,
    text: str,
    timeout_s: float,
    max_retries: int,
    retry_backoff_ms: float,
) -> Tuple[Optional[Dict[str, Any]], Dict[str, Any]]:
    """
    Calls webhook with retry/backoff and returns:
      (normalized command candidate or None, meta dict)
    """
    attempts_total = max(1, int(max_retries) + 1)
    last_error: Optional[str] = None

    for attempt in range(1, attempts_total + 1):
        try:
            resp = _request_json(webhook_url, {"text": text}, timeout_s=timeout_s)
            llm_raw = _extract_command_candidate(resp)
            if isinstance(llm_raw, dict):
                return llm_raw, {
                    "source": "llm_webhook",
                    "attempts": attempt,
                    "retries_used": attempt - 1,
                }
            last_error = "no_command_in_response"
        except (HTTPError, URLError, TimeoutError) as e:
            last_error = str(e)
        except Exception as e:
            last_error = str(e)

        if attempt < attempts_total and retry_backoff_ms > 0:
            # Exponential backoff to reduce burst/load in n8n + LLM provider.
            sleep_s = (retry_backoff_ms * (2 ** (attempt - 1))) / 1000.0
            time.sleep(sleep_s)

    return None, {
        "source": "llm_webhook",
        "error": last_error or "unknown_webhook_error",
        "attempts": attempts_total,
        "retries_used": max(0, attempts_total - 1),
    }


def _constraint_valid(candidate: Optional[Dict[str, Any]]) -> Tuple[bool, List[str]]:
    """
    Checks whether normalized command values are inside declared limits.
    """
    issues: List[str] = []
    if not isinstance(candidate, dict):
        return False, ["not_dict"]

    cmd = candidate.get("command")
    if cmd == "move_distance":
        v = candidate.get("value")
        if not isinstance(v, (int, float)) or not (NUM_RANGES["move_distance"][0] <= float(v) <= NUM_RANGES["move_distance"][1]):
            issues.append("move_distance_out_of_range")
    elif cmd == "turn_angle":
        v = candidate.get("value")
        if not isinstance(v, (int, float)) or not (NUM_RANGES["turn_angle"][0] <= float(v) <= NUM_RANGES["turn_angle"][1]):
            issues.append("turn_angle_out_of_range")
    elif cmd == "arc":
        val = candidate.get("value")
        if not isinstance(val, dict):
            issues.append("arc_not_dict")
        else:
            vv = val.get("v")
            ww = val.get("w")
            dd = val.get("duration")
            if not isinstance(vv, (int, float)) or not (NUM_RANGES["arc_v"][0] <= float(vv) <= NUM_RANGES["arc_v"][1]):
                issues.append("arc_v_out_of_range")
            if not isinstance(ww, (int, float)) or not (NUM_RANGES["arc_w"][0] <= float(ww) <= NUM_RANGES["arc_w"][1]):
                issues.append("arc_w_out_of_range")
            if not isinstance(dd, (int, float)) or not (NUM_RANGES["arc_duration"][0] <= float(dd) <= NUM_RANGES["arc_duration"][1]):
                issues.append("arc_duration_out_of_range")
    elif cmd in ("explore_mode", "validation_mode", "stop", "set_genes"):
        pass
    else:
        issues.append("unknown_command")

    return len(issues) == 0, issues


def run(args: argparse.Namespace) -> int:
    cases = _load_dataset(args.dataset)
    llm_map = {}
    if args.llm_map:
        with open(args.llm_map, "r", encoding="utf-8") as f:
            llm_map = json.load(f)

    run_id = args.run_id or datetime.now().strftime("run_%Y%m%d_%H%M%S")
    outdir = os.path.join(args.outdir, run_id)
    os.makedirs(outdir, exist_ok=True)

    records: List[Dict[str, Any]] = []
    by_parser = {
        "cap": {"valid": 0, "constraint_valid": 0, "exact": 0, "command_match": 0, "count_with_gt": 0, "latencies": []},
        "llm": {"valid": 0, "constraint_valid": 0, "exact": 0, "command_match": 0, "count_with_gt": 0, "latencies": []},
    }
    amb_by_parser = {
        "cap": {"exact": 0, "count_with_gt": 0},
        "llm": {"exact": 0, "count_with_gt": 0},
    }

    for idx, case in enumerate(cases):
        text = case["text"]
        case_id = case.get("id", f"case_{idx:03d}")
        category = str(case.get("category", "general"))
        is_amb = bool(case.get("ambiguous", False))

        expected_norm = None
        if isinstance(case.get("expected"), dict):
            expected_norm = _normalize_candidate(case["expected"])

        # CAP
        t0 = now_s()
        try:
            cap_raw, cap_meta = parse_cap({"text": text})
        except Exception as e:
            cap_raw, cap_meta = None, {"source": "cap", "error": str(e)}
        cap_ms = (now_s() - t0) * 1000.0
        cap_norm = _normalize_candidate(cap_raw)
        cap_valid, cap_issues = _command_valid(cap_norm)
        cap_constraint_valid, cap_constraint_issues = _constraint_valid(cap_norm)

        # LLM
        t1 = now_s()
        llm_raw = None
        llm_meta: Dict[str, Any] = {"source": "llm", "error": "no_llm_parser_source"}
        if text in llm_map and isinstance(llm_map[text], dict):
            llm_raw = llm_map[text]
            llm_meta = {"source": "llm_map"}
        elif args.llm_webhook:
            llm_raw, llm_meta = _call_llm_webhook(
                webhook_url=args.llm_webhook,
                text=text,
                timeout_s=args.llm_timeout,
                max_retries=args.max_retries,
                retry_backoff_ms=args.retry_backoff_ms,
            )
        llm_ms = (now_s() - t1) * 1000.0
        llm_norm = _normalize_candidate(llm_raw)
        llm_valid, llm_issues = _command_valid(llm_norm)
        llm_constraint_valid, llm_constraint_issues = _constraint_valid(llm_norm)

        row: Dict[str, Any] = {
            "id": case_id,
            "category": category,
            "ambiguous": is_amb,
            "text": text,
            "expected_normalized": expected_norm,
            "cap": {
                "latency_ms": cap_ms,
                "raw": cap_raw,
                "normalized": cap_norm,
                "valid": cap_valid,
                "issues": cap_issues,
                "constraint_valid": cap_constraint_valid,
                "constraint_issues": cap_constraint_issues,
                "meta": cap_meta,
            },
            "llm": {
                "latency_ms": llm_ms,
                "raw": llm_raw,
                "normalized": llm_norm,
                "valid": llm_valid,
                "issues": llm_issues,
                "constraint_valid": llm_constraint_valid,
                "constraint_issues": llm_constraint_issues,
                "meta": llm_meta,
            },
        }

        for parser_name, parsed in (("cap", cap_norm), ("llm", llm_norm)):
            by_parser[parser_name]["latencies"].append(row[parser_name]["latency_ms"])
            if row[parser_name]["valid"]:
                by_parser[parser_name]["valid"] += 1
            if row[parser_name]["constraint_valid"]:
                by_parser[parser_name]["constraint_valid"] += 1

            if isinstance(expected_norm, dict):
                by_parser[parser_name]["count_with_gt"] += 1
                exact = _json_match(parsed, expected_norm, args.tol) if isinstance(parsed, dict) else False
                row[parser_name]["exact_match"] = exact
                if exact:
                    by_parser[parser_name]["exact"] += 1

                got_cmd = parsed.get("command") if isinstance(parsed, dict) else None
                exp_cmd = expected_norm.get("command")
                cmd_match = got_cmd == exp_cmd
                row[parser_name]["command_match"] = cmd_match
                if cmd_match:
                    by_parser[parser_name]["command_match"] += 1

                if is_amb:
                    amb_by_parser[parser_name]["count_with_gt"] += 1
                    if exact:
                        amb_by_parser[parser_name]["exact"] += 1
            else:
                row[parser_name]["exact_match"] = None
                row[parser_name]["command_match"] = None

        records.append(row)
        if args.delay_ms > 0:
            time.sleep(args.delay_ms / 1000.0)

    summary = {
        "meta": {
            "timestamp": datetime.now().isoformat(),
            "dataset": os.path.abspath(args.dataset),
            "cases_total": len(records),
            "cases_with_ground_truth": by_parser["cap"]["count_with_gt"],
            "llm_source": "map+webhook" if args.llm_webhook and args.llm_map else ("webhook" if args.llm_webhook else "map"),
            "llm_webhook": args.llm_webhook,
            "delay_ms": args.delay_ms,
            "max_retries": args.max_retries,
            "retry_backoff_ms": args.retry_backoff_ms,
            "run_id": run_id,
        },
        "cap": {
            "valid_rate_pct": _pct(by_parser["cap"]["valid"], len(records)),
            "constraint_valid_rate_pct": _pct(by_parser["cap"]["constraint_valid"], len(records)),
            "exact_match_rate_pct": _pct(by_parser["cap"]["exact"], by_parser["cap"]["count_with_gt"]),
            "command_match_rate_pct": _pct(by_parser["cap"]["command_match"], by_parser["cap"]["count_with_gt"]),
            "latency": _lat_stats(by_parser["cap"]["latencies"]),
            "ambiguous_exact_match_rate_pct": _pct(amb_by_parser["cap"]["exact"], amb_by_parser["cap"]["count_with_gt"]),
        },
        "llm": {
            "valid_rate_pct": _pct(by_parser["llm"]["valid"], len(records)),
            "constraint_valid_rate_pct": _pct(by_parser["llm"]["constraint_valid"], len(records)),
            "exact_match_rate_pct": _pct(by_parser["llm"]["exact"], by_parser["llm"]["count_with_gt"]),
            "command_match_rate_pct": _pct(by_parser["llm"]["command_match"], by_parser["llm"]["count_with_gt"]),
            "latency": _lat_stats(by_parser["llm"]["latencies"]),
            "ambiguous_exact_match_rate_pct": _pct(amb_by_parser["llm"]["exact"], amb_by_parser["llm"]["count_with_gt"]),
        },
    }

    # Save full JSON
    with open(os.path.join(outdir, "benchmark_results.json"), "w", encoding="utf-8") as f:
        json.dump({"summary": summary, "records": records}, f, ensure_ascii=False, indent=2)

    # Summary CSV
    with open(os.path.join(outdir, "benchmark_summary.csv"), "w", encoding="utf-8", newline="") as f:
        w = csv.writer(f)
        w.writerow(["parser", "valid_rate_pct", "constraint_valid_rate_pct", "exact_match_rate_pct", "command_match_rate_pct",
                    "ambiguous_exact_match_rate_pct", "lat_mean_ms", "lat_p50_ms", "lat_p95_ms", "lat_p99_ms"])
        for p in ("cap", "llm"):
            lat = summary[p]["latency"]
            w.writerow([
                p,
                f"{summary[p]['valid_rate_pct']:.2f}",
                f"{summary[p]['constraint_valid_rate_pct']:.2f}",
                f"{summary[p]['exact_match_rate_pct']:.2f}",
                f"{summary[p]['command_match_rate_pct']:.2f}",
                f"{summary[p]['ambiguous_exact_match_rate_pct']:.2f}",
                f"{lat['mean_ms']:.2f}",
                f"{lat['p50_ms']:.2f}",
                f"{lat['p95_ms']:.2f}",
                f"{lat['p99_ms']:.2f}",
            ])

    # Cases CSV
    with open(os.path.join(outdir, "benchmark_cases.csv"), "w", encoding="utf-8", newline="") as f:
        w = csv.writer(f)
        w.writerow([
            "id", "category", "ambiguous", "text",
            "cap_valid", "cap_constraint_valid", "cap_exact_match", "cap_command_match", "cap_latency_ms",
            "llm_valid", "llm_constraint_valid", "llm_exact_match", "llm_command_match", "llm_latency_ms",
        ])
        for r in records:
            w.writerow([
                r["id"], r["category"], int(r["ambiguous"]), r["text"],
                int(r["cap"]["valid"]), int(r["cap"]["constraint_valid"]), r["cap"]["exact_match"], r["cap"]["command_match"], f"{r['cap']['latency_ms']:.2f}",
                int(r["llm"]["valid"]), int(r["llm"]["constraint_valid"]), r["llm"]["exact_match"], r["llm"]["command_match"], f"{r['llm']['latency_ms']:.2f}",
            ])

    # Summary MD
    md = []
    md.append("# LLM Parsing Benchmark\n")
    md.append(f"- Run: `{run_id}`")
    md.append(f"- Cases: **{summary['meta']['cases_total']}**")
    md.append(f"- Cases with ground truth: **{summary['meta']['cases_with_ground_truth']}**")
    md.append(f"- LLM source: `{summary['meta']['llm_source']}`\n")
    md.append("| Parser | Valid % | Constraint % | Exact % | Command % | Ambiguous Exact % | Mean ms | p50 | p95 | p99 |")
    md.append("|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|")
    for p in ("cap", "llm"):
        lat = summary[p]["latency"]
        md.append(
            f"| {p.upper()} | {summary[p]['valid_rate_pct']:.2f} | {summary[p]['constraint_valid_rate_pct']:.2f} | "
            f"{summary[p]['exact_match_rate_pct']:.2f} | "
            f"{summary[p]['command_match_rate_pct']:.2f} | {summary[p]['ambiguous_exact_match_rate_pct']:.2f} | "
            f"{lat['mean_ms']:.2f} | {lat['p50_ms']:.2f} | {lat['p95_ms']:.2f} | {lat['p99_ms']:.2f} |"
        )
    with open(os.path.join(outdir, "benchmark_summary.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(md) + "\n")

    print(f"[OK] Benchmark saved to: {outdir}")
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run CAP vs LLM parsing benchmark from NL dataset.")
    parser.add_argument("--dataset", required=True, help="JSON dataset file with benchmark cases.")
    parser.add_argument("--outdir", default="scenario_results/llm_parsing", help="Output base directory.")
    parser.add_argument("--run-id", default="", help="Optional run folder name. Default: timestamp.")
    parser.add_argument("--llm-map", default="", help="Optional JSON map {text: command_json} for LLM fallback.")
    parser.add_argument("--llm-webhook", default="", help="Optional webhook URL for LLM parser.")
    parser.add_argument("--llm-timeout", type=float, default=8.0, help="Webhook timeout (seconds).")
    parser.add_argument("--delay-ms", type=float, default=0.0, help="Sleep between cases (milliseconds).")
    parser.add_argument("--max-retries", type=int, default=2, help="Extra retries for each webhook call.")
    parser.add_argument("--retry-backoff-ms", type=float, default=250.0, help="Base retry backoff in milliseconds.")
    parser.add_argument("--tol", type=float, default=1e-3, help="Numeric tolerance for exact-match checks.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    os.makedirs(args.outdir, exist_ok=True)
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())

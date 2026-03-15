#!/usr/bin/env python3
"""
Build cap-vs-llm report artifacts from scenario JSON files:
  - main metrics table (csv + md + png)
  - avg-vs-worst figure (png)
  - summary json
"""

import argparse
import json
from pathlib import Path
from typing import Dict, Any, List, Tuple

import matplotlib.pyplot as plt
import numpy as np


def _load(path: Path) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def _mean(vals: List[float]) -> float:
    return float(sum(vals) / len(vals)) if vals else 0.0


def _std(vals: List[float]) -> float:
    if len(vals) < 2:
        return 0.0
    m = _mean(vals)
    var = sum((v - m) ** 2 for v in vals) / (len(vals) - 1)
    return float(var ** 0.5)


def _pct(before: float, after: float) -> float:
    if before == 0:
        return 0.0
    return (before - after) / before * 100.0


def _aggregate(doc: Dict[str, Any]) -> Dict[str, float]:
    tests = doc.get("tests", {})
    dist_attempts: List[float] = []
    durations: List[float] = []
    attempts = 0
    success = 0
    collisions = 0
    resets = 0

    for tname, t in tests.items():
        attempts += int(t.get("attempts", 0) or 0)
        success += int(t.get("success", 0) or 0)
        collisions += int(t.get("collisions", 0) or 0)
        resets += int(t.get("resets", 0) or 0)

        for d in t.get("details", []):
            if d.get("duration_s") is not None:
                durations.append(float(d["duration_s"]))
            if ("linear" in tname) and d.get("ok") and d.get("distance_error_m") is not None:
                dist_attempts.append(abs(float(d["distance_error_m"])))

    return {
        "distance_avg": _mean(dist_attempts),
        "distance_std": _std(dist_attempts),
        "distance_worst": max(dist_attempts) if dist_attempts else 0.0,
        "duration_avg": _mean(durations),
        "duration_std": _std(durations),
        "attempts": float(attempts),
        "success": float(success),
        "success_rate": (float(success) / float(attempts)) if attempts > 0 else 0.0,
        "collisions": float(collisions),
        "resets": float(resets),
    }


def _fmt_pm(v: float, s: float, digits: int = 3) -> str:
    return f"{v:.{digits}f} ± {s:.{digits}f}"


def _write_table_csv(path: Path, header: List[str], rows: List[List[str]]) -> None:
    with open(path, "w", encoding="utf-8") as f:
        f.write(",".join(header) + "\n")
        for r in rows:
            f.write(",".join(r) + "\n")


def _write_table_md(path: Path, header: List[str], rows: List[List[str]]) -> None:
    with open(path, "w", encoding="utf-8") as f:
        f.write("| " + " | ".join(header) + " |\n")
        f.write("| " + " | ".join(["---"] * len(header)) + " |\n")
        for r in rows:
            f.write("| " + " | ".join(r) + " |\n")


def _plot_table_png(path: Path, header: List[str], rows: List[List[str]]) -> None:
    fig_h = max(3.8, 0.55 * (len(rows) + 2))
    fig, ax = plt.subplots(figsize=(14.0, fig_h))
    ax.axis("off")
    table = ax.table(cellText=rows, colLabels=header, loc="center")
    table.auto_set_font_size(False)
    table.set_fontsize(9)
    table.scale(1.0, 1.45)
    plt.tight_layout()
    plt.savefig(path, dpi=200, bbox_inches="tight")
    plt.close(fig)


def _plot_avg_vs_worst(path: Path, stats: Dict[str, Dict[str, float]]) -> None:
    keys = ["cap_default", "cap_after", "llm_default", "llm_after"]
    labels = ["CAP Before", "CAP After", "LLM Before", "LLM After"]
    avg = [stats[k]["distance_avg"] for k in keys]
    avg_std = [stats[k]["distance_std"] for k in keys]
    worst = [stats[k]["distance_worst"] for k in keys]

    x = np.arange(len(labels))
    w = 0.36

    fig, ax = plt.subplots(figsize=(10, 5.2))
    ax.bar(x - w / 2, avg, w, yerr=avg_std, capsize=4, label="Average linear error (m)")
    ax.bar(x + w / 2, worst, w, label="Worst linear error (m)")
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_ylabel("Error (m)")
    ax.set_title("CAP vs LLM: Average vs Worst Linear Error")
    ax.grid(axis="y", alpha=0.25)
    ax.legend()
    plt.tight_layout()
    plt.savefig(path, dpi=200)
    plt.close(fig)


def build_report(input_dir: Path, profile: str, out_prefix: str) -> Dict[str, Any]:
    cap_default = _load(input_dir / "cap_default.json")
    llm_default = _load(input_dir / "llm_default.json")
    cap_after = _load(input_dir / f"cap_{profile}.json")
    llm_after = _load(input_dir / f"llm_{profile}.json")

    s_cap_def = _aggregate(cap_default)
    s_llm_def = _aggregate(llm_default)
    s_cap_after = _aggregate(cap_after)
    s_llm_after = _aggregate(llm_after)

    rows = [
        ["CAP", "Before", _fmt_pm(s_cap_def["distance_avg"], s_cap_def["distance_std"]), f"{s_cap_def['distance_worst']:.3f}",
         _fmt_pm(s_cap_def["duration_avg"], s_cap_def["duration_std"]), f"{100.0*s_cap_def['success_rate']:.1f}%",
         f"{int(s_cap_def['collisions'])}", f"{int(s_cap_def['resets'])}"],
        ["CAP", "After", _fmt_pm(s_cap_after["distance_avg"], s_cap_after["distance_std"]), f"{s_cap_after['distance_worst']:.3f}",
         _fmt_pm(s_cap_after["duration_avg"], s_cap_after["duration_std"]), f"{100.0*s_cap_after['success_rate']:.1f}%",
         f"{int(s_cap_after['collisions'])}", f"{int(s_cap_after['resets'])}"],
        ["LLM", "Before", _fmt_pm(s_llm_def["distance_avg"], s_llm_def["distance_std"]), f"{s_llm_def['distance_worst']:.3f}",
         _fmt_pm(s_llm_def["duration_avg"], s_llm_def["duration_std"]), f"{100.0*s_llm_def['success_rate']:.1f}%",
         f"{int(s_llm_def['collisions'])}", f"{int(s_llm_def['resets'])}"],
        ["LLM", "After", _fmt_pm(s_llm_after["distance_avg"], s_llm_after["distance_std"]), f"{s_llm_after['distance_worst']:.3f}",
         _fmt_pm(s_llm_after["duration_avg"], s_llm_after["duration_std"]), f"{100.0*s_llm_after['success_rate']:.1f}%",
         f"{int(s_llm_after['collisions'])}", f"{int(s_llm_after['resets'])}"],
    ]
    header = ["Method", "Phase", "Avg Linear Error (m)", "Worst Linear Error (m)", "Avg Duration (s)", "Success Rate", "Collisions", "Resets"]

    table_csv = input_dir / f"{out_prefix}_table.csv"
    table_md = input_dir / f"{out_prefix}_table.md"
    table_png = input_dir / f"{out_prefix}_table.png"
    fig_png = input_dir / f"{out_prefix}_avg_vs_worst.png"
    summary_json = input_dir / f"{out_prefix}_summary.json"

    _write_table_csv(table_csv, header, rows)
    _write_table_md(table_md, header, rows)
    _plot_table_png(table_png, header, rows)

    stats_plot = {
        "cap_default": s_cap_def,
        "cap_after": s_cap_after,
        "llm_default": s_llm_def,
        "llm_after": s_llm_after,
    }
    _plot_avg_vs_worst(fig_png, stats_plot)

    summary = {
        "profile": profile,
        "cap": {
            "before": s_cap_def,
            "after": s_cap_after,
            "improvement_pct": {
                "distance_avg": _pct(s_cap_def["distance_avg"], s_cap_after["distance_avg"]),
                "distance_worst": _pct(s_cap_def["distance_worst"], s_cap_after["distance_worst"]),
                "duration_avg": _pct(s_cap_def["duration_avg"], s_cap_after["duration_avg"]),
            },
        },
        "llm": {
            "before": s_llm_def,
            "after": s_llm_after,
            "improvement_pct": {
                "distance_avg": _pct(s_llm_def["distance_avg"], s_llm_after["distance_avg"]),
                "distance_worst": _pct(s_llm_def["distance_worst"], s_llm_after["distance_worst"]),
                "duration_avg": _pct(s_llm_def["duration_avg"], s_llm_after["duration_avg"]),
            },
        },
        "cap_vs_llm_pct": {
            "before_distance_avg": _pct(s_llm_def["distance_avg"], s_cap_def["distance_avg"]),
            "before_distance_worst": _pct(s_llm_def["distance_worst"], s_cap_def["distance_worst"]),
            "before_duration_avg": _pct(s_llm_def["duration_avg"], s_cap_def["duration_avg"]),
            "after_distance_avg": _pct(s_llm_after["distance_avg"], s_cap_after["distance_avg"]),
            "after_distance_worst": _pct(s_llm_after["distance_worst"], s_cap_after["distance_worst"]),
            "after_duration_avg": _pct(s_llm_after["duration_avg"], s_cap_after["duration_avg"]),
        },
        "artifacts": {
            "table_csv": str(table_csv),
            "table_md": str(table_md),
            "table_png": str(table_png),
            "avg_vs_worst_png": str(fig_png),
        },
    }

    with open(summary_json, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)

    return summary


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--input-dir", required=True)
    ap.add_argument("--profile", required=True, choices=["pygad", "moead"])
    ap.add_argument("--out-prefix", default="cap_vs_llm")
    args = ap.parse_args()

    summary = build_report(Path(args.input_dir), args.profile, args.out_prefix)
    print("[REPORT] Done.")
    for k, v in summary["artifacts"].items():
        print(f"[REPORT] {k}: {v}")


if __name__ == "__main__":
    main()

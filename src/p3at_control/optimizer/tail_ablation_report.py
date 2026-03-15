#!/usr/bin/env python3
"""
Build consolidated Tail-Shielding ON/OFF figures for CAP vs LLM.

Expected inputs (per method, per seed):
  scenario_results/<method>/tail_ablation/seed_<NN>/
    tail_ablation_cap_<method>_sNN.json
    tail_ablation_llm_<method>_sNN.json
"""

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


METRICS = [
    ("distance_avg_linear_m", "Distance avg (m)", "lower"),
    ("duration_avg_s", "Duration avg (s)", "lower"),
    ("collisions_total", "Collisions total", "lower"),
    ("resets_total", "Resets total", "lower"),
    ("min_front_avg_m", "Min front avg (m)", "higher"),
]


def _load(path: Path) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def build_method_figure(base: Path, method: str, seed: int) -> Path:
    seed_tag = f"s{seed:02d}"
    seed_dir = base / method / "tail_ablation" / f"seed_{seed:02d}"
    cap_json = seed_dir / f"tail_ablation_cap_{method}_{seed_tag}.json"
    llm_json = seed_dir / f"tail_ablation_llm_{method}_{seed_tag}.json"

    if not cap_json.exists() or not llm_json.exists():
        missing = [str(p) for p in (cap_json, llm_json) if not p.exists()]
        raise FileNotFoundError("Missing tail-ablation input files:\n - " + "\n - ".join(missing))

    cap = _load(cap_json)
    llm = _load(llm_json)

    labels = [m[1] for m in METRICS]
    x = np.arange(len(labels))
    w = 0.2

    cap_off = [cap["off"][k] for k, _, _ in METRICS]
    cap_on = [cap["on"][k] for k, _, _ in METRICS]
    llm_off = [llm["off"][k] for k, _, _ in METRICS]
    llm_on = [llm["on"][k] for k, _, _ in METRICS]

    cap_impr = [cap["improvement_pct_on_vs_off"][k] for k, _, _ in METRICS]
    llm_impr = [llm["improvement_pct_on_vs_off"][k] for k, _, _ in METRICS]

    fig, (ax0, ax1) = plt.subplots(2, 1, figsize=(12, 8), sharex=True, height_ratios=[2.2, 1.3])

    # Top: raw OFF/ON values for CAP and LLM
    ax0.bar(x - 1.5 * w, cap_off, w, label="CAP OFF", color="#b94a48")
    ax0.bar(x - 0.5 * w, cap_on, w, label="CAP ON", color="#5cb85c")
    ax0.bar(x + 0.5 * w, llm_off, w, label="LLM OFF", color="#f0ad4e")
    ax0.bar(x + 1.5 * w, llm_on, w, label="LLM ON", color="#428bca")
    ax0.set_title(
        f"Tail-Shielding Ablation ({method.upper()}, seed {seed:02d})\n"
        "Top: raw metrics (OFF/ON for CAP and LLM)"
    )
    ax0.grid(axis="y", alpha=0.25)
    ax0.legend(ncol=2, fontsize=9)

    # Bottom: improvement percentage ON vs OFF
    ax1.axhline(0.0, color="black", linewidth=1.0)
    ax1.bar(x - 0.18, cap_impr, 0.36, label="CAP improvement", color="#5cb85c")
    ax1.bar(x + 0.18, llm_impr, 0.36, label="LLM improvement", color="#428bca")
    ax1.set_ylabel("Improvement % (ON vs OFF)")
    ax1.set_xticks(x)
    ax1.set_xticklabels(labels, rotation=0)
    ax1.grid(axis="y", alpha=0.25)
    ax1.legend(fontsize=9)

    rule = (
        "Rule: positive improvement means Tail-Shielding ON is better than OFF. "
        "For min_front_avg, higher is better."
    )
    plt.figtext(0.5, 0.01, rule, ha="center", fontsize=9)
    plt.tight_layout(rect=[0.02, 0.04, 0.98, 0.98])

    out_png = seed_dir / f"tail_ablation_{method}_cap_vs_llm_on_off.png"
    plt.savefig(out_png, dpi=220)
    plt.close(fig)
    return out_png


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--method", choices=["pygad", "moead", "both"], default="both")
    ap.add_argument(
        "--base",
        default="src/p3at_control/optimizer/scenario_results",
        help="Base directory containing scenario_results/<method>/tail_ablation",
    )
    args = ap.parse_args()

    base = Path(args.base)
    methods = ["pygad", "moead"] if args.method == "both" else [args.method]
    for method in methods:
        out = build_method_figure(base=base, method=method, seed=int(args.seed))
        print(f"[OK] {out}")


if __name__ == "__main__":
    main()


#!/usr/bin/env python3
# -------------------------------------------------------
# compare_validation.py (Controller-aware + Learning Curve)
# -------------------------------------------------------
# - Compara BEFORE vs AFTER
# - Gera gráfico de comparação
# - Gera relatório Markdown
# - Gera curva de aprendizado a partir de validation_gXXX_iYY.json
# -------------------------------------------------------

import json
import os
import glob
import argparse
from pathlib import Path
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator
import numpy as np
import math
from datetime import datetime

# -------------------------------------------------------
# Utils
# -------------------------------------------------------

def load_json(path):
    with open(path, "r") as f:
        return json.load(f)

def pct_improvement(before, after, lower_is_better=True):
    if before == 0:
        return 0.0
    if lower_is_better:
        return (before - after) / before * 100.0
    return (after - before) / before * 100.0

def fmt_pct(v):
    emoji = "🟢" if v > 0 else "🔴"
    return f"{v:+.2f}% {emoji}"

# -------------------------------------------------------
# BEFORE vs AFTER
# -------------------------------------------------------

def compare_metrics(before, after):
    report = {}
    warnings = []

    for test_name, b in before["tests"].items():
        if test_name not in after["tests"]:
            warnings.append(f"{test_name}: missing in AFTER")
            continue

        a = after["tests"][test_name]
        rows = []
        identical = True

        if "distance_error_avg" in b and b["distance_error_avg"] is not None:
            if b["distance_error_avg"] != a.get("distance_error_avg"):
                identical = False
            rows.append({
                "metric": "distance_error_avg",
                "before": b["distance_error_avg"],
                "after": a["distance_error_avg"],
                "improvement": pct_improvement(
                    b["distance_error_avg"], a["distance_error_avg"]
                ),
            })

        if "angle_error_avg" in b and b["angle_error_avg"] is not None:
            if b["angle_error_avg"] != a.get("angle_error_avg"):
                identical = False
            rows.append({
                "metric": "angle_error_avg",
                "before": b["angle_error_avg"],
                "after": a["angle_error_avg"],
                "improvement": pct_improvement(
                    b["angle_error_avg"], a["angle_error_avg"]
                ),
            })

        if b.get("duration_avg") != a.get("duration_avg"):
            identical = False
        rows.append({
            "metric": "duration_avg",
            "before": b["duration_avg"],
            "after": a["duration_avg"],
            "improvement": pct_improvement(
                b["duration_avg"], a["duration_avg"]
            ),
        })

        if b.get("collisions") != a.get("collisions"):
            identical = False
        rows.append({
            "metric": "collisions",
            "before": b["collisions"],
            "after": a["collisions"],
            "improvement": pct_improvement(
                b["collisions"], a["collisions"]
            ),
        })

        if b.get("resets") != a.get("resets"):
            identical = False
        rows.append({
            "metric": "resets",
            "before": b["resets"],
            "after": a["resets"],
            "improvement": pct_improvement(
                b["resets"], a["resets"]
            ),
        })

        report[test_name] = rows
        if identical:
            warnings.append(f"{test_name}: identical metrics between BEFORE/AFTER")

    return report, warnings


def worst_case_error(before, after, filename="worst_case_error.png"):
    labels = []
    b_vals = []
    a_vals = []

    for t, b in before["tests"].items():
        if t not in after["tests"]:
            continue
        a = after["tests"][t]
        val_b = None
        val_a = None
        if b.get("distance_error_max") is not None and a.get("distance_error_max") is not None:
            val_b = float(b["distance_error_max"])
            val_a = float(a["distance_error_max"])
        if b.get("angle_error_max") is not None and a.get("angle_error_max") is not None:
            val_b = float(b["angle_error_max"])
            val_a = float(a["angle_error_max"])

        if val_b is None or val_a is None:
            continue
        labels.append(t)
        b_vals.append(val_b)
        a_vals.append(val_a)

    if not labels:
        return

    x = np.arange(len(labels))
    w = 0.35

    plt.figure(figsize=(12, 6))
    plt.bar(x - w/2, b_vals, w, label="Before")
    plt.bar(x + w/2, a_vals, w, label="After")
    plt.ylabel("Worst error (m or deg)")
    plt.title("Worst case per test – lower is better")
    plt.xticks(x, labels, rotation=30, ha="right")
    plt.grid(axis="y")
    plt.legend()
    plt.figtext(
        0.5, 0.01,
        "Before = blue | After = orange | Lower worst error = improvement",
        ha="center", fontsize=9,
        bbox={"facecolor": "white", "alpha": 0.85, "edgecolor": "none"}
    )
    plt.tight_layout(rect=[0, 0.05, 1, 0.98])
    plt.savefig(filename)
    print(f"📊 Plot saved: {filename}")


# -------------------------------------------------------
# Heatmap de robustez (success, collisions/attempt, resets/attempt)
# -------------------------------------------------------

def plot_robustness_heatmap(before, after, filename="robustness_heatmap.png"):
    metrics = ["success_rate", "collisions_per_attempt", "resets_per_attempt"]
    tests = []
    data = []

    for t, b in before["tests"].items():
        if t not in after["tests"]:
            continue
        a = after["tests"][t]
        attempts_b = max(1, int(b.get("attempts", 0)))
        attempts_a = max(1, int(a.get("attempts", 0)))
        row_before = [
            float(b.get("success", 0)) / attempts_b,
            float(b.get("collisions", 0)) / attempts_b,
            float(b.get("resets", 0)) / attempts_b,
        ]
        row_after = [
            float(a.get("success", 0)) / attempts_a,
            float(a.get("collisions", 0)) / attempts_a,
            float(a.get("resets", 0)) / attempts_a,
        ]
        tests.append(f"{t}\nB")
        data.append(row_before)
        tests.append(f"{t}\nA")
        data.append(row_after)

    if not data:
        return

    arr = np.array(data)
    fig, ax = plt.subplots(figsize=(len(tests)*0.6 + 3, 5))
    im = ax.imshow(arr.T, aspect="auto", cmap="coolwarm")
    ax.set_yticks(range(len(metrics)))
    ax.set_yticklabels(["Success rate (↑)", "Collisions/attempt (↓)", "Resets/attempt (↓)"])
    ax.set_xticks(range(len(tests)))
    ax.set_xticklabels(tests, rotation=45, ha="right")
    plt.colorbar(im, ax=ax, fraction=0.025, pad=0.04)
    ax.set_title("Robustness per test (Before=B / After=A)")
    plt.figtext(
        0.5, 0.01,
        "Success: higher is better | Collisions/Resets: lower is better | Even rows = Before, odd rows = After",
        ha="center", fontsize=9,
        bbox={"facecolor": "white", "alpha": 0.85, "edgecolor": "none"}
    )
    plt.tight_layout()
    plt.savefig(filename)
    print(f"📊 Plot saved: {filename}")


# -------------------------------------------------------
# Boxplot/CDF simplificado do erro por teste (linear/ang)
# -------------------------------------------------------

def _collect_errors(doc):
    errs = {}
    for tname, t in doc.get("tests", {}).items():
        vals = []
        for att in t.get("details", []):
            if not att.get("ok"):
                continue
            if att.get("distance_error_m") is not None:
                vals.append(abs(float(att["distance_error_m"])))
            elif att.get("angle_error_deg") is not None:
                vals.append(abs(float(att["angle_error_deg"])))
        errs[tname] = vals
    return errs


# -------------------------------------------------------
# Pior vs médio (erro e duração)
# -------------------------------------------------------

def plot_avg_vs_worst(before, after, filename="avg_vs_worst.png"):
    labels = []
    avg_vals = []
    worst_vals = []

    for t, b in before["tests"].items():
        if t not in after["tests"]:
            continue
        a = after["tests"][t]
        # escolher métrica de erro conforme tipo
        err_b = b.get("distance_error_avg") if b.get("distance_error_avg") is not None else b.get("angle_error_avg")
        err_a = a.get("distance_error_avg") if a.get("distance_error_avg") is not None else a.get("angle_error_avg")
        worst_b = b.get("distance_error_max") if b.get("distance_error_max") is not None else b.get("angle_error_max")
        worst_a = a.get("distance_error_max") if a.get("distance_error_max") is not None else a.get("angle_error_max")

        if err_b is None or err_a is None or worst_b is None or worst_a is None:
            continue

        labels.append(f"{t}\nB-avg")
        avg_vals.append(float(err_b))
        labels.append(f"{t}\nA-avg")
        avg_vals.append(float(err_a))
        labels.append(f"{t}\nB-max")
        worst_vals.append(float(worst_b))
        labels.append(f"{t}\nA-max")
        worst_vals.append(float(worst_a))

    if not labels or not avg_vals or not worst_vals:
        return

    x = np.arange(len(labels)//2)
    plt.figure(figsize=(max(10, len(labels)), 6))
    width = 0.35
    # Agrupar por par (avg, max) Before/After
    idx = 0
    avg_b = []
    avg_a = []
    max_b = []
    max_a = []
    for t, b in before["tests"].items():
        if t not in after["tests"]:
            continue
        a = after["tests"][t]
        err_b = b.get("distance_error_avg") if b.get("distance_error_avg") is not None else b.get("angle_error_avg")
        err_a = a.get("distance_error_avg") if a.get("distance_error_avg") is not None else a.get("angle_error_avg")
        worst_b = b.get("distance_error_max") if b.get("distance_error_max") is not None else b.get("angle_error_max")
        worst_a = a.get("distance_error_max") if a.get("distance_error_max") is not None else a.get("angle_error_max")
        if err_b is None or err_a is None or worst_b is None or worst_a is None:
            continue
        avg_b.append(float(err_b))
        avg_a.append(float(err_a))
        max_b.append(float(worst_b))
        max_a.append(float(worst_a))
        idx += 1

    x = np.arange(len(avg_b))
    plt.bar(x - width/2, avg_b, width, label="Before avg")
    plt.bar(x + width/2, avg_a, width, label="After avg")
    plt.bar(x - width/2, max_b, width, bottom=avg_b, alpha=0.3, label="Before max")
    plt.bar(x + width/2, max_a, width, bottom=avg_a, alpha=0.3, label="After max")
    plt.xticks(x, list(before["tests"].keys())[:len(avg_b)], rotation=45, ha="right")
    plt.ylabel("Error (m or deg)")
    plt.title("Average error vs worst case per test (lower is better)")
    plt.legend()
    plt.figtext(
        0.5, 0.01,
        "Solid bars = average error (lower is better); transparency = worst case (lower is better)",
        ha="center", fontsize=9,
        bbox={"facecolor": "white", "alpha": 0.85, "edgecolor": "none"}
    )
    plt.tight_layout(rect=[0, 0.05, 1, 0.98])
    plt.savefig(filename)
    print(f"📊 Plot saved: {filename}")


# -------------------------------------------------------
# Clearance e tempo parado/desvio
# -------------------------------------------------------

def plot_clearance_and_time(before, after, filename="clearance_time.png"):
    return  # removido


# -------------------------------------------------------
# Resets e tempo total por teste
# -------------------------------------------------------

def plot_resets_and_duration(before, after, filename="resets_duration.png"):
    tests = []
    b_resets = []
    a_resets = []
    b_time = []
    a_time = []

    for t, b in before["tests"].items():
        if t not in after["tests"]:
            continue
        a = after["tests"][t]
        tests.append(t)
        b_resets.append(int(b.get("resets", 0)))
        a_resets.append(int(a.get("resets", 0)))
        b_time.append(float(b.get("duration_avg", 0.0)))
        a_time.append(float(a.get("duration_avg", 0.0)))

    if not tests:
        return

    x = np.arange(len(tests))
    w = 0.35
    fig, ax1 = plt.subplots(figsize=(12, 6))
    ax1.bar(x - w/2, b_resets, w, label="Resets Before", color="#1f77b4")
    ax1.bar(x + w/2, a_resets, w, label="Resets After", color="#ff7f0e")
    ax1.set_ylabel("Resets (lower is better)")
    ax1.set_xticks(x)
    ax1.set_xticklabels(tests, rotation=45, ha="right")
    ax1.grid(axis="y")
    ax1.legend(loc="upper left")

    ax2 = ax1.twinx()
    ax2.plot(x, b_time, "-o", color="#2ca02c", label="Dur Before")
    ax2.plot(x, a_time, "-x", color="#d62728", label="Dur After")
    ax2.set_ylabel("Average duration (s) (lower is better)")
    ax2.legend(loc="upper right")

    plt.title("Resets and average duration per test")
    plt.tight_layout()
    plt.savefig(filename)
    print(f"📊 Plot saved: {filename}")


# -------------------------------------------------------
# Curva de fitness decomposta (parcial)
# -------------------------------------------------------

# -------------------------------------------------------
# Curva de aprendizado
# -------------------------------------------------------
# -------------------------------------------------------
# Plot BEFORE vs AFTER
# -------------------------------------------------------

def _parse_linear_distance(name: str):
    if not name.startswith("linear_") or not name.endswith("m"):
        return None
    try:
        return float(name.split("_")[1].replace("m", ""))
    except Exception:
        return None


def _extract_error_metric(test_entry):
    if test_entry.get("distance_error_avg") is not None:
        return float(test_entry["distance_error_avg"])
    if test_entry.get("angle_error_avg") is not None:
        return float(test_entry["angle_error_avg"])
    return None

def plot_time_vs_error(before, after, filename="time_vs_error.png"):
    b_x = []
    b_y = []
    a_x = []
    a_y = []

    for t, b in before["tests"].items():
        if t not in after["tests"]:
            continue
        a = after["tests"][t]

        b_err = _extract_error_metric(b)
        a_err = _extract_error_metric(a)
        if b_err is None or a_err is None:
            continue

        b_x.append(float(b.get("duration_avg", 0.0)))
        b_y.append(b_err)
        a_x.append(float(a.get("duration_avg", 0.0)))
        a_y.append(a_err)

    if not b_x:
        return

    plt.figure(figsize=(8, 6))
    plt.scatter(b_x, b_y, label="Before (blue/circle)", marker="o", color="#1f77b4")
    plt.scatter(a_x, a_y, label="After (orange/x)", marker="x", color="#ff7f0e")
    plt.xlabel("Average time (s)")
    plt.ylabel("Average error (m or deg)")
    plt.title("Time vs Error")
    plt.legend()
    plt.figtext(
        0.5, 0.97,
        "Before = blue/circle | After = orange/x | Lower time and lower error are better",
        ha="center", fontsize=9,
        bbox={"facecolor": "white", "alpha": 0.85, "edgecolor": "none"}
    )
    plt.grid(True)
    plt.tight_layout(rect=[0, 0, 1, 0.93])
    plt.savefig(filename)
    print(f"📊 Plot saved: {filename}")

# -------------------------------------------------------


# -------------------------------------------------------
# Learning Curve
# -------------------------------------------------------

def score_validation(doc):
    score = 0.0
    count = 0

    for t in doc["tests"].values():
        if t.get("distance_error_avg") is not None:
            score += t["distance_error_avg"]
            count += 1

        if t.get("angle_error_avg") is not None:
            score += t["angle_error_avg"]
            count += 1

        score += 0.01 * t.get("duration_avg", 0.0)
        score += 0.5 * t.get("collisions", 0)
        score += 0.5 * t.get("resets", 0)

    return score / max(count, 1)

def plot_learning_curve(
    curve_json="learning_curve.json",
    filename="learning_curve.png",
    mode="absolute",
    y_step=None,
):
    if not os.path.exists(curve_json):
        print(f"⚠️ {curve_json} not found. Skipping learning curve.")
        return

    with open(curve_json, "r") as f:
        curve = json.load(f)

    if not curve:
        print("⚠️ learning_curve.json empty.")
        return

    generations = [int(c["generation"]) for c in curve]
    best_fitness = [float(c["best_fitness"]) for c in curve]

    if mode == "delta":
        best_value = max(best_fitness)
        y_values = [v - best_value for v in best_fitness]
        y_label = "Fitness delta to best (0 = best)"
        title = "Learning Curve - Delta to global best"
    else:
        y_values = best_fitness
        y_label = "Fitness (higher is better)"
        title = "Learning Curve - Best individual per generation"

    plt.figure(figsize=(10, 5))
    plt.plot(generations, y_values, marker="o")
    plt.xticks(sorted(set(generations)))
    plt.gca().xaxis.set_major_locator(MultipleLocator(1))
    if y_step is not None and y_step > 0:
        plt.gca().yaxis.set_major_locator(MultipleLocator(y_step))
    plt.xlabel("Generation")
    plt.ylabel(y_label)
    plt.title(title)
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(filename)

    print(f"📈 Learning curve saved: {filename}")


# -------------------------------------------------------
# Markdown
# -------------------------------------------------------
def render_markdown(report, warnings=None, figures=None, learning_curve_json="learning_curve.json"):
    figures = figures or {}
    lines = []
    lines.append("# 📊 P3AT Optimization Report\n")
    lines.append(f"**Date:** {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")

    if warnings:
        lines.append("## Warnings\n")
        for w in warnings:
            lines.append(f"- {w}")
        lines.append("")

    # -----------------------------
    # Métricas principais
    # -----------------------------
    time_vs_error = figures.get("time_vs_error", "time_vs_error.png")
    robustness_heatmap = figures.get("robustness_heatmap", "robustness_heatmap.png")
    avg_vs_worst = figures.get("avg_vs_worst", "avg_vs_worst.png")
    resets_duration = figures.get("resets_duration", "resets_duration.png")
    worst_case = figures.get("worst_case_error", "worst_case_error.png")
    learning_curve_png = figures.get("learning_curve", "learning_curve.png")

    if os.path.exists(time_vs_error):
        lines.append(f"![Time vs Error]({time_vs_error})\n")

    if os.path.exists(robustness_heatmap):
        lines.append("## Robustness (success/collision/reset)\n")
        lines.append(f"![Robustness]({robustness_heatmap})\n")

    if os.path.exists(avg_vs_worst):
        lines.append("## Average error vs worst case\n")
        lines.append(f"![Avg vs Worst]({avg_vs_worst})\n")

    if os.path.exists(resets_duration):
        lines.append("## Resets and average duration\n")
        lines.append(f"![Resets/Duração]({resets_duration})\n")

    if os.path.exists(worst_case):
        lines.append("## Worst error per test\n")
        lines.append(f"![Worst Error]({worst_case})\n")

    for test, rows in report.items():
        lines.append(f"### Test: {test}")
        lines.append("| Metric | Before | After | Improvement |")
        lines.append("|--------|--------|-------|-------------|")

        for r in rows:
            lines.append(
                f"| {r['metric']} | {r['before']:.4f} | {r['after']:.4f} | {fmt_pct(r['improvement'])} |"
            )
        lines.append("")

    # -----------------------------
    # Curva de aprendizado
    # -----------------------------
    if os.path.exists(learning_curve_json):
        with open(learning_curve_json, "r") as f:
            curve = json.load(f)

        if curve:
            best = curve[-1]["best_genes"]

            lines.append("## Learning Curve\n")
            lines.append(f"![Learning Curve]({learning_curve_png})\n")

            lines.append("### Best genes found\n")
            lines.append("| Gene | Value |")
            lines.append("|------|-------|")

            for k, v in best.items():
                lines.append(f"| {k} | {v:.4f} |")

            lines.append("")

    return "\n".join(lines)

# -------------------------------------------------------
# MAIN
# -------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Compare BEFORE vs AFTER validation JSON files.")
    parser.add_argument("--before", default="validation_before.json", help="Path to BEFORE JSON")
    parser.add_argument("--after", default="validation_after.json", help="Path to AFTER JSON")
    parser.add_argument("--out", default="validation_report.md", help="Path to output markdown report")
    parser.add_argument("--figure-prefix", default="", help="Prefix for generated figure filenames")
    parser.add_argument("--learning-curve", default="learning_curve.json", help="Path to learning curve JSON")
    parser.add_argument(
        "--learning-curve-mode",
        default="absolute",
        choices=["absolute", "delta"],
        help="Learning curve style: absolute fitness or delta to best",
    )
    parser.add_argument(
        "--learning-curve-y-step",
        type=float,
        default=None,
        help="Optional y-axis tick step for learning curve (e.g. 0.5, 1.0)",
    )
    args = parser.parse_args()

    before = load_json(args.before)
    after  = load_json(args.after)

    figure_prefix = str(args.figure_prefix).strip()
    if not figure_prefix:
        stem = Path(args.out).stem
        figure_prefix = stem.replace("validation_report_", "")

    def fig_name(base):
        return f"{figure_prefix}_{base}" if figure_prefix else base

    report, warnings = compare_metrics(before, after)

    figures = {
        "robustness_heatmap": fig_name("robustness_heatmap.png"),
        "avg_vs_worst": fig_name("avg_vs_worst.png"),
        "resets_duration": fig_name("resets_duration.png"),
        "worst_case_error": fig_name("worst_case_error.png"),
        "time_vs_error": fig_name("time_vs_error.png"),
        "learning_curve": fig_name("learning_curve.png"),
    }

    plot_robustness_heatmap(before, after, filename=figures["robustness_heatmap"])
    plot_avg_vs_worst(before, after, filename=figures["avg_vs_worst"])
    plot_resets_and_duration(before, after, filename=figures["resets_duration"])
    worst_case_error(before, after, filename=figures["worst_case_error"])
    plot_time_vs_error(before, after, filename=figures["time_vs_error"])
    plot_learning_curve(
        curve_json=args.learning_curve,
        filename=figures["learning_curve"],
        mode=args.learning_curve_mode,
        y_step=args.learning_curve_y_step,
    )

    if warnings:
        print("⚠️ Warnings:")
        for w in warnings:
            print(f" - {w}")

    md = render_markdown(report, warnings, figures=figures, learning_curve_json=args.learning_curve)

    with open(args.out, "w") as f:
        f.write(md)

    print(f"📄 Report saved: {args.out}")



if __name__ == "__main__":
    main()

#!/usr/bin/env python3
import csv
import json
import math
import os
import statistics
from pathlib import Path

import matplotlib.pyplot as plt


ROOT = Path(__file__).resolve().parent
OUTDIR = ROOT / "scenario_results" / "moead" / "paper_summary"
SEEDS = [43, 44, 45, 46]


def load_json(path: Path):
    with path.open() as f:
        return json.load(f)


def mean(xs):
    return sum(xs) / len(xs) if xs else 0.0


def std(xs):
    return statistics.stdev(xs) if len(xs) > 1 else 0.0


def summarize_validation(path: Path):
    doc = load_json(path)
    tests = doc["tests"]
    dist_avg = []
    dist_max = []
    durations = []
    min_front = []
    collisions = 0
    resets = 0
    success = 0
    attempts = 0
    for test in tests.values():
        attempts += int(test.get("attempts", 0) or 0)
        success += int(test.get("success", 0) or 0)
        collisions += int(test.get("collisions", 0) or 0)
        resets += int(test.get("resets", 0) or 0)
        if test.get("duration_avg") is not None:
            durations.append(float(test["duration_avg"]))
        if test.get("distance_error_avg") is not None:
            dist_avg.append(float(test["distance_error_avg"]))
        if test.get("distance_error_max") is not None:
            dist_max.append(float(test["distance_error_max"]))
        if test.get("min_front_avg") is not None:
            min_front.append(float(test["min_front_avg"]))
    return {
        "distance_error_avg_linear": mean(dist_avg),
        "distance_error_worst_linear": max(dist_max) if dist_max else 0.0,
        "duration_avg_all": mean(durations),
        "collisions_total": float(collisions),
        "resets_total": float(resets),
        "success_rate": (float(success) / float(attempts)) if attempts else 0.0,
        "min_front_avg": mean(min_front),
    }


def aggregate_rows(rows, metrics):
    out = {}
    for metric in metrics:
        vals = [float(r[metric]) for r in rows]
        out[metric] = {
            "mean": mean(vals),
            "std": std(vals),
        }
    return out


def fmt_pm(m, s, digits=4):
    return f"{m:.{digits}f} +- {s:.{digits}f}"


def write_csv(path: Path, header, rows):
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(rows)


def write_md(path: Path, title: str, header, rows):
    with path.open("w") as f:
        f.write(f"# {title}\n\n")
        f.write("| " + " | ".join(header) + " |\n")
        f.write("|" + "|".join(["---"] * len(header)) + "|\n")
        for row in rows:
            f.write("| " + " | ".join(str(x) for x in row) + " |\n")


def plot_grouped(path: Path, title: str, labels, series):
    fig, ax = plt.subplots(figsize=(8.8, 4.8))
    width = 0.35 if len(series) <= 2 else 0.22
    x = range(len(labels))
    offsets = []
    if len(series) == 2:
        offsets = [-width / 2, width / 2]
    elif len(series) == 3:
        offsets = [-width, 0.0, width]
    elif len(series) == 4:
        offsets = [-1.5 * width, -0.5 * width, 0.5 * width, 1.5 * width]
    else:
        offsets = [0.0] * len(series)

    for idx, (name, vals) in enumerate(series.items()):
        ax.bar([i + offsets[idx] for i in x], vals, width=width, label=name)

    ax.set_xticks(list(x))
    ax.set_xticklabels(labels, rotation=12, ha="right")
    ax.set_title(title)
    ax.grid(axis="y", alpha=0.25)
    ax.legend(frameon=False)
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def plot_env_shift(path: Path, env_summary):
    labels = [
        "Err avg",
        "Duration",
        "Collisions",
        "Resets",
        "Front clr",
    ]
    fig, axes = plt.subplots(1, 2, figsize=(10.8, 4.6), sharey=False)
    for ax, parser in zip(axes, ["cap", "llm"]):
        parser_doc = env_summary[parser]
        series = {
            "Baseline": [
                parser_doc["baseline"]["distance_error_avg_linear"]["mean"],
                parser_doc["baseline"]["duration_avg_all"]["mean"],
                parser_doc["baseline"]["collisions_total"]["mean"],
                parser_doc["baseline"]["resets_total"]["mean"],
                parser_doc["baseline"]["min_front_avg"]["mean"],
            ],
            "Unseen ON": [
                parser_doc["on"]["distance_error_avg_linear"]["mean"],
                parser_doc["on"]["duration_avg_all"]["mean"],
                parser_doc["on"]["collisions_total"]["mean"],
                parser_doc["on"]["resets_total"]["mean"],
                parser_doc["on"]["min_front_avg"]["mean"],
            ],
            "Unseen OFF": [
                parser_doc["off"]["distance_error_avg_linear"]["mean"],
                parser_doc["off"]["duration_avg_all"]["mean"],
                parser_doc["off"]["collisions_total"]["mean"],
                parser_doc["off"]["resets_total"]["mean"],
                parser_doc["off"]["min_front_avg"]["mean"],
            ],
        }
        width = 0.24
        x = range(len(labels))
        offsets = [-width, 0.0, width]
        for idx, (name, vals) in enumerate(series.items()):
            ax.bar([i + offsets[idx] for i in x], vals, width=width, label=name)
        ax.set_title(parser.upper())
        ax.set_xticks(list(x))
        ax.set_xticklabels(labels, rotation=18, ha="right")
        ax.grid(axis="y", alpha=0.25)
    axes[0].legend(frameon=False, loc="upper left")
    fig.suptitle("MOEA/D environment shift summary")
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def build_baseline():
    metrics = [
        "distance_error_avg_linear",
        "distance_error_worst_linear",
        "duration_avg_all",
        "collisions_total",
        "resets_total",
        "success_rate",
        "min_front_avg",
    ]
    baseline = {}
    seed_rows = []
    for parser in ["cap", "llm"]:
        rows = []
        for seed in SEEDS:
            path = ROOT / f"scenario_results/moead/gen_{parser}/seed_{seed}/validation_{parser}_after_moead_s{seed}.json"
            row = summarize_validation(path)
            row["seed"] = seed
            rows.append(row)
        baseline[parser] = aggregate_rows(rows, metrics)
        for row in rows:
            seed_rows.append(
                [
                    parser,
                    row["seed"],
                    row["distance_error_avg_linear"],
                    row["distance_error_worst_linear"],
                    row["duration_avg_all"],
                    row["collisions_total"],
                    row["resets_total"],
                    row["success_rate"],
                    row["min_front_avg"],
                ]
            )

    write_csv(
        OUTDIR / "moead_baseline_cap_vs_llm.csv",
        [
            "parser",
            "seed",
            "distance_error_avg_linear",
            "distance_error_worst_linear",
            "duration_avg_all",
            "collisions_total",
            "resets_total",
            "success_rate",
            "min_front_avg",
        ],
        seed_rows,
    )

    summary_rows = []
    for metric in metrics:
        summary_rows.append(
            [
                metric,
                fmt_pm(baseline["cap"][metric]["mean"], baseline["cap"][metric]["std"]),
                fmt_pm(baseline["llm"][metric]["mean"], baseline["llm"][metric]["std"]),
            ]
        )
    write_md(
        OUTDIR / "moead_baseline_cap_vs_llm.md",
        "MOEA/D baseline parser summary",
        ["Metric", "CAP", "LLM"],
        summary_rows,
    )

    plot_grouped(
        OUTDIR / "moead_baseline_cap_vs_llm.png",
        "MOEA/D baseline parser comparison",
        ["Err avg", "Worst err", "Duration", "Resets", "Front clr"],
        {
            "CAP": [
                baseline["cap"]["distance_error_avg_linear"]["mean"],
                baseline["cap"]["distance_error_worst_linear"]["mean"],
                baseline["cap"]["duration_avg_all"]["mean"],
                baseline["cap"]["resets_total"]["mean"],
                baseline["cap"]["min_front_avg"]["mean"],
            ],
            "LLM": [
                baseline["llm"]["distance_error_avg_linear"]["mean"],
                baseline["llm"]["distance_error_worst_linear"]["mean"],
                baseline["llm"]["duration_avg_all"]["mean"],
                baseline["llm"]["resets_total"]["mean"],
                baseline["llm"]["min_front_avg"]["mean"],
            ],
        },
    )
    return baseline


def build_tail():
    metrics = [
        "distance_avg_linear_m",
        "duration_avg_s",
        "collisions_total",
        "resets_total",
        "min_front_avg_m",
    ]
    tail = {}
    md_rows = []
    csv_rows = []
    for parser in ["cap", "llm"]:
        docs = [
            load_json(ROOT / f"scenario_results/moead/tail_ablation/seed_{seed}/tail_ablation_{parser}_moead_s{seed}.json")
            for seed in SEEDS
        ]
        tail[parser] = {}
        for section in ["off", "on", "improvement_pct_on_vs_off"]:
            tail[parser][section] = {}
            for metric in metrics:
                vals = [float(doc[section][metric]) for doc in docs]
                tail[parser][section][metric] = {"mean": mean(vals), "std": std(vals)}
                csv_rows.append([parser, section, metric, mean(vals), std(vals)])
        md_rows.append([f"{parser.upper()} OFF", "", ""])
        for metric in metrics:
            md_rows.append(
                [
                    metric,
                    fmt_pm(tail[parser]["off"][metric]["mean"], tail[parser]["off"][metric]["std"]),
                    fmt_pm(tail[parser]["on"][metric]["mean"], tail[parser]["on"][metric]["std"]),
                ]
            )

    write_csv(
        OUTDIR / "moead_tail_ablation_summary.csv",
        ["parser", "section", "metric", "mean", "std"],
        csv_rows,
    )
    write_md(
        OUTDIR / "moead_tail_ablation_summary.md",
        "MOEA/D tail ablation summary",
        ["Metric", "OFF", "ON"],
        md_rows,
    )
    plot_grouped(
        OUTDIR / "moead_tail_ablation_aggregate.png",
        "MOEA/D tail ablation aggregate",
        ["CAP err", "CAP dur", "CAP front", "LLM err", "LLM dur", "LLM front"],
        {
            "OFF": [
                tail["cap"]["off"]["distance_avg_linear_m"]["mean"],
                tail["cap"]["off"]["duration_avg_s"]["mean"],
                tail["cap"]["off"]["min_front_avg_m"]["mean"],
                tail["llm"]["off"]["distance_avg_linear_m"]["mean"],
                tail["llm"]["off"]["duration_avg_s"]["mean"],
                tail["llm"]["off"]["min_front_avg_m"]["mean"],
            ],
            "ON": [
                tail["cap"]["on"]["distance_avg_linear_m"]["mean"],
                tail["cap"]["on"]["duration_avg_s"]["mean"],
                tail["cap"]["on"]["min_front_avg_m"]["mean"],
                tail["llm"]["on"]["distance_avg_linear_m"]["mean"],
                tail["llm"]["on"]["duration_avg_s"]["mean"],
                tail["llm"]["on"]["min_front_avg_m"]["mean"],
            ],
        },
    )
    return tail


def build_env():
    metrics = [
        "distance_error_avg_linear",
        "duration_avg_all",
        "collisions_total",
        "resets_total",
        "success_rate",
        "min_front_avg",
    ]
    env = {}
    rows = []
    for parser in ["cap", "llm"]:
        base_rows = []
        on_rows = []
        off_rows = []
        for seed in SEEDS:
            base_rows.append(
                summarize_validation(ROOT / f"scenario_results/moead/gen_{parser}/seed_{seed}/validation_{parser}_after_moead_s{seed}.json")
            )
            suffix = f"s{seed}_llm" if parser == "llm" else f"s{seed}"
            on_rows.append(
                summarize_validation(ROOT / f"scenario_results/moead/env_shift/{parser}/seed_{seed}/validation_test_20x30eq_on_{suffix}.json")
            )
            off_rows.append(
                summarize_validation(ROOT / f"scenario_results/moead/env_shift/{parser}/seed_{seed}/validation_test_20x30eq_off_{suffix}.json")
            )
        env[parser] = {
            "baseline": aggregate_rows(base_rows, metrics),
            "on": aggregate_rows(on_rows, metrics),
            "off": aggregate_rows(off_rows, metrics),
        }
        for condition in ["baseline", "on", "off"]:
            for metric in metrics:
                rows.append(
                    [
                        parser,
                        condition,
                        metric,
                        env[parser][condition][metric]["mean"],
                        env[parser][condition][metric]["std"],
                    ]
                )
    write_csv(
        OUTDIR / "moead_env_shift_summary.csv",
        ["parser", "condition", "metric", "mean", "std"],
        rows,
    )
    md_rows = []
    for parser in ["cap", "llm"]:
        md_rows.append([f"{parser.upper()} baseline", "", ""])
        for metric in metrics:
            md_rows.append(
                [
                    metric,
                    fmt_pm(env[parser]["baseline"][metric]["mean"], env[parser]["baseline"][metric]["std"]),
                    fmt_pm(env[parser]["on"][metric]["mean"], env[parser]["on"][metric]["std"]),
                    fmt_pm(env[parser]["off"][metric]["mean"], env[parser]["off"][metric]["std"]),
                ]
            )
    write_md(
        OUTDIR / "moead_env_shift_summary.md",
        "MOEA/D environment shift summary",
        ["Metric", "Baseline", "Unseen ON", "Unseen OFF"],
        md_rows,
    )
    plot_env_shift(OUTDIR / "moead_env_shift_aggregate.png", env)
    return env


def main():
    OUTDIR.mkdir(parents=True, exist_ok=True)
    baseline = build_baseline()
    tail = build_tail()
    env = build_env()
    summary = {
        "baseline": baseline,
        "tail_ablation": tail,
        "env_shift": env,
    }
    with (OUTDIR / "moead_paper_summary.json").open("w") as f:
        json.dump(summary, f, indent=2)
    print(f"Saved paper summary artifacts in {OUTDIR}")


if __name__ == "__main__":
    main()

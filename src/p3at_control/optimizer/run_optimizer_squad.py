#!/usr/bin/env python3
# --------------------------------------------------------------------------------------
# FILE: run_optimizer_squad.py
#
# Examples:
#   python3 run_optimizer_squad.py --pygad --llm
#   python3 run_optimizer_squad.py --moead --cap
#   python3 run_optimizer_squad.py --pygad --compare
#
# What this script does:
#   Normal mode (--cap or --llm):
#     1) validator BEFORE (classic, optimizer-centric)
#     2) run chosen optimizer
#     3) validator AFTER  (classic, optimizer-centric)
#     4) compare_validation (classic)
#     5) scenario_runner Block B for parser-aware before/after with selected parser
#
#   Compare mode (--compare):
#     - runs scenario_runner Block B for BOTH parsers (cap,llm)
#       using profiles: default + chosen optimizer profile
#       and writes cap_vs_llm summary files.
# --------------------------------------------------------------------------------------

import argparse
import json
import subprocess
import sys
from pathlib import Path


def run_cmd(cmd, cwd=None, timeout=None, env=None):
    print("\n[SQUAD] RUN:", " ".join(cmd))
    p = subprocess.run(
        cmd,
        cwd=cwd,
        env=env,
        timeout=timeout,
        text=True,
    )
    if p.returncode != 0:
        raise SystemExit(f"[SQUAD] ERROR code={p.returncode}: {' '.join(cmd)}")


# Default webhook for llm mode (edit here if n8n IP/port changes)
DEFAULT_LLM_WEBHOOK = "http://192.168.18.121:5000/command"


def optimizer_info(args, here: Path):
    if args.moead:
        return {
            "key": "moead",
            "name": "MOEAD/IRECED",
            "script": here / "moead_ireced_optimizer.py",
            "profile": "moead",
            "best_name": "best_genes_moead.json",
            "curve_name": "learning_curve.json",
        }
    if args.pygad:
        return {
            "key": "pygad",
            "name": "PyGAD",
            "script": here / "pygad_optimizer.py",
            "profile": "pygad",
            "best_name": "best_genes_pygad.json",
            "curve_name": "learning_curve.json",
        }
    return {
        "key": "nsga2",
        "name": "NSGA-II",
        "script": here / "nsga2_optimizer.py",
        "profile": None,  # scenario runner currently supports default/pygad/moead
        "best_name": "best_genes_nsga2.json",
        "curve_name": "learning_curve_nsga2.json",
    }


def selected_parser_method(args):
    if args.cap:
        return "cap"
    if args.llm:
        return "llm"
    return "llm"


def _seeded_name(filename: str, seed: int) -> str:
    p = Path(filename)
    return f"{p.stem}_s{seed:02d}{p.suffix}"


def _step_done(path: Path) -> bool:
    return path.exists() and path.is_file()


def _load_json(path: Path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def _mean(vals):
    return sum(vals) / len(vals) if vals else 0.0


def _pct_improvement(before, after):
    if before == 0:
        return 0.0
    return (before - after) / before * 100.0


def _aggregate_doc(doc):
    tests = doc.get("tests", {})
    dist_vals = []
    dist_worst = []
    durations = []
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
        "success_rate": (success_total / attempts_total) if attempts_total > 0 else 0.0,
    }


def _write_offline_compare(scenario_outdir: Path, profile: str):
    req = [
        scenario_outdir / "cap_default.json",
        scenario_outdir / "llm_default.json",
        scenario_outdir / f"cap_{profile}.json",
        scenario_outdir / f"llm_{profile}.json",
    ]
    missing = [str(p) for p in req if not p.exists()]
    if missing:
        raise SystemExit(
            "[SQUAD] Missing files for --compare (run cap/llm first):\n - " + "\n - ".join(missing)
        )

    cap_default = _aggregate_doc(_load_json(req[0]))
    llm_default = _aggregate_doc(_load_json(req[1]))
    cap_after = _aggregate_doc(_load_json(req[2]))
    llm_after = _aggregate_doc(_load_json(req[3]))

    summary = {
        "meta": {
            "mode": "offline_compare",
            "profile": profile,
            "input_dir": str(scenario_outdir),
            "note": "Comparison built only from existing JSON outputs (no webhook call).",
        },
        "aggregates": {
            "cap": {"default": cap_default, profile: cap_after},
            "llm": {"default": llm_default, profile: llm_after},
        },
        "improvements_vs_default": {
            "cap": {
                profile: {
                    "distance_error_avg_linear_pct": _pct_improvement(
                        cap_default["distance_error_avg_linear"], cap_after["distance_error_avg_linear"]
                    ),
                    "distance_error_worst_linear_pct": _pct_improvement(
                        cap_default["distance_error_worst_linear"], cap_after["distance_error_worst_linear"]
                    ),
                    "duration_avg_all_pct": _pct_improvement(
                        cap_default["duration_avg_all"], cap_after["duration_avg_all"]
                    ),
                }
            },
            "llm": {
                profile: {
                    "distance_error_avg_linear_pct": _pct_improvement(
                        llm_default["distance_error_avg_linear"], llm_after["distance_error_avg_linear"]
                    ),
                    "distance_error_worst_linear_pct": _pct_improvement(
                        llm_default["distance_error_worst_linear"], llm_after["distance_error_worst_linear"]
                    ),
                    "duration_avg_all_pct": _pct_improvement(
                        llm_default["duration_avg_all"], llm_after["duration_avg_all"]
                    ),
                }
            },
        },
        "method_comparison": {
            "default": {
                "cap_vs_llm_distance_avg_linear_pct": _pct_improvement(
                    llm_default["distance_error_avg_linear"], cap_default["distance_error_avg_linear"]
                ),
                "cap_vs_llm_worst_linear_pct": _pct_improvement(
                    llm_default["distance_error_worst_linear"], cap_default["distance_error_worst_linear"]
                ),
                "cap_vs_llm_duration_avg_pct": _pct_improvement(
                    llm_default["duration_avg_all"], cap_default["duration_avg_all"]
                ),
            },
            profile: {
                "cap_vs_llm_distance_avg_linear_pct": _pct_improvement(
                    llm_after["distance_error_avg_linear"], cap_after["distance_error_avg_linear"]
                ),
                "cap_vs_llm_worst_linear_pct": _pct_improvement(
                    llm_after["distance_error_worst_linear"], cap_after["distance_error_worst_linear"]
                ),
                "cap_vs_llm_duration_avg_pct": _pct_improvement(
                    llm_after["duration_avg_all"], cap_after["duration_avg_all"]
                ),
            },
        },
    }

    out = scenario_outdir / "cap_vs_llm_summary.json"
    with open(out, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)
    return out


def _extract_tail_metrics(doc: dict) -> dict:
    agg = _aggregate_doc(doc)
    min_front_vals = []
    for t in doc.get("tests", {}).values():
        for d in t.get("details", []):
            v = d.get("min_front")
            if v is not None:
                min_front_vals.append(float(v))
    return {
        "distance_avg_linear_m": agg["distance_error_avg_linear"],
        "duration_avg_s": agg["duration_avg_all"],
        "collisions_total": agg["collisions_total"],
        "resets_total": agg["resets_total"],
        "min_front_avg_m": _mean(min_front_vals),
    }


def _write_tail_ablation_report(on_json: Path, off_json: Path, out_json: Path, out_md: Path):
    on_doc = _load_json(on_json)
    off_doc = _load_json(off_json)
    on_m = _extract_tail_metrics(on_doc)
    off_m = _extract_tail_metrics(off_doc)

    def pct_impr(metric: str):
        # lower-is-better: distance, duration, collisions, resets
        if metric in ("distance_avg_linear_m", "duration_avg_s", "collisions_total", "resets_total"):
            return _pct_improvement(off_m[metric], on_m[metric])
        # higher-is-better: min_front_avg_m
        base = off_m[metric]
        if base == 0:
            return 0.0
        return (on_m[metric] - base) / base * 100.0

    summary = {
        "meta": {
            "mode": "tail_ablation",
            "on_file": str(on_json),
            "off_file": str(off_json),
            "rule": "positive improvement means tail_shielding=on is better",
        },
        "off": off_m,
        "on": on_m,
        "improvement_pct_on_vs_off": {k: pct_impr(k) for k in on_m.keys()},
    }
    with open(out_json, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)

    lines = [
        "# Tail-Shielding Ablation",
        "",
        f"- ON file: `{on_json.name}`",
        f"- OFF file: `{off_json.name}`",
        "- Positive improvement => ON better than OFF",
        "",
        "| Metric | OFF | ON | Improvement (ON vs OFF) |",
        "|---|---:|---:|---:|",
    ]
    for k in ("distance_avg_linear_m", "duration_avg_s", "collisions_total", "resets_total", "min_front_avg_m"):
        lines.append(f"| {k} | {off_m[k]:.6f} | {on_m[k]:.6f} | {summary['improvement_pct_on_vs_off'][k]:+.2f}% |")
    with open(out_md, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")

    # Optional visual artifacts (PNG table + grouped bar plot)
    try:
        import matplotlib.pyplot as plt

        metrics = ["distance_avg_linear_m", "duration_avg_s", "collisions_total", "resets_total", "min_front_avg_m"]
        labels = [
            "Distance avg (m)\n(lower better)",
            "Duration avg (s)\n(lower better)",
            "Collisions total\n(lower better)",
            "Resets total\n(lower better)",
            "Min front avg (m)\n(higher better)",
        ]

        # 1) Table PNG
        table_png = out_json.with_suffix("").with_name(out_json.stem + "_table.png")
        fig, ax = plt.subplots(figsize=(11.5, 3.8))
        ax.axis("off")
        header = ["Metric", "OFF", "ON", "Improvement ON vs OFF"]
        rows = [
            [m, f"{off_m[m]:.6f}", f"{on_m[m]:.6f}", f"{summary['improvement_pct_on_vs_off'][m]:+.2f}%"]
            for m in metrics
        ]
        tbl = ax.table(cellText=rows, colLabels=header, loc="center")
        tbl.auto_set_font_size(False)
        tbl.set_fontsize(9)
        tbl.scale(1.0, 1.35)
        for c in range(len(header)):
            cell = tbl[0, c]
            cell.set_facecolor("#d9edf7")
            cell.set_text_props(weight="bold")
        plt.title(
            "Tail-Shielding Ablation (ON vs OFF)\nRule: positive improvement means ON is better",
            fontsize=11,
            weight="bold",
            pad=10,
        )
        plt.tight_layout()
        plt.savefig(table_png, dpi=220)
        plt.close(fig)

        # 2) Grouped bars (raw OFF/ON values)
        bars_png = out_json.with_suffix("").with_name(out_json.stem + "_metrics.png")
        x = list(range(len(metrics)))
        off_vals = [off_m[m] for m in metrics]
        on_vals = [on_m[m] for m in metrics]
        width = 0.38
        fig, ax = plt.subplots(figsize=(12.5, 4.5))
        ax.bar([i - width / 2 for i in x], off_vals, width, label="OFF", color="#d9534f")
        ax.bar([i + width / 2 for i in x], on_vals, width, label="ON", color="#5cb85c")
        ax.set_xticks(x)
        ax.set_xticklabels(labels, rotation=0)
        ax.set_title("Tail-Shielding Ablation: OFF vs ON (raw metrics)")
        ax.grid(axis="y", alpha=0.25)
        ax.legend()
        plt.tight_layout()
        plt.savefig(bars_png, dpi=220)
        plt.close(fig)

        summary["artifacts"] = {
            "table_png": str(table_png),
            "metrics_png": str(bars_png),
        }
        with open(out_json, "w", encoding="utf-8") as f:
            json.dump(summary, f, indent=2, ensure_ascii=False)

        with open(out_md, "a", encoding="utf-8") as f:
            f.write("\n## Figures\n")
            f.write(f"\n![Tail ablation table]({table_png.name})\n")
            f.write(f"\n![Tail ablation metrics]({bars_png.name})\n")
    except Exception as e:
        print(f"[SQUAD] WARN: failed to render tail-ablation PNG artifacts: {e}")

    return out_json, out_md


def main():
    here = Path(__file__).resolve().parent

    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--moead", action="store_true")
    g.add_argument("--pygad", action="store_true")
    g.add_argument("--nsga2", action="store_true")

    m = ap.add_mutually_exclusive_group(required=False)
    m.add_argument("--cap", action="store_true", help="Parser mode baseline (CaP)")
    m.add_argument("--llm", action="store_true", help="Parser mode current (LLM)")

    ap.add_argument("--compare", action="store_true", help="Run only cap vs llm comparison for selected optimizer profile")
    ap.add_argument("--tail-ablation", action="store_true", help="Run Tail-Shielding ON/OFF ablation using best genes")

    ap.add_argument("--gens", type=int, default=8)
    ap.add_argument("--pop", type=int, default=10)
    ap.add_argument("--repeats", type=int, default=5)
    ap.add_argument("--seeds", type=int, default=1, help="Number of independent seed runs (paper default: 3)")
    ap.add_argument("--seed-base", type=int, default=42, help="Base seed. Effective seeds: seed-base + i")
    ap.add_argument("--validator-timeout", type=float, default=1800.0)
    ap.add_argument("--resume", action="store_true", help="Resume optimizer from cached generation outputs when supported")
    ap.add_argument("--controller-ip", default="127.0.0.1")
    ap.add_argument("--controller-port", type=int, default=20001)
    ap.add_argument("--env-label", default="webots_60x60_obstacles")

    ap.add_argument("--current-webhook", default="", help=f"LLM parser webhook URL (default: {DEFAULT_LLM_WEBHOOK})")
    ap.add_argument(
        "--current-map",
        default="",
        help="Optional fallback map text->JSON for llm mode in scenario_runner",
    )
    ap.add_argument("--tail-parser", choices=["cap", "llm"], default="llm", help="Parser mode for tail ablation labels")
    ap.add_argument("--tail", choices=["keep", "on", "off"], default="keep", help="Tail-shielding mode for normal validator runs")
    args = ap.parse_args()

    validator = here / "p3at_validator.py"
    comparator = here / "compare_validation.py"
    scenario = here / "scenario_runner.py"
    cap_llm_report = here / "cap_llm_report.py"
    tail_ablation_report = here / "tail_ablation_report.py"
    info = optimizer_info(args, here)

    if not validator.exists():
        raise SystemExit(f"[SQUAD] Missing validator: {validator}")
    if not comparator.exists():
        raise SystemExit(f"[SQUAD] Missing comparator: {comparator}")
    if not scenario.exists():
        raise SystemExit(f"[SQUAD] Missing scenario runner: {scenario}")
    if not cap_llm_report.exists():
        raise SystemExit(f"[SQUAD] Missing cap/llm report script: {cap_llm_report}")
    if not tail_ablation_report.exists():
        raise SystemExit(f"[SQUAD] Missing tail ablation report script: {tail_ablation_report}")
    if not info["script"].exists():
        raise SystemExit(f"[SQUAD] Missing optimizer: {info['script']}")

    scenario_outdir = here / "scenario_results" / info["key"]
    scenario_outdir.mkdir(parents=True, exist_ok=True)

    llm_webhook = str(args.current_webhook).strip() or DEFAULT_LLM_WEBHOOK

    # ------------------------------------
    # Compare mode: cap vs llm only (offline from existing JSON)
    # ------------------------------------
    if args.compare:
        if info["profile"] is None:
            raise SystemExit("[SQUAD] --compare is supported only for --pygad or --moead")
        compared_any = False
        for i in range(int(args.seeds)):
            seed = int(args.seed_base) + i
            sd = scenario_outdir / f"seed_{seed:02d}"
            if not sd.exists():
                continue
            out = _write_offline_compare(sd, info["profile"])
            run_cmd(
                [
                    sys.executable,
                    str(cap_llm_report),
                    "--input-dir",
                    str(sd),
                    "--profile",
                    str(info["profile"]),
                    "--out-prefix",
                    "cap_vs_llm",
                ],
                cwd=str(here),
                timeout=None,
            )
            print(f"[SQUAD] Seed {seed:02d} summary: {out}")
            compared_any = True

        # backward compatibility: if no seed dirs found, try root dir once
        if not compared_any:
            out = _write_offline_compare(scenario_outdir, info["profile"])
            run_cmd(
                [
                    sys.executable,
                    str(cap_llm_report),
                    "--input-dir",
                    str(scenario_outdir),
                    "--profile",
                    str(info["profile"]),
                    "--out-prefix",
                    "cap_vs_llm",
                ],
                cwd=str(here),
                timeout=None,
            )
            print(f"[SQUAD] Root summary: {out}")
            compared_any = True

        if not compared_any:
            raise SystemExit("[SQUAD] No scenario files found for compare mode.")
        print("\n[SQUAD] DONE (compare mode)")
        print(f"[SQUAD] Optimizer profile: {info['profile']}")
        print(f"[SQUAD] Output dir: {scenario_outdir}")
        return

    # ------------------------------------
    # Tail-shielding ablation mode
    # ------------------------------------
    if args.tail_ablation:
        parser_method = str(args.tail_parser)
        for i in range(int(args.seeds)):
            seed = int(args.seed_base) + i
            seed_tag = f"s{seed:02d}"
            seed_dir = scenario_outdir / "tail_ablation" / f"seed_{seed:02d}"
            seed_dir.mkdir(parents=True, exist_ok=True)

            genes_name = _seeded_name(str(info["best_name"]), seed)
            genes_candidates = [
                scenario_outdir / f"gen_{parser_method}" / f"seed_{seed:02d}" / genes_name,
                scenario_outdir / f"gen_{parser_method}" / genes_name,
                scenario_outdir / f"gen_{parser_method}" / str(info["best_name"]),
            ]
            genes_file = next((p for p in genes_candidates if p.exists()), None)
            if genes_file is None:
                raise SystemExit(
                    "[SQUAD] Missing genes file for tail ablation. Tried:\n - "
                    + "\n - ".join(str(p) for p in genes_candidates)
                )

            label_on = f"{parser_method}_{info['key']}_tail_on_{seed_tag}"
            label_off = f"{parser_method}_{info['key']}_tail_off_{seed_tag}"

            run_cmd(
                [
                    sys.executable,
                    str(validator),
                    "--label",
                    label_on,
                    "--repeats",
                    str(args.repeats),
                    "--env",
                    str(args.env_label),
                    "--validation-mode",
                    "0",
                    "--tail-shielding",
                    "on",
                    "--genes-file",
                    str(genes_file),
                ],
                cwd=str(seed_dir),
                timeout=args.validator_timeout,
            )
            run_cmd(
                [
                    sys.executable,
                    str(validator),
                    "--label",
                    label_off,
                    "--repeats",
                    str(args.repeats),
                    "--env",
                    str(args.env_label),
                    "--validation-mode",
                    "0",
                    "--tail-shielding",
                    "off",
                    "--genes-file",
                    str(genes_file),
                ],
                cwd=str(seed_dir),
                timeout=args.validator_timeout,
            )

            on_json = seed_dir / f"validation_{label_on}.json"
            off_json = seed_dir / f"validation_{label_off}.json"
            out_json = seed_dir / f"tail_ablation_{parser_method}_{info['key']}_{seed_tag}.json"
            out_md = seed_dir / f"tail_ablation_{parser_method}_{info['key']}_{seed_tag}.md"
            _write_tail_ablation_report(on_json, off_json, out_json, out_md)
            print(f"[SQUAD] Tail ablation report: {out_json}")
            print(f"[SQUAD] Tail ablation table:  {out_md}")

            # Auto-build consolidated CAP vs LLM ON/OFF figure when both parser runs exist.
            cap_done = (seed_dir / f"tail_ablation_cap_{info['key']}_{seed_tag}.json").exists()
            llm_done = (seed_dir / f"tail_ablation_llm_{info['key']}_{seed_tag}.json").exists()
            if cap_done and llm_done:
                run_cmd(
                    [
                        sys.executable,
                        str(tail_ablation_report),
                        "--method",
                        str(info["key"]),
                        "--seed",
                        str(seed),
                        "--base",
                        str(scenario_outdir.parent),
                    ],
                    cwd=str(here),
                    timeout=None,
                )
            else:
                print(
                    "[SQUAD] Tail consolidated figure pending: run tail-ablation for both "
                    f"parsers (cap and llm). Current status: cap={cap_done} llm={llm_done}"
                )
        print("\n[SQUAD] DONE (tail ablation mode)")
        return

    # ------------------------------------
    # Normal mode: classic + parser-aware (for each seed)
    # ------------------------------------
    parser_method = selected_parser_method(args)
    for i in range(int(args.seeds)):
        seed = int(args.seed_base) + i
        seed_tag = f"s{seed:02d}"
        gen_root = scenario_outdir / f"gen_{parser_method}"
        gen_root.mkdir(parents=True, exist_ok=True)
        gen_dir = gen_root / f"seed_{seed:02d}"
        gen_dir.mkdir(parents=True, exist_ok=True)
        seed_scenario_dir = scenario_outdir / f"seed_{seed:02d}"
        seed_scenario_dir.mkdir(parents=True, exist_ok=True)
        print(f"\n[SQUAD] ===== Seed {seed} ({i+1}/{args.seeds}) =====")

        # 1) BEFORE (classic validator)
        before_label = f"{parser_method}_before_{info['key']}_{seed_tag}"
        before_json = gen_dir / f"validation_{before_label}.json"
        if args.resume and _step_done(before_json):
            print(f"[SQUAD] SKIP before validator (resume): {before_json.name}")
        else:
            run_cmd(
                [
                    sys.executable,
                    str(validator),
                    "--label",
                    before_label,
                    "--repeats",
                    str(args.repeats),
                    "--env",
                    str(args.env_label),
                    "--validation-mode",
                    "0",
                    "--tail-shielding",
                    str(args.tail),
                ],
                cwd=str(gen_dir),
                timeout=args.validator_timeout,
            )

        # 2) OPTIMIZER
        best_name = _seeded_name(str(info["best_name"]), seed)
        curve_name = _seeded_name(str(info["curve_name"]), seed)
        best_path = gen_dir / best_name
        curve_path = gen_dir / curve_name
        opt_cmd = [
            sys.executable,
            str(info["script"]),
            "--validator",
            str(validator),
            "--gens",
            str(args.gens),
            "--pop",
            str(args.pop),
            "--seed",
            str(seed),
            "--repeats",
            str(args.repeats),
            "--validator-timeout",
            str(args.validator_timeout),
            "--controller-ip",
            str(args.controller_ip),
            "--controller-port",
            str(args.controller_port),
            "--out-best",
            best_name,
            "--out-curve",
            curve_name,
        ]
        if args.nsga2:
            opt_cmd += ["--out-pareto", _seeded_name("pareto_front.json", seed)]
        if args.resume and (args.pygad or args.moead):
            opt_cmd += ["--resume"]
        if args.resume and _step_done(best_path) and _step_done(curve_path):
            print(f"[SQUAD] SKIP optimizer (resume): {best_name}")
        else:
            run_cmd(opt_cmd, cwd=str(gen_dir), timeout=None)

        # 3) AFTER (classic validator)
        after_label = f"{parser_method}_after_{info['key']}_{seed_tag}"
        after_json = gen_dir / f"validation_{after_label}.json"
        if args.resume and _step_done(after_json):
            print(f"[SQUAD] SKIP after validator (resume): {after_json.name}")
        else:
            run_cmd(
                [
                    sys.executable,
                    str(validator),
                    "--label",
                    after_label,
                    "--repeats",
                    str(args.repeats),
                    "--env",
                    str(args.env_label),
                    "--validation-mode",
                    "0",
                    "--tail-shielding",
                    str(args.tail),
                ],
                cwd=str(gen_dir),
                timeout=args.validator_timeout,
            )

        # 4) COMPARE (classic before/after)
        compare_md = gen_dir / f"validation_report_{parser_method}_{info['key']}_{seed_tag}.md"
        if args.resume and _step_done(compare_md):
            print(f"[SQUAD] SKIP compare (resume): {compare_md.name}")
        else:
            run_cmd(
                [
                    sys.executable,
                    str(comparator),
                    "--before",
                    f"validation_{before_label}.json",
                    "--after",
                    f"validation_{after_label}.json",
                    "--out",
                    compare_md.name,
                    "--learning-curve",
                    curve_name,
                ],
                cwd=str(gen_dir),
                timeout=None,
            )

        # 5) Parser-aware before/after for selected parser (scenario runner)
        profiles = "default"
        if info["profile"] is not None:
            profiles = f"default,{info['profile']}"
        pygad_genes_arg = ""
        moead_genes_arg = ""
        if info["key"] == "pygad":
            pygad_genes_arg = str(gen_dir / _seeded_name("best_genes_pygad.json", seed))
        elif info["key"] == "moead":
            moead_genes_arg = str(gen_dir / _seeded_name("best_genes_moead.json", seed))
        scenario_done = (
            (seed_scenario_dir / f"{parser_method}_default.json").exists()
            and (
                info["profile"] is None
                or (seed_scenario_dir / f"{parser_method}_{info['profile']}.json").exists()
            )
        )
        if args.resume and scenario_done:
            print(f"[SQUAD] SKIP scenario runner (resume): seed_{seed:02d}/{parser_method}")
        else:
            run_cmd(
                [
                    sys.executable,
                    str(scenario),
                    "--run-mode",
                    "b",
                    "--method-order",
                    parser_method,
                    "--profiles",
                    profiles,
                    "--repeats",
                    str(args.repeats),
                    "--exec-timeout",
                    str(args.validator_timeout),
                    "--outdir",
                    str(seed_scenario_dir),
                    "--current-webhook",
                    llm_webhook,
                    "--current-map",
                    str(args.current_map),
                    "--pygad-genes",
                    pygad_genes_arg,
                    "--moead-genes",
                    moead_genes_arg,
                ],
                cwd=str(here),
                timeout=None,
            )

    print("\n[SQUAD] DONE")
    print(f"[SQUAD] Optimizer: {info['name']}")
    print(f"[SQUAD] Parser mode: {parser_method}")
    print(f"[SQUAD] Seeds: {args.seeds} (base={args.seed_base})")
    print(f"[SQUAD] LLM webhook in use: {llm_webhook}")
    print(f"[SQUAD] All outputs: {scenario_outdir}")


if __name__ == "__main__":
    main()

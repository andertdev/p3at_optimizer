#!/usr/bin/env python3
# --------------------------------------------------------------------------------------
# FILE: pygad_optimizer.py
# ROLE:
#   - Mesma "interface" do moead_ireced_optimizer.py:
#       * Envia genes via UDP: {"command":"set_genes","value":{...}}  -> 127.0.0.1:20001
#       * Avalia rodando o validator (subprocess) e lendo validation_<label>.json
#       * Salva best_genes.json + learning_curve.json
#
# USO:
#   python3 pygad_optimizer.py --validator ./p3at_validator.py --gens 8 --pop 8
#
# DEP:
#   pip install pygad
# --------------------------------------------------------------------------------------

import argparse
import json
import os
import random
import socket
import subprocess
import sys
import time
import re
from typing import Dict, Any, Optional, List, Tuple

try:
    import pygad
except ImportError as e:
    raise SystemExit(
        "PyGAD não encontrado. Instale com: pip install pygad\n"
        f"Detalhe: {e}"
    )

# -----------------------------
# UDP (controller)
# -----------------------------
CTRL_IP = "127.0.0.1"
CTRL_CMD_PORT = 20001

# -----------------------------
# Espaço de genes (6 genes completos, como no MOEAD)
# -----------------------------
GENE_SPACE = {
    "base_speed": (0.12, 0.50),
    "turn_speed": (0.30, 1.80),
    "avoid_gain": (0.30, 3.00),
    "dist_stop": (0.70, 1.00),
    "dist_avoid": (1.20, 2.00),
    "turn_force": (1.00, 2.00),
}

GENE_QUANT = {
    "base_speed": 0.01,
    "turn_speed": 0.02,
    "avoid_gain": 0.05,
    "dist_stop": 0.02,
    "dist_avoid": 0.02,
    "turn_force": 0.05,
}

# -----------------------------
# Fitness weights (igual ao moead_ireced_optimizer.py)
# -----------------------------
W_DIST_ERR = 380.0
W_ANG_ERR  = 10.0
W_TIME     = 1.5

W_DIST_STD = 160.0
W_ANG_STD  = 8.0
W_TIME_STD = 2.0

P_COLLISION = 260.0
P_RESET     = 150.0
P_FAIL      = 400.0
P_STOPPED   = 120.0
P_AVOID     = 60.0
P_MIN_FRONT = 140.0
P_MIN_SIDE  = 110.0
P_SIDE_ASYM = 40.0

BONUS_STABLE = 60.0


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def quantize(x: float, step: float) -> float:
    if step <= 0:
        return x
    return round(x / step) * step


def json_dumps(obj: Any) -> str:
    return json.dumps(obj, ensure_ascii=False, separators=(",", ":"))


# -----------------------------
# UDP sender -> controller
# -----------------------------
class ControllerLink:
    def __init__(self, ip: str, port: int):
        self.ip = ip
        self.port = port
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.tx.setblocking(True)

    def send(self, payload: Dict[str, Any]) -> None:
        data = json_dumps(payload).encode("utf-8")
        self.tx.sendto(data, (self.ip, self.port))

    def set_genes(self, genes: Dict[str, float]) -> None:
        self.send({"command": "set_genes", "value": genes})


# -----------------------------
# Fitness from validator JSON
# -----------------------------
def compute_fitness_from_validation(doc: Dict[str, Any]) -> float:
    tests = doc.get("tests", {})

    dist_err = []
    ang_err = []
    time_vals = []
    dist_std = []
    ang_std = []
    time_std = []
    stopped_frac = []
    avoid_frac = []
    min_front_vals = []
    min_left_vals = []
    min_right_vals = []

    total_collisions = 0
    total_resets = 0
    total_attempts = 0
    total_success = 0

    for t in tests.values():
        attempts = int(t.get("attempts", 0))
        success = int(t.get("success", 0))

        total_attempts += attempts
        total_success += success

        total_collisions += int(t.get("collisions", 0))
        total_resets += int(t.get("resets", 0))

        if t.get("distance_error_avg") is not None:
            dist_err.append(float(t["distance_error_avg"]))
        if t.get("distance_error_std") is not None:
            dist_std.append(float(t["distance_error_std"]))

        if t.get("angle_error_avg") is not None:
            ang_err.append(float(t["angle_error_avg"]))
        if t.get("angle_error_std") is not None:
            ang_std.append(float(t["angle_error_std"]))

        if t.get("duration_avg") is not None:
            time_vals.append(float(t["duration_avg"]))
        if t.get("duration_std") is not None:
            time_std.append(float(t["duration_std"]))

        if t.get("stopped_time_frac_avg") is not None:
            stopped_frac.append(float(t["stopped_time_frac_avg"]))
        if t.get("avoid_time_frac_avg") is not None:
            avoid_frac.append(float(t["avoid_time_frac_avg"]))
        if t.get("min_front_avg") is not None:
            min_front_vals.append(float(t["min_front_avg"]))
        if t.get("min_left_avg") is not None:
            min_left_vals.append(float(t["min_left_avg"]))
        if t.get("min_right_avg") is not None:
            min_right_vals.append(float(t["min_right_avg"]))

    if total_attempts == 0:
        return -1e6

    dist_avg = sum(dist_err) / max(1, len(dist_err))
    ang_avg = sum(ang_err) / max(1, len(ang_err))
    time_avg = sum(time_vals) / max(1, len(time_vals))

    dist_std_avg = sum(dist_std) / max(1, len(dist_std))
    ang_std_avg = sum(ang_std) / max(1, len(ang_std))
    time_std_avg = sum(time_std) / max(1, len(time_std))

    stopped_avg = sum(stopped_frac) / max(1, len(stopped_frac))
    avoid_avg = sum(avoid_frac) / max(1, len(avoid_frac))
    min_front_avg = sum(min_front_vals) / max(1, len(min_front_vals))
    min_left_avg = sum(min_left_vals) / max(1, len(min_left_vals))
    min_right_avg = sum(min_right_vals) / max(1, len(min_right_vals))

    success_rate = total_success / total_attempts

    fitness = 0.0
    fitness -= W_DIST_ERR * dist_avg
    fitness -= W_ANG_ERR * ang_avg
    fitness -= W_DIST_STD * dist_std_avg
    fitness -= W_ANG_STD * ang_std_avg

    fitness -= W_TIME * time_avg
    fitness -= W_TIME_STD * time_std_avg

    fitness -= P_COLLISION * total_collisions
    fitness -= P_RESET * total_resets
    fitness -= P_FAIL * (total_attempts - total_success)
    fitness -= P_STOPPED * stopped_avg
    fitness -= P_AVOID * avoid_avg

    if min_front_avg > 0:
        fitness -= P_MIN_FRONT / max(min_front_avg, 0.05)
    if min_left_avg > 0:
        fitness -= P_MIN_SIDE / max(min_left_avg, 0.05)
    if min_right_avg > 0:
        fitness -= P_MIN_SIDE / max(min_right_avg, 0.05)
    if min_left_avg > 0 and min_right_avg > 0:
        # Penaliza trajetória "grudada" em um único lado por muito tempo.
        fitness -= P_SIDE_ASYM * abs(min_left_avg - min_right_avg)

    fitness += 300.0 * success_rate

    if total_collisions == 0 and total_resets == 0:
        fitness += BONUS_STABLE

    return fitness


# -----------------------------
# Run validator
# -----------------------------
def run_validator(validator_path: str, label: str, repeats: int, timeout_s: float) -> Tuple[bool, str]:
    out_file = f"validation_{label}.json"
    cmd = [sys.executable, validator_path, "--label", label, "--repeats", str(repeats)]
    print(f"[PYGAD] RUN: {' '.join(cmd)}")

    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout_s)
        if p.stdout:
            print(p.stdout.strip())
        if p.stderr:
            print(p.stderr.strip())

        if p.returncode != 0:
            print(f"[PYGAD] Validator retornou code={p.returncode}")
            return False, out_file

        if not os.path.exists(out_file):
            print(f"[PYGAD] Validator não gerou o arquivo esperado: {out_file}")
            return False, out_file

        return True, out_file

    except subprocess.TimeoutExpired:
        print(f"[PYGAD] TIMEOUT validator após {timeout_s}s")
        return False, out_file
    except Exception as e:
        print(f"[PYGAD] Erro executando validator: {e}")
        return False, out_file


# -----------------------------
# Genes vector <-> dict
# -----------------------------
GENE_KEYS = list(GENE_SPACE.keys())

def vec_to_genes(vec: List[float]) -> Dict[str, float]:
    g = {}
    for i, k in enumerate(GENE_KEYS):
        lo, hi = GENE_SPACE[k]
        step = GENE_QUANT.get(k, 0.0)
        x = float(vec[i])
        x = clamp(x, lo, hi)
        x = quantize(x, step)
        g[k] = x

    # guardrail: dist_avoid >= dist_stop + 0.05
    if g["dist_avoid"] < g["dist_stop"]:
        g["dist_avoid"] = quantize(g["dist_stop"] + 0.05, GENE_QUANT.get("dist_avoid", 0.01))
        g["dist_avoid"] = clamp(g["dist_avoid"], *GENE_SPACE["dist_avoid"])
    return g


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--validator", required=True, help="Caminho para p3at_validator.py (controller-centric)")
    ap.add_argument("--gens", type=int, default=8)
    ap.add_argument("--pop", type=int, default=10)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--repeats", type=int, default=5)
    ap.add_argument("--validator-timeout", type=float, default=1800.0)
    ap.add_argument("--out-best", default="best_genes.json")
    ap.add_argument("--out-curve", default="learning_curve.json")
    ap.add_argument("--controller-ip", default=CTRL_IP)
    ap.add_argument("--controller-port", type=int, default=CTRL_CMD_PORT)
    ap.add_argument("--resume", action="store_true", help="Resume from cached validation_gXXX_iYY.json files")
    args = ap.parse_args()

    random.seed(args.seed)

    link = ControllerLink(args.controller_ip, args.controller_port)

    gens = int(args.gens)
    pop_size = int(args.pop)

    print(f"[PYGAD] Controller: {args.controller_ip}:{args.controller_port}")
    print(f"[PYGAD] Validator: {args.validator}")
    print(f"[PYGAD] Gens={gens} Pop={pop_size} Repeats={args.repeats}")

    learning_curve: List[Dict[str, Any]] = []
    best_global = {"fitness": -1e18, "genes": None, "validation_file": None}
    cached_fit: Dict[Tuple[int, int], Tuple[float, str]] = {}
    resume_gen_limit = 0

    if args.resume:
        pat = re.compile(r"^validation_(?:s(?P<seed>\d{2})_)?g(?P<gen>\d{3})_i(?P<idx>\d{2})\.json$")
        files = [f for f in os.listdir(".") if pat.match(f)]
        by_gen: Dict[int, set] = {}
        for fn in files:
            m = pat.match(fn)
            if not m:
                continue
            file_seed = m.group("seed")
            if file_seed is not None and int(file_seed) != int(args.seed):
                continue
            g = int(m.group("gen"))
            i = int(m.group("idx"))
            by_gen.setdefault(g, set()).add(i)

        # Última geração COMPLETA em sequência (1..k)
        g = 1
        while len(by_gen.get(g, set())) >= pop_size:
            g += 1
        resume_gen_limit = g - 1

        # Reaproveita também geração parcial atual, desde que o cache exista no diretório da seed.
        for gen, idxs in by_gen.items():
            for idx in idxs:
                candidates = [
                    f"validation_s{int(args.seed):02d}_g{gen:03d}_i{idx:02d}.json",
                    f"validation_g{gen:03d}_i{idx:02d}.json",
                ]
                fn = next((name for name in candidates if os.path.exists(name)), None)
                if not fn:
                    continue
                try:
                    with open(fn, "r") as f:
                        doc = json.load(f)
                    meta = doc.get("meta", {})
                    if int(meta.get("repeats", args.repeats)) != int(args.repeats):
                        continue
                    fit = compute_fitness_from_validation(doc)
                    cached_fit[(gen, idx)] = (fit, fn)
                except Exception:
                    continue

        partial_next = sorted(list(by_gen.get(resume_gen_limit + 1, set()))) if (resume_gen_limit + 1) in by_gen else []
        print(
            f"[PYGAD][RESUME] cache_files={len(files)} "
            f"last_complete_gen={resume_gen_limit} "
            f"partial_next_gen_indices={partial_next[:10]}{'...' if len(partial_next) > 10 else ''}"
        )
        if resume_gen_limit > 0:
            print(f"[PYGAD][RESUME] retomando do início da geração {resume_gen_limit + 1}")
        elif partial_next:
            print(f"[PYGAD][RESUME] retomando na geração parcial {resume_gen_limit + 1}")

    # Para labels por indivíduo
    # PyGAD chama fitness com solution_idx na população atual.
    def fitness_func(ga_instance, solution, solution_idx):
        # geração atual durante avaliação (0-based completed -> +1)
        gen = ga_instance.generations_completed + 1
        label = f"s{int(args.seed):02d}_g{gen:03d}_i{int(solution_idx):02d}"

        genes = vec_to_genes(solution)

        # Resume: usa resultado já validado para gerações completas anteriores.
        if args.resume and (gen, int(solution_idx)) in cached_fit:
            fit, out_file = cached_fit[(gen, int(solution_idx))]
            if fit > best_global["fitness"]:
                best_global["fitness"] = fit
                best_global["genes"] = dict(genes)
                best_global["validation_file"] = out_file
            print(f"[PYGAD][RESUME] gen={gen} idx={solution_idx:02d} fit={fit:.2f} (cache: {out_file})")
            return fit

        # 1) aplica genes
        try:
            link.set_genes(genes)
        except Exception as e:
            print(f"[PYGAD] Falha enviando set_genes: {e}")
            return -1e18

        # 2) roda validator
        ok, out_file = run_validator(
            validator_path=args.validator,
            label=label,
            repeats=args.repeats,
            timeout_s=args.validator_timeout
        )
        if not ok:
            return -1e18

        # 3) fitness
        try:
            with open(out_file, "r") as f:
                doc = json.load(f)
            fit = compute_fitness_from_validation(doc)
        except Exception as e:
            print(f"[PYGAD] Erro lendo/avaliando {out_file}: {e}")
            fit = -1e18

        # Atualiza best global “on the fly”
        if fit > best_global["fitness"]:
            best_global["fitness"] = fit
            best_global["genes"] = dict(genes)
            best_global["validation_file"] = out_file
            print(f"[PYGAD] ★ Novo BEST: fit={fit:.2f} genes={genes} file={out_file}")

        print(f"[PYGAD] gen={gen} idx={solution_idx:02d} fit={fit:.2f} genes={genes}")
        return fit

    def on_generation(ga_instance):
        # snapshot por geração (best conhecido até aqui)
        learning_curve.append({
            "generation": ga_instance.generations_completed,
            "best_fitness": best_global["fitness"],
            "best_genes": best_global["genes"],
            "best_validation_file": best_global["validation_file"],
        })

    # gene_space por dimensão (PyGAD permite lista de ranges)
    gene_space = []
    for k in GENE_KEYS:
        lo, hi = GENE_SPACE[k]
        gene_space.append({"low": lo, "high": hi})

    ga_instance = pygad.GA(
        num_generations=gens,
        sol_per_pop=pop_size,
        num_parents_mating=max(2, pop_size // 2),
        num_genes=len(GENE_KEYS),
        gene_space=gene_space,
        fitness_func=fitness_func,
        on_generation=on_generation,
        parent_selection_type="tournament",
        K_tournament=3,
        crossover_type="single_point",
        mutation_type="random",
        mutation_percent_genes=35,
        random_seed=args.seed,
        keep_elitism=max(1, pop_size // 4),
        allow_duplicate_genes=True,
    )

    print("[PYGAD] 🚀 Iniciando otimização com PyGAD...")
    ga_instance.run()

    # Se on_generation não rodou (gens=0), garante arquivo
    if not learning_curve:
        learning_curve.append({
            "generation": 0,
            "best_fitness": best_global["fitness"],
            "best_genes": best_global["genes"],
            "best_validation_file": best_global["validation_file"],
        })

    # salva
    if best_global["genes"] is None:
        best_global["genes"] = vec_to_genes([random.uniform(*GENE_SPACE[k]) for k in GENE_KEYS])

    with open(args.out_best, "w") as f:
        json.dump(best_global["genes"], f, indent=2)

    with open(args.out_curve, "w") as f:
        json.dump(learning_curve, f, indent=2)

    print(f"\n[PYGAD] ✅ Fim. Best fitness={best_global['fitness']:.2f}")
    print(f"[PYGAD] ✅ {args.out_best}: {os.path.abspath(args.out_best)}")
    print(f"[PYGAD] ✅ {args.out_curve}: {os.path.abspath(args.out_curve)}")


if __name__ == "__main__":
    main()

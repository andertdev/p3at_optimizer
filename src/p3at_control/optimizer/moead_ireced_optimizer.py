#!/usr/bin/env python3
# --------------------------------------------------------------------------------------
# FILE: moead_ireced_optimizer.py  (FINAL - Validator does all measurements)
# ROLE:
#   - Evolui genes do controller Webots (fonte única de verdade) via UDP set_genes
#   - Avalia fitness chamando o validator (subprocess) e lendo o JSON gerado
#   - Salva best_genes.json + learning_curve.json
#
# REQUISITOS:
#   1) Controller Webots escutando UDP em 127.0.0.1:20001 com comando:
#        {"command":"set_genes","value":{...}}
#   2) Validator (novo) disponível como script executável python:
#        p3at_validator.py
#      e que gere: validation_<label>.json no formato:
#        {
#          "meta": {...},
#          "tests": {
#             "linear_1m": {"attempts":..,"success":..,"collisions":..,"resets":..,
#                           "distance_error_avg":..,"duration_avg":..},
#             "turn_90deg": {"angle_error_avg":..,"duration_avg":.., ...},
#             ...
#          }
#        }
#
# USO:
#   python3 moead_ireced_optimizer.py --validator ./p3at_validator.py
# --------------------------------------------------------------------------------------

import argparse
import json
import math
import os
import random
import re
import socket
import subprocess
import sys
import time
from dataclasses import dataclass
from typing import Dict, Any, Optional, List, Tuple

# -----------------------------
# UDP (controller)
# -----------------------------
CTRL_IP = "127.0.0.1"
CTRL_CMD_PORT = 20001
UDP_MAX_BYTES = 65507

# -----------------------------
# Espaço de genes (ajuste conforme seu controller suportar)
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
# Fitness weights (robustez > precisão absoluta)
# -----------------------------
W_DIST_ERR = 380.0      # penaliza erro linear médio
W_ANG_ERR  = 10.0       # penaliza erro angular médio (graus)
W_TIME     = 1.5        # penaliza duração média (s)

W_DIST_STD = 160.0      # penaliza oscilação no erro linear
W_ANG_STD  = 8.0        # penaliza oscilação no erro angular
W_TIME_STD = 2.0        # penaliza oscilação no tempo

P_COLLISION = 260.0     # penaliza colisões (robustez)
P_RESET     = 150.0     # penaliza resets
P_FAIL      = 400.0     # penaliza falhas (tentativas - sucessos)
P_STOPPED   = 120.0     # penaliza tempo parado
P_AVOID     = 60.0      # penaliza tempo em desvio
P_MIN_FRONT = 140.0     # penaliza ficar muito perto (usa 1/distância)
P_MIN_SIDE  = 110.0     # penaliza ficar muito perto das laterais
P_SIDE_ASYM = 40.0      # penaliza assimetria lateral persistente

# bônus por estabilidade/robustez (opcional)
BONUS_STABLE = 60.0


# -----------------------------
# Utils
# -----------------------------
def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x

def quantize(x: float, step: float) -> float:
    if step <= 0:
        return x
    return round(x / step) * step

def safe_float(v, default=0.0) -> float:
    try:
        return float(v)
    except Exception:
        return default

def json_dumps(obj: Any) -> str:
    return json.dumps(obj, ensure_ascii=False, separators=(",", ":"))

def now_s() -> float:
    return time.time()


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
# Genes: sample / mutate / crossover
# -----------------------------
def random_genes() -> Dict[str, float]:
    g = {}
    for k, (lo, hi) in GENE_SPACE.items():
        x = random.uniform(lo, hi)
        g[k] = quantize(x, GENE_QUANT.get(k, 0.0))

    # guardrails
    g["dist_stop"] = clamp(g["dist_stop"], *GENE_SPACE["dist_stop"])
    g["dist_avoid"] = clamp(g["dist_avoid"], *GENE_SPACE["dist_avoid"])
    if g["dist_avoid"] < g["dist_stop"]:
        g["dist_avoid"] = quantize(g["dist_stop"] + 0.05, GENE_QUANT.get("dist_avoid", 0.01))
        g["dist_avoid"] = clamp(g["dist_avoid"], *GENE_SPACE["dist_avoid"])
    return g

def mutate(parent: Dict[str, float], sigma_frac: float = 0.12, p_mut: float = 0.85) -> Dict[str, float]:
    child = dict(parent)
    for k, (lo, hi) in GENE_SPACE.items():
        if random.random() > p_mut:
            continue
        span = (hi - lo)
        x = child[k] + random.gauss(0.0, sigma_frac * span)
        x = clamp(x, lo, hi)
        child[k] = quantize(x, GENE_QUANT.get(k, 0.0))

    if child["dist_avoid"] < child["dist_stop"]:
        child["dist_avoid"] = quantize(child["dist_stop"] + 0.05, GENE_QUANT.get("dist_avoid", 0.01))
        child["dist_avoid"] = clamp(child["dist_avoid"], *GENE_SPACE["dist_avoid"])
    return child

def crossover(a: Dict[str, float], b: Dict[str, float]) -> Dict[str, float]:
    c = {}
    for k in GENE_SPACE.keys():
        c[k] = a[k] if random.random() < 0.5 else b[k]
    if c["dist_avoid"] < c["dist_stop"]:
        c["dist_avoid"] = quantize(c["dist_stop"] + 0.05, GENE_QUANT.get("dist_avoid", 0.01))
    return c


# -----------------------------
# Fitness from validator JSON
# -----------------------------
def compute_fitness_from_validation(doc):
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

    # -----------------------------
    # Segurança
    # -----------------------------
    if total_attempts == 0:
        return -1e6

    # -----------------------------
    # Médias
    # -----------------------------
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

    # -----------------------------
    # Fitness (MAIOR = MELHOR)
    # -----------------------------
    fitness = 0.0

    # Precisão
    fitness -= W_DIST_ERR * dist_avg
    fitness -= W_ANG_ERR * ang_avg
    fitness -= W_DIST_STD * dist_std_avg
    fitness -= W_ANG_STD * ang_std_avg

    # Tempo (impacta base_speed e avoid_gain)
    fitness -= W_TIME * time_avg
    fitness -= W_TIME_STD * time_std_avg

    # Robustez
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

    # Taxa de sucesso
    fitness += 300.0 * success_rate

    # Bônus de estabilidade
    if total_collisions == 0 and total_resets == 0:
        fitness += BONUS_STABLE

    return fitness



# -----------------------------
# Run validator
# -----------------------------
def run_validator(validator_path: str, label: str, repeats: int, timeout_s: float) -> Tuple[bool, str]:
    """
    Executa o validator e retorna (ok, output_json_filename).
    O validator deve gerar validation_<label>.json.
    """
    out_file = f"validation_{label}.json"
    cmd = [sys.executable, validator_path, "--label", label, "--repeats", str(repeats)]
    print(f"[OPT] RUN: {' '.join(cmd)}")

    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout_s)
        if p.stdout:
            print(p.stdout.strip())
        if p.stderr:
            print(p.stderr.strip())

        if p.returncode != 0:
            print(f"[OPT] Validator retornou code={p.returncode}")
            return False, out_file

        if not os.path.exists(out_file):
            print(f"[OPT] Validator não gerou o arquivo esperado: {out_file}")
            return False, out_file

        return True, out_file

    except subprocess.TimeoutExpired:
        print(f"[OPT] TIMEOUT validator após {timeout_s}s")
        return False, out_file
    except Exception as e:
        print(f"[OPT] Erro executando validator: {e}")
        return False, out_file


# -----------------------------
# Individual
# -----------------------------
@dataclass
class Individual:
    genes: Dict[str, float]
    fitness: float = -1e18
    validation_file: Optional[str] = None


# -----------------------------
# Main optimization loop (EA simples e robusto)
# -----------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--validator", required=True, help="Caminho para p3at_validator.py (novo)")
    ap.add_argument("--gens", type=int, default=8)
    ap.add_argument("--pop", type=int, default=10)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--repeats", type=int, default=5, help="Repetições por teste no validator (mais lento, porém mais estável)")
    ap.add_argument("--validator-timeout", type=float, default=1800.0, help="Timeout total do validator por indivíduo (s)")
    ap.add_argument("--out-best", default="best_genes.json")
    ap.add_argument("--out-curve", default="learning_curve.json")
    ap.add_argument("--controller-ip", default=CTRL_IP)
    ap.add_argument("--controller-port", type=int, default=CTRL_CMD_PORT)
    ap.add_argument("--resume", action="store_true", help="Resume from cached validation_sXX_gXXX_iYY.json files")
    args = ap.parse_args()

    random.seed(args.seed)

    link = ControllerLink(args.controller_ip, args.controller_port)

    gens = int(args.gens)
    pop_size = int(args.pop)

    print(f"[OPT] Controller: {args.controller_ip}:{args.controller_port}")
    print(f"[OPT] Validator: {args.validator}")
    print(f"[OPT] Gens={gens} Pop={pop_size} Repeats={args.repeats}")

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

        g = 1
        while len(by_gen.get(g, set())) >= pop_size:
            g += 1
        resume_gen_limit = g - 1

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
            f"[OPT][RESUME] cache_files={len(files)} "
            f"last_complete_gen={resume_gen_limit} "
            f"partial_next_gen_indices={partial_next[:10]}{'...' if len(partial_next) > 10 else ''}"
        )
        if resume_gen_limit > 0:
            print(f"[OPT][RESUME] retomando do início da geração {resume_gen_limit + 1}")
        elif partial_next:
            print(f"[OPT][RESUME] retomando na geração parcial {resume_gen_limit + 1}")

    # init pop
    pop: List[Individual] = [Individual(genes=random_genes()) for _ in range(pop_size)]
    best = Individual(genes=random_genes(), fitness=-1e18)

    learning_curve = []

    for gen in range(1, gens + 1):
        print(f"\n[OPT] ===== Geração {gen}/{gens} =====")

        for i in range(pop_size):
            ind = pop[i]

            if args.resume and (gen, i) in cached_fit:
                fit, out_file = cached_fit[(gen, i)]
                ind.fitness = fit
                ind.validation_file = out_file
                pop[i] = ind
                print(f"[OPT][RESUME] {i+1:02d}/{pop_size} fit={ind.fitness:.2f} (cache: {out_file})")
                if ind.fitness > best.fitness:
                    best = Individual(genes=dict(ind.genes), fitness=ind.fitness, validation_file=ind.validation_file)
                    print(f"[OPT] ★ Novo BEST: fit={best.fitness:.2f} genes={best.genes} file={best.validation_file}")
                continue

            # 1) aplica genes no controller
            try:
                link.set_genes(ind.genes)
            except Exception as e:
                print(f"[OPT] Falha enviando set_genes: {e}")
                ind.fitness = -1e18
                pop[i] = ind
                continue

            # 2) roda validator (ele mede tudo e gera JSON)
            label = f"s{int(args.seed):02d}_g{gen:03d}_i{i:02d}"
            ok, out_file = run_validator(
                validator_path=args.validator,
                label=label,
                repeats=args.repeats,
                timeout_s=args.validator_timeout
            )
            ind.validation_file = out_file

            if not ok:
                ind.fitness = -1e18
                pop[i] = ind
                print(f"[OPT] {i+1:02d}/{pop_size} fit={ind.fitness:.2f} genes={ind.genes} (validator FAIL)")
                continue

            # 3) lê JSON e calcula fitness
            try:
                with open(out_file, "r") as f:
                    doc = json.load(f)
                ind.fitness = compute_fitness_from_validation(doc)
            except Exception as e:
                print(f"[OPT] Erro lendo/avaliando {out_file}: {e}")
                ind.fitness = -1e18

            pop[i] = ind
            print(f"[OPT] {i+1:02d}/{pop_size} fit={ind.fitness:.2f} genes={ind.genes}")

            if ind.fitness > best.fitness:
                best = Individual(genes=dict(ind.genes), fitness=ind.fitness, validation_file=ind.validation_file)
                print(f"[OPT] ★ Novo BEST: fit={best.fitness:.2f} genes={best.genes} file={best.validation_file}")

        # curva por geração
        learning_curve.append({
            "generation": gen,
            "best_fitness": best.fitness,
            "best_genes": dict(best.genes),
            "best_validation_file": best.validation_file,
        })

        # seleção (elites)
        pop_sorted = sorted(pop, key=lambda x: x.fitness, reverse=True)
        elites = pop_sorted[: max(2, pop_size // 4)]  # top 25%

        # nova pop
        new_pop: List[Individual] = []
        new_pop.append(Individual(genes=dict(elites[0].genes)))
        new_pop.append(Individual(genes=dict(elites[1].genes)))

        while len(new_pop) < pop_size:
            pa = random.choice(elites).genes
            pb = random.choice(elites).genes
            child = crossover(pa, pb)
            sigma = 0.35 if gen <= 2 else 0.15
            child = mutate(child, sigma_frac=sigma, p_mut=0.9)

            new_pop.append(Individual(genes=child))

        pop = new_pop

    # salva best genes e curva
    with open(args.out_best, "w") as f:
        json.dump(best.genes, f, indent=2)

    with open(args.out_curve, "w") as f:
        json.dump(learning_curve, f, indent=2)

    print(f"\n[OPT] ✅ Fim. Best fitness={best.fitness:.2f}")
    print(f"[OPT] ✅ {args.out_best}: {os.path.abspath(args.out_best)}")
    print(f"[OPT] ✅ {args.out_curve}: {os.path.abspath(args.out_curve)}")


if __name__ == "__main__":
    main()

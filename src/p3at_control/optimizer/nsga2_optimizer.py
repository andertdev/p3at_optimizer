#!/usr/bin/env python3
# --------------------------------------------------------------------------------------
# FILE: nsga2_optimizer.py
# ROLE:
#   - Otimizador NSGA-II simples (sem dependências externas)
#   - Envia genes via UDP: {"command":"set_genes","value":{...}}
#   - Avalia via validator (subprocess) e lê validation_<label>.json
#   - Salva best_genes.json + learning_curve.json + pareto_front.json (opcional)
#
# USO:
#   python3 nsga2_optimizer.py --validator ./p3at_validator.py --gens 8 --pop 8
# --------------------------------------------------------------------------------------

import argparse
import json
import os
import random
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
# Espaço de genes (igual ao MOEAD/PyGAD)
# -----------------------------
GENE_SPACE = {
    "base_speed": (0.12, 0.50),
    "turn_speed": (0.30, 1.80),
    "avoid_gain": (0.30, 3.00),
    "dist_stop": (0.55, 1.00),
    "dist_avoid": (1.00, 2.00),
    "turn_force": (0.80, 2.00),
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
# Utils
# -----------------------------
def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def quantize(x: float, step: float) -> float:
    if step <= 0:
        return x
    return round(x / step) * step


def json_dumps(obj: Any) -> str:
    return json.dumps(obj, ensure_ascii=False, separators=(",", ":"))


def safe_float(v, default=0.0) -> float:
    try:
        return float(v)
    except Exception:
        return default


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
        if len(data) > UDP_MAX_BYTES:
            raise ValueError("Payload UDP muito grande")
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
        span = hi - lo
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
# Validator + objectives
# -----------------------------
def run_validator(validator_path: str, label: str, repeats: int, timeout_s: float) -> Tuple[bool, str]:
    out_file = f"validation_{label}.json"
    cmd = [sys.executable, validator_path, "--label", label, "--repeats", str(repeats)]
    print(f"[NSGA2] RUN: {' '.join(cmd)}")

    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout_s)
        if p.stdout:
            print(p.stdout.strip())
        if p.stderr:
            print(p.stderr.strip())

        if p.returncode != 0:
            print(f"[NSGA2] Validator retornou code={p.returncode}")
            return False, out_file

        if not os.path.exists(out_file):
            print(f"[NSGA2] Validator não gerou o arquivo esperado: {out_file}")
            return False, out_file

        return True, out_file

    except subprocess.TimeoutExpired:
        print(f"[NSGA2] TIMEOUT validator após {timeout_s}s")
        return False, out_file
    except Exception as e:
        print(f"[NSGA2] Erro executando validator: {e}")
        return False, out_file


def compute_objectives(doc: Dict[str, Any]) -> Tuple[float, float, float]:
    tests = doc.get("tests", {})

    dist_err = []
    ang_err = []
    time_vals = []
    dist_std = []
    ang_std = []
    time_std = []

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

    if total_attempts == 0:
        return (1e9, 1e9, 1e9)

    dist_avg = sum(dist_err) / max(1, len(dist_err))
    ang_avg = sum(ang_err) / max(1, len(ang_err))
    time_avg = sum(time_vals) / max(1, len(time_vals))

    dist_std_avg = sum(dist_std) / max(1, len(dist_std))
    ang_std_avg = sum(ang_std) / max(1, len(ang_std))
    time_std_avg = sum(time_std) / max(1, len(time_std))

    fail_count = max(0, total_attempts - total_success)

    obj_dist = dist_avg + 0.5 * dist_std_avg
    obj_ang = ang_avg + 0.5 * ang_std_avg
    obj_time = time_avg + 0.2 * time_std_avg + 5.0 * total_collisions + 3.0 * total_resets + 2.0 * fail_count

    return (obj_dist, obj_ang, obj_time)


def compute_scalar_score(objectives: Tuple[float, float, float]) -> float:
    # Menor é melhor. Retornamos score negativo para "melhor fitness".
    o1, o2, o3 = objectives
    return -(200.0 * o1 + 20.0 * o2 + 1.0 * o3)


# -----------------------------
# NSGA-II core
# -----------------------------
def dominates(a: Tuple[float, ...], b: Tuple[float, ...]) -> bool:
    return all(x <= y for x, y in zip(a, b)) and any(x < y for x, y in zip(a, b))


def fast_non_dominated_sort(pop: List["Individual"]) -> List[List["Individual"]]:
    fronts: List[List[Individual]] = []
    size = len(pop)
    idx_of = {id(ind): i for i, ind in enumerate(pop)}
    S: List[List[int]] = [[] for _ in range(size)]
    n: List[int] = [0] * size

    for i in range(size):
        p = pop[i]
        for j in range(size):
            if i == j:
                continue
            q = pop[j]
            if dominates(p.objectives, q.objectives):
                S[i].append(j)
            elif dominates(q.objectives, p.objectives):
                n[i] += 1
        if n[i] == 0:
            p.rank = 0
            if not fronts:
                fronts.append([])
            fronts[0].append(p)

    i = 0
    while i < len(fronts):
        next_front: List[Individual] = []
        for p in fronts[i]:
            p_idx = idx_of.get(id(p))
            if p_idx is None:
                continue
            for q_idx in S[p_idx]:
                n[q_idx] -= 1
                if n[q_idx] == 0:
                    q = pop[q_idx]
                    q.rank = i + 1
                    next_front.append(q)
        if next_front:
            fronts.append(next_front)
        i += 1

    return fronts


def crowding_distance(front: List["Individual"]) -> None:
    if not front:
        return
    m = len(front[0].objectives)
    for p in front:
        p.crowding = 0.0

    for i in range(m):
        front.sort(key=lambda x: x.objectives[i])
        front[0].crowding = float("inf")
        front[-1].crowding = float("inf")
        fmin = front[0].objectives[i]
        fmax = front[-1].objectives[i]
        if fmax == fmin:
            continue
        for j in range(1, len(front) - 1):
            prev_f = front[j - 1].objectives[i]
            next_f = front[j + 1].objectives[i]
            front[j].crowding += (next_f - prev_f) / (fmax - fmin)


def tournament_select(pop: List["Individual"]) -> "Individual":
    a, b = random.sample(pop, 2)
    if a.rank < b.rank:
        return a
    if b.rank < a.rank:
        return b
    return a if a.crowding >= b.crowding else b


@dataclass
class Individual:
    genes: Dict[str, float]
    objectives: Tuple[float, float, float] = (1e9, 1e9, 1e9)
    fitness: float = -1e18
    validation_file: Optional[str] = None
    rank: int = 0
    crowding: float = 0.0


# -----------------------------
# Main optimization loop
# -----------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--validator", required=True, help="Caminho para p3at_validator.py")
    ap.add_argument("--gens", type=int, default=8)
    ap.add_argument("--pop", type=int, default=8)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--repeats", type=int, default=1)
    ap.add_argument("--validator-timeout", type=float, default=1800.0)
    ap.add_argument("--out-best", default="best_genes.json")
    ap.add_argument("--out-curve", default="learning_curve.json")
    ap.add_argument("--out-pareto", default=None)
    ap.add_argument("--controller-ip", default=CTRL_IP)
    ap.add_argument("--controller-port", type=int, default=CTRL_CMD_PORT)
    args = ap.parse_args()

    random.seed(args.seed)

    link = ControllerLink(args.controller_ip, args.controller_port)

    gens = int(args.gens)
    pop_size = int(args.pop)

    print(f"[NSGA2] Controller: {args.controller_ip}:{args.controller_port}")
    print(f"[NSGA2] Validator: {args.validator}")
    print(f"[NSGA2] Gens={gens} Pop={pop_size} Repeats={args.repeats}")

    pop: List[Individual] = [Individual(genes=random_genes()) for _ in range(pop_size)]
    best = Individual(genes=random_genes(), fitness=-1e18)
    learning_curve = []

    for gen in range(1, gens + 1):
        print(f"\n[NSGA2] ===== Geração {gen}/{gens} =====")

        # avaliar população
        for i in range(pop_size):
            ind = pop[i]

            try:
                link.set_genes(ind.genes)
            except Exception as e:
                print(f"[NSGA2] Falha enviando set_genes: {e}")
                ind.objectives = (1e9, 1e9, 1e9)
                ind.fitness = -1e18
                pop[i] = ind
                continue

            label = f"g{gen:03d}_i{i:02d}"
            ok, out_file = run_validator(
                validator_path=args.validator,
                label=label,
                repeats=args.repeats,
                timeout_s=args.validator_timeout
            )
            ind.validation_file = out_file

            if not ok:
                ind.objectives = (1e9, 1e9, 1e9)
                ind.fitness = -1e18
                pop[i] = ind
                print(f"[NSGA2] {i+1:02d}/{pop_size} FAIL genes={ind.genes}")
                continue

            try:
                with open(out_file, "r") as f:
                    doc = json.load(f)
                ind.objectives = compute_objectives(doc)
                ind.fitness = compute_scalar_score(ind.objectives)
            except Exception as e:
                print(f"[NSGA2] Erro lendo/avaliando {out_file}: {e}")
                ind.objectives = (1e9, 1e9, 1e9)
                ind.fitness = -1e18

            pop[i] = ind
            print(f"[NSGA2] {i+1:02d}/{pop_size} obj={ind.objectives} genes={ind.genes}")

            if ind.fitness > best.fitness:
                best = Individual(
                    genes=dict(ind.genes),
                    objectives=ind.objectives,
                    fitness=ind.fitness,
                    validation_file=ind.validation_file
                )
                print(f"[NSGA2] ★ Novo BEST: fit={best.fitness:.2f} obj={best.objectives} genes={best.genes}")

        # ranking + crowding
        fronts = fast_non_dominated_sort(pop)
        for f in fronts:
            crowding_distance(f)

        # gerar offspring
        offspring: List[Individual] = []
        while len(offspring) < pop_size:
            p1 = tournament_select(pop)
            p2 = tournament_select(pop)
            child_genes = mutate(crossover(p1.genes, p2.genes))
            offspring.append(Individual(genes=child_genes))

        # avaliar offspring
        for i in range(pop_size):
            ind = offspring[i]
            try:
                link.set_genes(ind.genes)
            except Exception as e:
                print(f"[NSGA2] Falha enviando set_genes (offspring): {e}")
                ind.objectives = (1e9, 1e9, 1e9)
                ind.fitness = -1e18
                offspring[i] = ind
                continue

            label = f"g{gen:03d}_o{i:02d}"
            ok, out_file = run_validator(
                validator_path=args.validator,
                label=label,
                repeats=args.repeats,
                timeout_s=args.validator_timeout
            )
            ind.validation_file = out_file

            if not ok:
                ind.objectives = (1e9, 1e9, 1e9)
                ind.fitness = -1e18
                offspring[i] = ind
                print(f"[NSGA2] offspring {i+1:02d}/{pop_size} FAIL")
                continue

            try:
                with open(out_file, "r") as f:
                    doc = json.load(f)
                ind.objectives = compute_objectives(doc)
                ind.fitness = compute_scalar_score(ind.objectives)
            except Exception as e:
                print(f"[NSGA2] Erro lendo/avaliando {out_file}: {e}")
                ind.objectives = (1e9, 1e9, 1e9)
                ind.fitness = -1e18

            offspring[i] = ind
            print(f"[NSGA2] offspring {i+1:02d}/{pop_size} obj={ind.objectives}")

            if ind.fitness > best.fitness:
                best = Individual(
                    genes=dict(ind.genes),
                    objectives=ind.objectives,
                    fitness=ind.fitness,
                    validation_file=ind.validation_file
                )
                print(f"[NSGA2] ★ Novo BEST: fit={best.fitness:.2f} obj={best.objectives} genes={best.genes}")

        # combinar e selecionar próxima geração
        combined = pop + offspring
        fronts = fast_non_dominated_sort(combined)
        next_pop: List[Individual] = []
        for f in fronts:
            crowding_distance(f)
            if len(next_pop) + len(f) <= pop_size:
                next_pop.extend(f)
            else:
                f.sort(key=lambda x: x.crowding, reverse=True)
                needed = pop_size - len(next_pop)
                next_pop.extend(f[:needed])
                break

        pop = next_pop

        # curva por geração
        learning_curve.append({
            "generation": gen,
            "best_fitness": best.fitness,
            "best_genes": dict(best.genes),
            "best_validation_file": best.validation_file,
        })

    # salvar best genes
    with open(args.out_best, "w") as f:
        json.dump(best.genes, f, indent=2, ensure_ascii=False)

    # salvar curva
    with open(args.out_curve, "w") as f:
        json.dump(learning_curve, f, indent=2, ensure_ascii=False)

    # salvar pareto
    if args.out_pareto:
        fronts = fast_non_dominated_sort(pop)
        pareto = fronts[0] if fronts else []
        out = []
        for ind in pareto:
            out.append({
                "genes": ind.genes,
                "objectives": ind.objectives,
                "validation_file": ind.validation_file,
            })
        with open(args.out_pareto, "w") as f:
            json.dump(out, f, indent=2, ensure_ascii=False)

    print("\n[NSGA2] ✅ FIM")
    print(f"[NSGA2] Best fitness: {best.fitness:.2f}")
    print(f"[NSGA2] Best genes: {best.genes}")
    if args.out_pareto:
        print(f"[NSGA2] Pareto front: {args.out_pareto}")


if __name__ == "__main__":
    main()

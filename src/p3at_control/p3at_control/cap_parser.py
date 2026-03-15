#!/usr/bin/env python3
"""
Code-as-Policies baseline parser.

Input:
  - natural language command (PT/EN) and/or structured candidate
Output:
  - normalized JSON command compatible with /p3at_target executor schema
"""
import copy
import re
from typing import Any, Dict, Tuple


GENE_RANGES = {
    "base_speed": (0.05, 0.60),
    "turn_speed": (0.10, 1.50),
    "avoid_gain": (0.10, 5.00),
    "dist_stop": (0.20, 2.00),
    "dist_avoid": (0.30, 3.00),
    "turn_force": (0.10, 3.00),
}

NUM_RANGES = {
    "move_distance": (-10.0, 10.0),   # meters
    "turn_angle": (-360.0, 360.0),    # degrees
    "linear": (-1.0, 1.0),            # m/s
    "angular": (-2.5, 2.5),           # rad/s
    "arc_v": (-1.0, 1.0),             # m/s
    "arc_w": (-2.5, 2.5),             # rad/s
    "arc_duration": (0.1, 120.0),     # s
}


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _to_bool(val: Any) -> bool:
    if isinstance(val, bool):
        return val
    if isinstance(val, (int, float)):
        return val != 0
    if isinstance(val, str):
        return val.strip().lower() in {
            "1", "true", "yes", "on", "enable", "enabled",
            "ativar", "ligar", "start", "iniciar"
        }
    return False


def _num(text: str) -> float:
    return float(text.replace(",", "."))


def _extract_first_number(text: str) -> float:
    m = re.search(r"[-+]?\d+(?:[.,]\d+)?", text)
    if not m:
        raise ValueError("No numeric value found in text command")
    return _num(m.group(0))


def _normalize_structured(candidate: Dict[str, Any]) -> Dict[str, Any]:
    d = copy.deepcopy(candidate)

    # Direct twist fallback
    if "command" not in d and ("linear" in d or "angular" in d):
        lin = float(d.get("linear", 0.0))
        ang = float(d.get("angular", 0.0))
        return {
            "linear": _clamp(lin, *NUM_RANGES["linear"]),
            "angular": _clamp(ang, *NUM_RANGES["angular"]),
        }

    cmd = str(d.get("command", "")).strip().lower()
    val = d.get("value", 0.0)

    if cmd == "move_distance":
        v = _clamp(float(val), *NUM_RANGES["move_distance"])
        return {"command": "move_distance", "value": v}

    if cmd == "turn_angle":
        v = _clamp(float(val), *NUM_RANGES["turn_angle"])
        return {"command": "turn_angle", "value": v}

    if cmd == "explore_mode":
        return {"command": "explore_mode", "value": _to_bool(val)}

    if cmd == "validation_mode":
        return {"command": "validation_mode", "value": _to_bool(val)}

    if cmd == "stop":
        return {"command": "stop", "value": 0.0}

    if cmd == "arc" and isinstance(val, dict):
        v = _clamp(float(val.get("v", 0.0)), *NUM_RANGES["arc_v"])
        w = _clamp(float(val.get("w", 0.0)), *NUM_RANGES["arc_w"])
        dur = _clamp(float(val.get("duration", 1.0)), *NUM_RANGES["arc_duration"])
        return {"command": "arc", "value": {"v": v, "w": w, "duration": dur}}

    if cmd == "set_genes" and isinstance(val, dict):
        out = {}
        for k, (lo, hi) in GENE_RANGES.items():
            if k in val:
                out[k] = _clamp(float(val[k]), lo, hi)
        return {"command": "set_genes", "value": out}

    # Unknown command: fail safe
    return {"command": "stop", "value": 0.0}


def _parse_text(text: str) -> Dict[str, Any]:
    t = text.strip().lower()
    t = re.sub(r"\s+", " ", t)

    if any(k in t for k in ("stop", "parar", "halt")):
        return {"command": "stop", "value": 0.0}

    if any(k in t for k in ("explore on", "explore_mode on", "ativar explor", "iniciar exploracao", "start explore")):
        return {"command": "explore_mode", "value": True}

    if any(k in t for k in ("explore off", "desativar explor", "parar exploracao", "stop explore")):
        return {"command": "explore_mode", "value": False}

    if "move" in t or "andar" in t or "forward" in t or "frente" in t or "voltar" in t or "back" in t:
        dist = _extract_first_number(t)
        if "voltar" in t or "back" in t or "backward" in t:
            dist = -abs(dist)
        return {"command": "move_distance", "value": _clamp(dist, *NUM_RANGES["move_distance"])}

    if "turn" in t or "girar" in t or "rotate" in t:
        deg = _extract_first_number(t)
        if "right" in t or "direita" in t:
            deg = -abs(deg)
        if "left" in t or "esquerda" in t:
            deg = abs(deg)
        return {"command": "turn_angle", "value": _clamp(deg, *NUM_RANGES["turn_angle"])}

    # Optional compact arc command:
    # "arc v=0.2 w=0.5 d=4" or "arco v=0.2 w=0.5 dur=4"
    if "arc" in t or "arco" in t:
        mv = re.search(r"(?:v=|v\s+)([-+]?\d+(?:[.,]\d+)?)", t)
        mw = re.search(r"(?:w=|w\s+)([-+]?\d+(?:[.,]\d+)?)", t)
        md = re.search(r"(?:d=|dur=|duration=|dur\s+|duration\s+)([-+]?\d+(?:[.,]\d+)?)", t)
        v = _clamp(_num(mv.group(1)) if mv else 0.2, *NUM_RANGES["arc_v"])
        w = _clamp(_num(mw.group(1)) if mw else 0.5, *NUM_RANGES["arc_w"])
        d = _clamp(_num(md.group(1)) if md else 3.0, *NUM_RANGES["arc_duration"])
        return {"command": "arc", "value": {"v": v, "w": w, "duration": d}}

    # Fail-safe default
    return {"command": "stop", "value": 0.0}


def parse_cap(payload: Dict[str, Any]) -> Tuple[Dict[str, Any], Dict[str, Any]]:
    """
    Returns:
      normalized_command, meta_info
    """
    meta = {"source": "cap", "input_type": None}

    if isinstance(payload, dict):
        if "text" in payload and isinstance(payload.get("text"), str):
            meta["input_type"] = "text"
            cmd = _parse_text(payload["text"])
            return _normalize_structured(cmd), meta
        meta["input_type"] = "structured"
        return _normalize_structured(payload), meta

    meta["input_type"] = "unknown"
    return {"command": "stop", "value": 0.0}, meta


#!/usr/bin/env python3
"""Discrete root-locus tuning pipeline for SoftBot AB pressure control."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import random
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from typing import Any

import numpy as np

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))


@dataclass
class Segment:
    direction: str
    y: list[float]
    u: list[float]
    source: str
    label: str


@dataclass
class ArxModel:
    direction: str
    a1: float
    a2: float
    b1: float
    b2: float
    delay: int
    ts: float
    train_rmse: float
    val_rmse: float


@dataclass
class Candidate:
    direction: str
    z0: float
    k_loop: float
    kp: float
    ki: float
    poles: list[complex]
    max_pole_mag: float
    time_to_target_s: float
    overshoot_kpa: float
    settling_s: float
    effort_mean: float
    effort_rms: float
    cost: float


def month_output_dir() -> str:
    dirname = time.strftime("%Y-%m")
    path = os.path.join(REPO_ROOT, "experiments", dirname)
    os.makedirs(path, exist_ok=True)
    return path


def sanitize_label(label: str) -> str:
    cleaned = "".join(
        ch if (ch.isalnum() or ch in {"_", "-"}) else "_" for ch in str(label).strip()
    )
    cleaned = cleaned.strip("_")
    return cleaned or "run"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Fit discrete ARX models and tune PI gains via root-locus search."
    )
    parser.add_argument("--id-csv", required=True, nargs="+", help="controller_id_raw CSV paths")
    parser.add_argument(
        "--reference-raw-csv",
        nargs="*",
        default=[],
        help="Optional pump_eval_raw CSVs used as secondary fitting data",
    )
    parser.add_argument("--sample-ms", type=int, default=20)
    parser.add_argument("--target-pressure-kpa", type=float, default=28.0)
    parser.add_argument("--target-vacuum-kpa", type=float, default=-15.0)
    parser.add_argument("--settle-band-kpa", type=float, default=1.0)
    parser.add_argument("--settle-hold-s", type=float, default=0.4)
    parser.add_argument("--max-overshoot-kpa", type=float, default=2.0)
    parser.add_argument("--sim-horizon-s", type=float, default=3.0)
    parser.add_argument("--z0-min", type=float, default=0.60)
    parser.add_argument("--z0-max", type=float, default=0.98)
    parser.add_argument("--z0-points", type=int, default=40)
    parser.add_argument("--k-min", type=float, default=1e-2)
    parser.add_argument("--k-max", type=float, default=1e4)
    parser.add_argument("--k-points", type=int, default=120)
    parser.add_argument("--max-pole-mag", type=float, default=0.995)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--tag", type=str, default="ab")
    return parser.parse_args()


def parse_float(value: str, default: float = 0.0) -> float:
    text = str(value or "").strip()
    if not text:
        return float(default)
    try:
        return float(text)
    except ValueError:
        return float(default)


def parse_int(value: str, default: int = 0) -> int:
    text = str(value or "").strip()
    if not text:
        return int(default)
    try:
        return int(float(text))
    except ValueError:
        return int(default)


def load_controller_id_segments(path: str) -> list[Segment]:
    if not os.path.exists(path):
        raise SystemExit(f"ID CSV not found: {path}")

    buckets: dict[tuple[str, str], dict[str, Any]] = {}
    with open(path, newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            direction = str(row.get("direction", "")).strip().lower()
            if direction not in {"inflate", "suction"}:
                continue
            run_idx = parse_int(row.get("run_idx", "0"), 0)
            repeat_idx = parse_int(row.get("repeat_idx", "0"), 0)
            pwm_cmd = parse_int(row.get("pwm_cmd", row.get("pwm_main", "0")), 0)
            key = (direction, f"run{run_idx:03d}_rep{repeat_idx:03d}_pwm{pwm_cmd:03d}")
            bucket = buckets.setdefault(
                key,
                {
                    "direction": direction,
                    "u": [],
                    "y_raw": [],
                    "label": key[1],
                    "source": os.path.basename(path),
                },
            )
            pressure = parse_float(
                row.get("pressure_kpa", row.get("pressure_filtered_kpa", "0")), 0.0
            )
            u_val = parse_float(row.get("pwm_main", row.get("pwm_cmd", "0")), 0.0)
            bucket["y_raw"].append(pressure)
            bucket["u"].append(u_val)

    segments: list[Segment] = []
    for _, bucket in buckets.items():
        y_raw = bucket["y_raw"]
        u = bucket["u"]
        if len(y_raw) < 8:
            continue
        y0 = y_raw[0]
        if bucket["direction"] == "inflate":
            y = [value - y0 for value in y_raw]
        else:
            y = [-(value - y0) for value in y_raw]
        segments.append(
            Segment(
                direction=bucket["direction"],
                y=y,
                u=u,
                source=bucket["source"],
                label=bucket["label"],
            )
        )
    return segments


def infer_direction_from_phase(phase: str) -> str | None:
    phase = str(phase or "").strip().lower()
    if "pressure" in phase:
        return "inflate"
    if "vacuum" in phase:
        return "suction"
    return None


def load_pump_eval_raw_segments(path: str) -> list[Segment]:
    if not os.path.exists(path):
        raise SystemExit(f"Reference raw CSV not found: {path}")

    buckets: dict[tuple[str, str], dict[str, Any]] = {}
    with open(path, newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            phase = str(row.get("phase", "")).strip()
            direction = infer_direction_from_phase(phase)
            if direction is None:
                continue
            run_idx = parse_int(row.get("run_idx", "0"), 0)
            key = (direction, f"run{run_idx:03d}_{phase}")
            bucket = buckets.setdefault(
                key,
                {
                    "direction": direction,
                    "u": [],
                    "y_raw": [],
                    "label": key[1],
                    "source": os.path.basename(path),
                },
            )
            y = parse_float(row.get("pressure_filtered_kpa", row.get("pressure_raw_kpa", "0")), 0.0)
            u = parse_float(row.get("pwm_main", "0"), 0.0)
            bucket["y_raw"].append(y)
            bucket["u"].append(u)

    segments: list[Segment] = []
    for _, bucket in buckets.items():
        y_raw = bucket["y_raw"]
        u = bucket["u"]
        if len(y_raw) < 8:
            continue
        y0 = y_raw[0]
        if bucket["direction"] == "inflate":
            y = [value - y0 for value in y_raw]
        else:
            y = [-(value - y0) for value in y_raw]
        segments.append(
            Segment(
                direction=bucket["direction"],
                y=y,
                u=u,
                source=bucket["source"],
                label=bucket["label"],
            )
        )
    return segments


def split_segments(segments: list[Segment], seed: int) -> tuple[list[Segment], list[Segment]]:
    items = list(segments)
    rng = random.Random(seed)
    rng.shuffle(items)
    if len(items) <= 2:
        return items, items
    cut = max(1, int(round(0.8 * len(items))))
    if cut >= len(items):
        cut = len(items) - 1
    return items[:cut], items[cut:]


def fit_arx_with_delay(segments: list[Segment], delay: int) -> tuple[np.ndarray, float]:
    rows: list[list[float]] = []
    targets: list[float] = []
    for segment in segments:
        y = segment.y
        u = segment.u
        n = len(y)
        start = max(2, delay + 1)
        if n <= start:
            continue
        for k in range(start, n):
            rows.append([y[k - 1], y[k - 2], u[k - delay], u[k - delay - 1]])
            targets.append(y[k])

    if len(rows) < 8:
        raise RuntimeError("Insufficient samples for ARX fit")

    x = np.asarray(rows, dtype=float)
    y_target = np.asarray(targets, dtype=float)
    coeffs, _, _, _ = np.linalg.lstsq(x, y_target, rcond=None)
    residual = y_target - x @ coeffs
    rmse = float(np.sqrt(np.mean(np.square(residual))))
    return coeffs, rmse


def simulate_arx_segment(coeffs: np.ndarray, delay: int, segment: Segment) -> np.ndarray:
    a1, a2, b1, b2 = coeffs
    y = np.asarray(segment.y, dtype=float)
    u = np.asarray(segment.u, dtype=float)
    n = len(y)
    if n == 0:
        return np.zeros(0, dtype=float)

    y_hat = np.array(y, copy=True)
    start = max(2, delay + 1)
    for k in range(start, n):
        y_hat[k] = a1 * y_hat[k - 1] + a2 * y_hat[k - 2] + b1 * u[k - delay] + b2 * u[k - delay - 1]
    return y_hat


def evaluate_arx_rmse(coeffs: np.ndarray, delay: int, segments: list[Segment]) -> float:
    errs: list[float] = []
    for segment in segments:
        y = np.asarray(segment.y, dtype=float)
        y_hat = simulate_arx_segment(coeffs, delay, segment)
        if len(y) == 0:
            continue
        errs.extend((y - y_hat).tolist())
    if not errs:
        return float("inf")
    return float(np.sqrt(np.mean(np.square(np.asarray(errs, dtype=float)))))


def fit_best_arx_model(direction: str, segments: list[Segment], ts: float, seed: int) -> ArxModel:
    train, val = split_segments(segments, seed=seed)

    best: ArxModel | None = None
    for delay in (0, 1, 2):
        try:
            coeffs, train_rmse = fit_arx_with_delay(train, delay)
        except RuntimeError:
            continue
        val_rmse = evaluate_arx_rmse(coeffs, delay, val)
        model = ArxModel(
            direction=direction,
            a1=float(coeffs[0]),
            a2=float(coeffs[1]),
            b1=float(coeffs[2]),
            b2=float(coeffs[3]),
            delay=delay,
            ts=ts,
            train_rmse=train_rmse,
            val_rmse=val_rmse,
        )
        if best is None or model.val_rmse < best.val_rmse:
            best = model

    if best is None:
        raise RuntimeError(f"Could not fit ARX model for direction={direction}")
    return best


def characteristic_polynomial(model: ArxModel, k_loop: float, z0: float) -> np.ndarray:
    d = int(model.delay)
    d_g = np.array([1.0, -model.a1, -model.a2] + [0.0] * d, dtype=float)
    n_g = np.array([model.b1, model.b2, 0.0], dtype=float)
    den_part = np.polymul(d_g, np.array([1.0, -1.0], dtype=float))
    num_part = float(k_loop) * np.polymul(n_g, np.array([1.0, -float(z0)], dtype=float))
    return np.polyadd(den_part, num_part)


def compute_pi_output(
    error: float, integral_sum: float, kp: float, ki: float
) -> tuple[float, float]:
    integral_candidate = integral_sum + error
    control_candidate = (kp * error) + (ki * integral_candidate)

    control_limited = min(255.0, max(0.0, control_candidate))
    sat_low = control_limited <= 0.0
    sat_high = control_limited >= 255.0
    pushes_further = (sat_low and control_candidate < 0.0) or (
        sat_high and control_candidate > 255.0
    )

    integral_next = integral_sum
    if not pushes_further:
        integral_next = integral_candidate

    if abs(ki) > 1e-9:
        limit = 255.0 / abs(ki)
        integral_next = min(limit, max(-limit, integral_next))
    else:
        integral_next = 0.0

    control = (kp * error) + (ki * integral_next)
    control = min(255.0, max(0.0, control))
    return control, integral_next


def simulate_closed_loop(
    model: ArxModel,
    *,
    kp: float,
    ki: float,
    target_kpa: float,
    horizon_s: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    steps = max(5, int(round(float(horizon_s) / model.ts)) + 1)
    y = np.zeros(steps, dtype=float)
    u = np.zeros(steps, dtype=float)
    t = np.arange(steps, dtype=float) * model.ts

    d = int(model.delay)
    integral_sum = 0.0
    y_km1 = 0.0
    y_km2 = 0.0

    for k in range(steps):
        error = float(target_kpa) - y_km1
        control, integral_sum = compute_pi_output(
            error=error,
            integral_sum=integral_sum,
            kp=float(kp),
            ki=float(ki) * model.ts,
        )
        u[k] = control

        u_kd = u[k - d] if (k - d) >= 0 else 0.0
        u_kd1 = u[k - d - 1] if (k - d - 1) >= 0 else 0.0

        y_k = (model.a1 * y_km1) + (model.a2 * y_km2) + (model.b1 * u_kd) + (model.b2 * u_kd1)
        y[k] = y_k

        y_km2 = y_km1
        y_km1 = y_k

    return t, y, u


def first_crossing_time(t: np.ndarray, y: np.ndarray, threshold: float) -> float | None:
    for idx, value in enumerate(y):
        if value >= threshold:
            return float(t[idx])
    return None


def settling_time(
    t: np.ndarray, y: np.ndarray, target: float, band: float, hold_s: float
) -> float | None:
    cross = first_crossing_time(t, y, target)
    if cross is None:
        return None

    start_idx = 0
    for idx, t_val in enumerate(t):
        if t_val >= cross:
            start_idx = idx
            break

    for i in range(start_idx, len(t)):
        if abs(y[i] - target) > band:
            continue
        t0 = t[i]
        j = i
        stable = True
        while j < len(t) and (t[j] - t0) < hold_s:
            if abs(y[j] - target) > band:
                stable = False
                break
            j += 1
        if stable and (hold_s <= 0.0 or (t[min(j, len(t) - 1)] - t0) >= hold_s):
            return float(t0)
    return None


def evaluate_candidate(
    model: ArxModel,
    *,
    z0: float,
    k_loop: float,
    target_kpa: float,
    settle_band_kpa: float,
    settle_hold_s: float,
    max_overshoot_kpa: float,
    horizon_s: float,
) -> Candidate | None:
    char_poly = characteristic_polynomial(model, k_loop, z0)
    poles = np.roots(char_poly)
    pole_mag = np.abs(poles)
    if np.any(~np.isfinite(pole_mag)):
        return None

    kp = float(k_loop) * float(z0)
    ki = (float(k_loop) * (1.0 - float(z0))) / model.ts
    if kp <= 0.0 or ki <= 0.0:
        return None

    t, y, u = simulate_closed_loop(
        model,
        kp=kp,
        ki=ki,
        target_kpa=target_kpa,
        horizon_s=horizon_s,
    )

    t_target = first_crossing_time(t, y, target_kpa)
    overshoot = max(0.0, float(np.max(y) - target_kpa))
    settle = settling_time(t, y, target_kpa, settle_band_kpa, settle_hold_s)

    t_target_eff = float(horizon_s) if t_target is None else float(t_target)
    settle_eff = float(horizon_s) if settle is None else float(settle)
    effort_mean = float(np.mean(u) / 255.0)
    effort_rms = float(np.sqrt(np.mean(np.square(u))) / 255.0)

    norm_time = min(2.0, t_target_eff / max(1e-9, float(horizon_s)))
    norm_over = min(3.0, overshoot / max(1e-9, float(max_overshoot_kpa)))
    norm_settle = min(2.0, settle_eff / max(1e-9, float(horizon_s)))
    cost = (0.40 * norm_time) + (0.30 * norm_over) + (0.20 * norm_settle) + (0.10 * effort_mean)

    return Candidate(
        direction=model.direction,
        z0=float(z0),
        k_loop=float(k_loop),
        kp=float(kp),
        ki=float(ki),
        poles=[complex(value) for value in poles],
        max_pole_mag=float(np.max(pole_mag)),
        time_to_target_s=t_target_eff,
        overshoot_kpa=overshoot,
        settling_s=settle_eff,
        effort_mean=effort_mean,
        effort_rms=effort_rms,
        cost=float(cost),
    )


def search_root_locus(
    model: ArxModel,
    *,
    target_kpa: float,
    settle_band_kpa: float,
    settle_hold_s: float,
    max_overshoot_kpa: float,
    horizon_s: float,
    z0_values: np.ndarray,
    k_values: np.ndarray,
    max_pole_mag: float,
) -> Candidate:
    best: Candidate | None = None

    for z0 in z0_values:
        for k_loop in k_values:
            candidate = evaluate_candidate(
                model,
                z0=float(z0),
                k_loop=float(k_loop),
                target_kpa=float(target_kpa),
                settle_band_kpa=float(settle_band_kpa),
                settle_hold_s=float(settle_hold_s),
                max_overshoot_kpa=float(max_overshoot_kpa),
                horizon_s=float(horizon_s),
            )
            if candidate is None:
                continue
            if candidate.max_pole_mag >= float(max_pole_mag):
                continue
            if best is None or candidate.cost < best.cost:
                best = candidate

    if best is None:
        raise RuntimeError(
            f"No stable candidate found for {model.direction}. "
            "Try widening K/z0 ranges or improving identification data."
        )
    return best


def plot_root_locus(
    path: str,
    *,
    models: dict[str, ArxModel],
    selected: dict[str, Candidate],
    k_values: np.ndarray,
) -> None:
    os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib-softbot")
    import matplotlib

    matplotlib.use("Agg")
    from matplotlib import pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=(12, 5), constrained_layout=True)
    directions = ["inflate", "suction"]

    for idx, direction in enumerate(directions):
        ax = axes[idx]
        model = models[direction]
        best = selected[direction]

        all_real: list[float] = []
        all_imag: list[float] = []
        for k_loop in k_values:
            poles = np.roots(characteristic_polynomial(model, float(k_loop), best.z0))
            all_real.extend(np.real(poles).tolist())
            all_imag.extend(np.imag(poles).tolist())

        ax.scatter(all_real, all_imag, s=8, alpha=0.25, label="Locus sweep")
        sel_real = [pole.real for pole in best.poles]
        sel_imag = [pole.imag for pole in best.poles]
        ax.scatter(sel_real, sel_imag, s=30, c="#e63946", label="Selected poles")

        theta = np.linspace(0.0, 2.0 * np.pi, 300)
        ax.plot(np.cos(theta), np.sin(theta), "k--", linewidth=1.0, label="Unit circle")
        ax.axhline(0.0, color="#6c757d", linewidth=0.8)
        ax.axvline(0.0, color="#6c757d", linewidth=0.8)
        ax.set_title(f"{direction} | z0={best.z0:.3f}, K={best.k_loop:.3f}")
        ax.set_xlabel("Real(z)")
        ax.set_ylabel("Imag(z)")
        ax.grid(alpha=0.3)
        ax.set_aspect("equal", adjustable="box")
        ax.legend(loc="best", fontsize=8)

    fig.suptitle("Discrete Root Locus (selected z0 per direction)")
    fig.savefig(path, dpi=180)
    plt.close(fig)


def write_report_md(path: str, payload: dict[str, Any]) -> None:
    lines: list[str] = []
    lines.append("# Root Locus Tuning Report")
    lines.append("")
    lines.append(f"- Timestamp: {payload['timestamp']}")
    lines.append("- Method: ARX(2, d=0..2) + discrete root-locus PI search")
    lines.append("")

    for direction in ("inflate", "suction"):
        entry = payload["direction_results"][direction]
        model = entry["model"]
        selected = entry["selected"]
        lines.append(f"## {direction}")
        lines.append("")
        lines.append(
            f"Model: a1={model['a1']:.6f}, a2={model['a2']:.6f}, "
            f"b1={model['b1']:.6f}, b2={model['b2']:.6f}, delay={model['delay']}"
        )
        lines.append(f"Fit RMSE: train={model['train_rmse']:.6f}, val={model['val_rmse']:.6f}")
        lines.append(
            f"Selected PI: z0={selected['z0']:.6f}, K={selected['k_loop']:.6f}, "
            f"kp={selected['kp']:.6f}, ki={selected['ki']:.6f}"
        )
        lines.append(
            f"Predicted: t_target={selected['time_to_target_s']:.3f}s, "
            f"overshoot={selected['overshoot_kpa']:.3f}kPa, "
            f"settling={selected['settling_s']:.3f}s, cost={selected['cost']:.4f}"
        )
        lines.append("")

    rec = payload["recommended_tuning"]
    lines.append("## Recommended Tuning")
    lines.append("")
    lines.append(f"- kp_pos: {rec['kp_pos']:.6f}")
    lines.append(f"- ki_pos: {rec['ki_pos']:.6f}")
    lines.append(f"- kp_neg: {rec['kp_neg']:.6f}")
    lines.append(f"- ki_neg: {rec['ki_neg']:.6f}")
    lines.append("")

    with open(path, "w", encoding="utf-8") as handle:
        handle.write("\n".join(lines) + "\n")


def main() -> int:
    args = parse_args()

    if args.sample_ms <= 0:
        raise SystemExit("--sample-ms debe ser > 0")
    if args.target_pressure_kpa <= 0:
        raise SystemExit("--target-pressure-kpa debe ser > 0")
    if args.target_vacuum_kpa >= 0:
        raise SystemExit("--target-vacuum-kpa debe ser < 0")
    if args.z0_points < 2:
        raise SystemExit("--z0-points debe ser >= 2")
    if args.k_points < 2:
        raise SystemExit("--k-points debe ser >= 2")
    if args.k_min <= 0 or args.k_max <= 0 or args.k_min >= args.k_max:
        raise SystemExit("Rango K inválido")

    ts = float(args.sample_ms) / 1000.0
    all_segments: list[Segment] = []
    for path in args.id_csv:
        all_segments.extend(load_controller_id_segments(path))
    for path in args.reference_raw_csv:
        all_segments.extend(load_pump_eval_raw_segments(path))

    if not all_segments:
        raise SystemExit("No se pudieron cargar segmentos para identificación")

    by_direction = {
        "inflate": [segment for segment in all_segments if segment.direction == "inflate"],
        "suction": [segment for segment in all_segments if segment.direction == "suction"],
    }

    for direction in ("inflate", "suction"):
        if len(by_direction[direction]) < 2:
            raise SystemExit(
                f"Datos insuficientes para {direction}: {len(by_direction[direction])} segmentos"
            )

    models = {
        "inflate": fit_best_arx_model("inflate", by_direction["inflate"], ts=ts, seed=args.seed),
        "suction": fit_best_arx_model(
            "suction", by_direction["suction"], ts=ts, seed=args.seed + 17
        ),
    }

    z0_values = np.linspace(float(args.z0_min), float(args.z0_max), int(args.z0_points))
    k_values = np.logspace(
        math.log10(float(args.k_min)), math.log10(float(args.k_max)), int(args.k_points)
    )

    selected = {
        "inflate": search_root_locus(
            models["inflate"],
            target_kpa=float(args.target_pressure_kpa),
            settle_band_kpa=float(args.settle_band_kpa),
            settle_hold_s=float(args.settle_hold_s),
            max_overshoot_kpa=float(args.max_overshoot_kpa),
            horizon_s=float(args.sim_horizon_s),
            z0_values=z0_values,
            k_values=k_values,
            max_pole_mag=float(args.max_pole_mag),
        ),
        "suction": search_root_locus(
            models["suction"],
            target_kpa=abs(float(args.target_vacuum_kpa)),
            settle_band_kpa=float(args.settle_band_kpa),
            settle_hold_s=float(args.settle_hold_s),
            max_overshoot_kpa=float(args.max_overshoot_kpa),
            horizon_s=float(args.sim_horizon_s),
            z0_values=z0_values,
            k_values=k_values,
            max_pole_mag=float(args.max_pole_mag),
        ),
    }

    recommended = {
        "kp_pos": float(selected["inflate"].kp),
        "ki_pos": float(selected["inflate"].ki),
        "kp_neg": -float(selected["suction"].kp),
        "ki_neg": -float(selected["suction"].ki),
    }

    out_dir = month_output_dir()
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    safe_tag = sanitize_label(args.tag)
    run_dir = os.path.join(out_dir, f"root_locus_{stamp}_{safe_tag}")
    os.makedirs(run_dir, exist_ok=True)

    plot_path = os.path.join(run_dir, "root_locus.png")
    json_path = os.path.join(run_dir, "tuning_result.json")
    md_path = os.path.join(run_dir, "tuning_report.md")

    plot_root_locus(plot_path, models=models, selected=selected, k_values=k_values)

    payload: dict[str, Any] = {
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "inputs": {
            "id_csv": [os.path.abspath(path) for path in args.id_csv],
            "reference_raw_csv": [os.path.abspath(path) for path in args.reference_raw_csv],
            "sample_ms": int(args.sample_ms),
            "target_pressure_kpa": float(args.target_pressure_kpa),
            "target_vacuum_kpa": float(args.target_vacuum_kpa),
            "settle_band_kpa": float(args.settle_band_kpa),
            "settle_hold_s": float(args.settle_hold_s),
            "max_overshoot_kpa": float(args.max_overshoot_kpa),
            "sim_horizon_s": float(args.sim_horizon_s),
            "z0_range": [float(args.z0_min), float(args.z0_max), int(args.z0_points)],
            "k_range": [float(args.k_min), float(args.k_max), int(args.k_points)],
            "max_pole_mag": float(args.max_pole_mag),
            "seed": int(args.seed),
        },
        "direction_results": {
            direction: {
                "model": asdict(models[direction]),
                "selected": {
                    "direction": selected[direction].direction,
                    "z0": selected[direction].z0,
                    "k_loop": selected[direction].k_loop,
                    "kp": selected[direction].kp,
                    "ki": selected[direction].ki,
                    "poles": [
                        {"real": pole.real, "imag": pole.imag} for pole in selected[direction].poles
                    ],
                    "max_pole_mag": selected[direction].max_pole_mag,
                    "time_to_target_s": selected[direction].time_to_target_s,
                    "overshoot_kpa": selected[direction].overshoot_kpa,
                    "settling_s": selected[direction].settling_s,
                    "effort_mean": selected[direction].effort_mean,
                    "effort_rms": selected[direction].effort_rms,
                    "cost": selected[direction].cost,
                },
                "segments_used": len(by_direction[direction]),
            }
            for direction in ("inflate", "suction")
        },
        "recommended_tuning": recommended,
        "artifacts": {
            "root_locus_png": os.path.relpath(plot_path, REPO_ROOT),
            "json": os.path.relpath(json_path, REPO_ROOT),
            "report_md": os.path.relpath(md_path, REPO_ROOT),
        },
    }

    with open(json_path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2)

    write_report_md(md_path, payload)

    print("Root-locus tuning completed")
    print(f"- kp_pos: {recommended['kp_pos']:.6f}")
    print(f"- ki_pos: {recommended['ki_pos']:.6f}")
    print(f"- kp_neg: {recommended['kp_neg']:.6f}")
    print(f"- ki_neg: {recommended['ki_neg']:.6f}")
    print(f"- JSON: {json_path}")
    print(f"- PNG:  {plot_path}")
    print(f"- MD:   {md_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

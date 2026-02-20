#!/usr/bin/env python3
"""
Pump swap validation benchmark for competition preparation.

Goal:
- Quantify how fast each pump setup reaches the same pressure target.
- Keep a persistent registry so "current pumps" vs "new pumps" comparison is fast.

Outputs:
- Raw CSV per sample.
- Summary CSV per run + aggregate stats.
- Optional cumulative registry CSV across sessions.
"""

from __future__ import annotations

import argparse
import csv
import os
import re
import statistics
import time
from collections.abc import Iterable
from dataclasses import dataclass
from datetime import datetime

import rclpy

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
SOFTWARE_ROOT = os.path.join(REPO_ROOT, "software")
if SOFTWARE_ROOT not in os.sys.path:
    os.sys.path.append(SOFTWARE_ROOT)

from sdk.protocol import CHAMBER_ABC  # noqa: E402
from sdk.softbot_interface import SoftBot  # noqa: E402


@dataclass
class Sample:
    pump_label: str
    mode: str
    run_idx: int
    t_s: float
    pressure_kpa: float
    pwm_main: int
    pwm_aux: int
    error_kpa: float


@dataclass
class RunMetrics:
    pump_label: str
    mode: str
    run_idx: int
    target_kpa: float
    chamber: int
    baseline_kpa: float
    peak_kpa: float
    final_kpa: float
    rise_t10_s: float | None
    rise_t90_s: float | None
    rise_t10_90_s: float | None
    time_to_target_s: float | None
    overshoot_kpa: float


@dataclass
class AggregateMetrics:
    rise_t10_90_mean_s: float | None
    rise_t10_90_std_s: float | None
    time_to_target_mean_s: float | None
    time_to_target_std_s: float | None
    overshoot_mean_kpa: float | None
    overshoot_std_kpa: float | None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Benchmark para validar bombas antes/despues de un cambio."
    )
    parser.add_argument(
        "--pump-label",
        type=str,
        required=True,
        help="Etiqueta de la configuracion de bombas (ej: actuales, nuevas_v1).",
    )
    parser.add_argument(
        "--mode",
        type=str,
        default="pid",
        choices=["pid"],
        help="Modo de control durante el benchmark.",
    )
    parser.add_argument(
        "--target-kpa",
        type=float,
        default=35.0,
        help="Setpoint de inflado en kPa.",
    )
    parser.add_argument(
        "--chamber",
        type=int,
        default=CHAMBER_ABC,
        choices=[1, 2, 3, 4, 5, 6, 7],
        help="Mascara de camaras activa (1=A, 2=B, 4=C, combinaciones hasta 7=ABC).",
    )
    parser.add_argument(
        "--runs",
        type=int,
        default=5,
        help="Numero de corridas del benchmark.",
    )
    parser.add_argument(
        "--timeout-s",
        type=float,
        default=3.0,
        help="Tiempo maximo de captura por corrida.",
    )
    parser.add_argument(
        "--sample-ms",
        type=int,
        default=20,
        help="Periodo de muestreo en milisegundos.",
    )
    parser.add_argument(
        "--vent-s",
        type=float,
        default=0.6,
        help="Tiempo de venteo antes de cada corrida.",
    )
    parser.add_argument(
        "--rest-s",
        type=float,
        default=0.4,
        help="Pausa despues de vent y despues de cada corrida.",
    )
    parser.add_argument(
        "--tag",
        type=str,
        default="",
        help="Sufijo opcional para nombres de archivo.",
    )
    parser.add_argument(
        "--registry-csv",
        type=str,
        default="experiments/pump_benchmark_registry.csv",
        help="CSV acumulado con historico de resultados.",
    )
    parser.add_argument(
        "--no-registry",
        action="store_true",
        help="No escribir en el registro acumulado.",
    )
    return parser.parse_args()


def sanitize_label(label: str) -> str:
    cleaned = re.sub(r"[^a-zA-Z0-9_-]+", "_", str(label).strip())
    return cleaned.strip("_") or "pump"


def month_output_dir() -> str:
    dirname = time.strftime("%Y-%m")
    path = os.path.join(REPO_ROOT, "experiments", dirname)
    os.makedirs(path, exist_ok=True)
    return path


def first_crossing(samples: Iterable[Sample], threshold: float) -> float | None:
    for sample in samples:
        if sample.pressure_kpa >= threshold:
            return sample.t_s
    return None


def collect_run(
    bot: SoftBot,
    pump_label: str,
    mode: str,
    run_idx: int,
    target_kpa: float,
    chamber: int,
    timeout_s: float,
    sample_period_s: float,
    vent_s: float,
    rest_s: float,
) -> tuple[list[Sample], RunMetrics]:
    bot.stop()
    bot.set_chamber(chamber)
    bot.vent(chamber_id=chamber, duration_s=vent_s)
    time.sleep(max(0.0, rest_s))

    pre_state = bot.get_state()
    baseline = float(pre_state["pressure"])

    if mode == "pid":
        bot.inflate(target_kpa)
    else:
        raise ValueError(f"Unsupported mode: {mode}")

    samples: list[Sample] = []
    t0 = time.time()

    while True:
        now = time.time()
        t_rel = now - t0
        state = bot.get_state()
        samples.append(
            Sample(
                pump_label=pump_label,
                mode=mode,
                run_idx=run_idx,
                t_s=t_rel,
                pressure_kpa=float(state["pressure"]),
                pwm_main=int(state["pwm_main"]),
                pwm_aux=int(state["pwm_aux"]),
                error_kpa=float(state["error"]),
            )
        )

        if t_rel >= timeout_s:
            break
        time.sleep(sample_period_s)

    bot.stop()
    time.sleep(max(0.0, rest_s))

    peak_kpa = max(sample.pressure_kpa for sample in samples)
    final_kpa = samples[-1].pressure_kpa
    p10 = baseline + 0.1 * (target_kpa - baseline)
    p90 = baseline + 0.9 * (target_kpa - baseline)
    t10 = first_crossing(samples, p10)
    t90 = first_crossing(samples, p90)
    rise_t10_90 = None
    if t10 is not None and t90 is not None and t90 >= t10:
        rise_t10_90 = t90 - t10
    t_target = first_crossing(samples, target_kpa)
    overshoot = peak_kpa - target_kpa

    metrics = RunMetrics(
        pump_label=pump_label,
        mode=mode,
        run_idx=run_idx,
        target_kpa=target_kpa,
        chamber=chamber,
        baseline_kpa=baseline,
        peak_kpa=peak_kpa,
        final_kpa=final_kpa,
        rise_t10_s=t10,
        rise_t90_s=t90,
        rise_t10_90_s=rise_t10_90,
        time_to_target_s=t_target,
        overshoot_kpa=overshoot,
    )
    return samples, metrics


def compute_aggregate(rows: list[RunMetrics]) -> AggregateMetrics:
    rise_vals = [row.rise_t10_90_s for row in rows if row.rise_t10_90_s is not None]
    target_vals = [row.time_to_target_s for row in rows if row.time_to_target_s is not None]
    overshoot_vals = [row.overshoot_kpa for row in rows]

    rise_mean = statistics.mean(rise_vals) if rise_vals else None
    rise_std = statistics.pstdev(rise_vals) if len(rise_vals) > 1 else (0.0 if rise_vals else None)
    target_mean = statistics.mean(target_vals) if target_vals else None
    target_std = (
        statistics.pstdev(target_vals) if len(target_vals) > 1 else (0.0 if target_vals else None)
    )
    overshoot_mean = statistics.mean(overshoot_vals) if overshoot_vals else None
    overshoot_std = (
        statistics.pstdev(overshoot_vals)
        if len(overshoot_vals) > 1
        else (0.0 if overshoot_vals else None)
    )

    return AggregateMetrics(
        rise_t10_90_mean_s=rise_mean,
        rise_t10_90_std_s=rise_std,
        time_to_target_mean_s=target_mean,
        time_to_target_std_s=target_std,
        overshoot_mean_kpa=overshoot_mean,
        overshoot_std_kpa=overshoot_std,
    )


def to_num(value: float | None) -> str:
    if value is None:
        return ""
    return f"{value:.6f}"


def write_outputs(
    raw_path: str,
    summary_path: str,
    all_samples: list[Sample],
    all_metrics: list[RunMetrics],
    aggregate: AggregateMetrics,
) -> None:
    with open(raw_path, "w", newline="", encoding="utf-8") as raw_file:
        writer = csv.writer(raw_file)
        writer.writerow(
            [
                "pump_label",
                "mode",
                "run_idx",
                "t_s",
                "pressure_kpa",
                "pwm_main",
                "pwm_aux",
                "error_kpa",
            ]
        )
        for row in all_samples:
            writer.writerow(
                [
                    row.pump_label,
                    row.mode,
                    row.run_idx,
                    f"{row.t_s:.6f}",
                    f"{row.pressure_kpa:.6f}",
                    row.pwm_main,
                    row.pwm_aux,
                    f"{row.error_kpa:.6f}",
                ]
            )

    with open(summary_path, "w", newline="", encoding="utf-8") as summary_file:
        writer = csv.writer(summary_file)
        writer.writerow(
            [
                "pump_label",
                "mode",
                "run_idx",
                "target_kpa",
                "chamber",
                "baseline_kpa",
                "peak_kpa",
                "final_kpa",
                "rise_t10_s",
                "rise_t90_s",
                "rise_t10_90_s",
                "time_to_target_s",
                "overshoot_kpa",
            ]
        )
        for row in all_metrics:
            writer.writerow(
                [
                    row.pump_label,
                    row.mode,
                    row.run_idx,
                    f"{row.target_kpa:.6f}",
                    row.chamber,
                    f"{row.baseline_kpa:.6f}",
                    f"{row.peak_kpa:.6f}",
                    f"{row.final_kpa:.6f}",
                    to_num(row.rise_t10_s),
                    to_num(row.rise_t90_s),
                    to_num(row.rise_t10_90_s),
                    to_num(row.time_to_target_s),
                    f"{row.overshoot_kpa:.6f}",
                ]
            )

        writer.writerow([])
        writer.writerow(
            [
                "rise_t10_90_mean_s",
                "rise_t10_90_std_s",
                "time_to_target_mean_s",
                "time_to_target_std_s",
                "overshoot_mean_kpa",
                "overshoot_std_kpa",
            ]
        )
        writer.writerow(
            [
                to_num(aggregate.rise_t10_90_mean_s),
                to_num(aggregate.rise_t10_90_std_s),
                to_num(aggregate.time_to_target_mean_s),
                to_num(aggregate.time_to_target_std_s),
                to_num(aggregate.overshoot_mean_kpa),
                to_num(aggregate.overshoot_std_kpa),
            ]
        )


def registry_path(path_arg: str) -> str:
    candidate = path_arg.strip()
    if not candidate:
        candidate = "experiments/pump_benchmark_registry.csv"
    if os.path.isabs(candidate):
        return candidate
    return os.path.join(REPO_ROOT, candidate)


def append_registry(
    path: str,
    args: argparse.Namespace,
    aggregate: AggregateMetrics,
    summary_path: str,
) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    exists = os.path.exists(path)
    with open(path, "a", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        if not exists:
            writer.writerow(
                [
                    "timestamp",
                    "pump_label",
                    "mode",
                    "target_kpa",
                    "chamber",
                    "runs",
                    "timeout_s",
                    "sample_ms",
                    "vent_s",
                    "rest_s",
                    "time_to_target_mean_s",
                    "time_to_target_std_s",
                    "rise_t10_90_mean_s",
                    "rise_t10_90_std_s",
                    "overshoot_mean_kpa",
                    "overshoot_std_kpa",
                    "summary_csv",
                ]
            )

        writer.writerow(
            [
                datetime.now().isoformat(timespec="seconds"),
                args.pump_label,
                args.mode,
                f"{args.target_kpa:.6f}",
                args.chamber,
                args.runs,
                f"{args.timeout_s:.3f}",
                args.sample_ms,
                f"{args.vent_s:.3f}",
                f"{args.rest_s:.3f}",
                to_num(aggregate.time_to_target_mean_s),
                to_num(aggregate.time_to_target_std_s),
                to_num(aggregate.rise_t10_90_mean_s),
                to_num(aggregate.rise_t10_90_std_s),
                to_num(aggregate.overshoot_mean_kpa),
                to_num(aggregate.overshoot_std_kpa),
                os.path.relpath(summary_path, REPO_ROOT),
            ]
        )


def _row_float(row: dict[str, str], key: str) -> float | None:
    value = (row.get(key, "") or "").strip()
    if not value:
        return None
    try:
        return float(value)
    except ValueError:
        return None


def _same_float(a: float | None, b: float, eps: float = 1e-6) -> bool:
    return a is not None and abs(a - float(b)) <= eps


def print_registry_ranking(path: str, args: argparse.Namespace) -> None:
    if not os.path.exists(path):
        return

    rows: list[dict[str, str]] = []
    with open(path, newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            if row.get("mode") != args.mode:
                continue
            chamber_raw = (row.get("chamber", "") or "").strip()
            try:
                chamber = int(chamber_raw)
            except ValueError:
                continue
            if chamber != int(args.chamber):
                continue
            target = _row_float(row, "target_kpa")
            if not _same_float(target, args.target_kpa):
                continue
            rows.append(row)

    if not rows:
        return

    rows.sort(
        key=lambda item: (
            _row_float(item, "time_to_target_mean_s") is None,
            _row_float(item, "time_to_target_mean_s") or 0.0,
        )
    )

    print("\n=== Ranking historico (misma camara, target y modo) ===")
    print("Menor time_to_target_mean_s es mejor.")
    print("pump_label | time_to_target_mean_s | rise_t10_90_mean_s | overshoot_mean_kpa")
    for row in rows:
        current_mark = " <== actual" if row.get("pump_label", "") == args.pump_label else ""
        t_target = row.get("time_to_target_mean_s", "") or "N/A"
        rise = row.get("rise_t10_90_mean_s", "") or "N/A"
        over = row.get("overshoot_mean_kpa", "") or "N/A"
        print(f"{row.get('pump_label', 'N/A')} | {t_target} | {rise} | {over}{current_mark}")


def print_run_summary(rows: list[RunMetrics], aggregate: AggregateMetrics) -> None:
    print("\n=== Resumen por corrida ===")
    for row in rows:
        rise_txt = f"{row.rise_t10_90_s:.4f}" if row.rise_t10_90_s is not None else "N/A"
        target_txt = f"{row.time_to_target_s:.4f}" if row.time_to_target_s is not None else "N/A"
        print(
            f"- run={row.run_idx:02d} t10-90={rise_txt}s "
            f"time_to_target={target_txt}s overshoot={row.overshoot_kpa:.3f}kPa"
        )

    print("\n=== Agregado ===")
    rise_mean_txt = (
        aggregate.rise_t10_90_mean_s if aggregate.rise_t10_90_mean_s is not None else "N/A"
    )
    target_mean_txt = (
        aggregate.time_to_target_mean_s if aggregate.time_to_target_mean_s is not None else "N/A"
    )
    print(f"- rise_t10_90_mean={rise_mean_txt} s")
    print(f"- time_to_target_mean={target_mean_txt} s")
    print(f"- overshoot_mean={aggregate.overshoot_mean_kpa:.6f} kPa")


def main() -> None:
    args = parse_args()
    if args.target_kpa <= 0:
        raise SystemExit("--target-kpa debe ser > 0.")
    if args.runs < 1:
        raise SystemExit("--runs debe ser >= 1.")

    out_dir = month_output_dir()
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    safe_label = sanitize_label(args.pump_label)
    safe_tag = sanitize_label(args.tag) if args.tag else ""
    suffix = f"_{safe_tag}" if safe_tag else ""
    raw_path = os.path.join(
        out_dir,
        f"pump_bench_raw_{timestamp}_{safe_label}_{args.mode}{suffix}.csv",
    )
    summary_path = os.path.join(
        out_dir,
        f"pump_bench_summary_{timestamp}_{safe_label}_{args.mode}{suffix}.csv",
    )

    rclpy.init()
    bot = SoftBot()

    all_samples: list[Sample] = []
    all_metrics: list[RunMetrics] = []

    try:
        sample_period_s = max(0.001, args.sample_ms / 1000.0)
        print("Iniciando benchmark de bombas...")
        print(
            f"pump_label={args.pump_label} | mode={args.mode} | chamber={args.chamber} | "
            f"target={args.target_kpa}kPa | runs={args.runs}"
        )
        for run_idx in range(1, args.runs + 1):
            samples, metrics = collect_run(
                bot=bot,
                pump_label=args.pump_label,
                mode=args.mode,
                run_idx=run_idx,
                target_kpa=args.target_kpa,
                chamber=args.chamber,
                timeout_s=args.timeout_s,
                sample_period_s=sample_period_s,
                vent_s=args.vent_s,
                rest_s=args.rest_s,
            )
            all_samples.extend(samples)
            all_metrics.append(metrics)
            rise_txt = (
                f"{metrics.rise_t10_90_s:.4f}" if metrics.rise_t10_90_s is not None else "N/A"
            )
            target_txt = (
                f"{metrics.time_to_target_s:.4f}" if metrics.time_to_target_s is not None else "N/A"
            )
            print(
                f"  run={run_idx:02d} t10-90={rise_txt}s "
                f"time_to_target={target_txt}s overshoot={metrics.overshoot_kpa:.3f}kPa"
            )

        aggregate = compute_aggregate(all_metrics)
        write_outputs(
            raw_path=raw_path,
            summary_path=summary_path,
            all_samples=all_samples,
            all_metrics=all_metrics,
            aggregate=aggregate,
        )
        print_run_summary(all_metrics, aggregate)
        print(f"\nCSV raw: {raw_path}")
        print(f"CSV summary: {summary_path}")

        if not args.no_registry:
            path = registry_path(args.registry_csv)
            append_registry(path=path, args=args, aggregate=aggregate, summary_path=summary_path)
            print(f"Registro acumulado: {path}")
            print_registry_ranking(path=path, args=args)
    finally:
        bot.stop()
        bot.close()


if __name__ == "__main__":
    main()

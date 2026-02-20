#!/usr/bin/env python3
"""
Bench routine: PID vs Turbo+PID inflation comparison.

Outputs:
- Raw time series CSV (one row per sample)
- Summary CSV (one row per run + grouped aggregates by mode)
"""

from __future__ import annotations

import argparse
import csv
import os
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
    mode: str
    run_idx: int
    t_s: float
    pressure_kpa: float
    pwm_main: int
    pwm_aux: int
    error_kpa: float


@dataclass
class RunMetrics:
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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Bench routine for turbo validation.")
    parser.add_argument(
        "--target-kpa",
        type=float,
        default=35.0,
        help="Inflation setpoint for both PID and Turbo+PID runs.",
    )
    parser.add_argument(
        "--chamber",
        type=int,
        default=CHAMBER_ABC,
        choices=[1, 2, 3, 4, 5, 6, 7],
        help="Active chamber mask for the test (1=A, 2=B, 4=C, 7=A+B+C).",
    )
    parser.add_argument(
        "--runs",
        type=int,
        default=5,
        help="Repetitions per mode.",
    )
    parser.add_argument(
        "--timeout-s",
        type=float,
        default=3.0,
        help="Max capture time per run.",
    )
    parser.add_argument(
        "--sample-ms",
        type=int,
        default=20,
        help="Sampling period in milliseconds.",
    )
    parser.add_argument(
        "--vent-s",
        type=float,
        default=0.6,
        help="Venting time before each run for repeatable start state.",
    )
    parser.add_argument(
        "--rest-s",
        type=float,
        default=0.4,
        help="Rest time after vent and after each run.",
    )
    parser.add_argument(
        "--tag",
        type=str,
        default="",
        help="Optional tag for output filename.",
    )
    return parser.parse_args()


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

    peak_kpa = max(s.pressure_kpa for s in samples)
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


def to_num(value: float | None) -> str:
    if value is None:
        return ""
    return f"{value:.6f}"


def write_outputs(
    raw_path: str,
    summary_path: str,
    all_samples: list[Sample],
    all_metrics: list[RunMetrics],
) -> None:
    with open(raw_path, "w", newline="", encoding="utf-8") as raw_file:
        writer = csv.writer(raw_file)
        writer.writerow(
            [
                "mode",
                "run_idx",
                "t_s",
                "pressure_kpa",
                "pwm_main",
                "pwm_aux",
                "error_kpa",
            ]
        )
        for sample in all_samples:
            writer.writerow(
                [
                    sample.mode,
                    sample.run_idx,
                    f"{sample.t_s:.6f}",
                    f"{sample.pressure_kpa:.6f}",
                    sample.pwm_main,
                    sample.pwm_aux,
                    f"{sample.error_kpa:.6f}",
                ]
            )

    with open(summary_path, "w", newline="", encoding="utf-8") as summary_file:
        writer = csv.writer(summary_file)
        writer.writerow(
            [
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
                "mode",
                "rise_t10_90_mean_s",
                "rise_t10_90_std_s",
                "time_to_target_mean_s",
                "overshoot_mean_kpa",
                "overshoot_std_kpa",
            ]
        )
        for mode in ("pid",):
            metrics = [m for m in all_metrics if m.mode == mode]
            rise_vals = [m.rise_t10_90_s for m in metrics if m.rise_t10_90_s is not None]
            target_vals = [m.time_to_target_s for m in metrics if m.time_to_target_s is not None]
            overshoot_vals = [m.overshoot_kpa for m in metrics]

            rise_mean = statistics.mean(rise_vals) if rise_vals else None
            rise_std = statistics.pstdev(rise_vals) if len(rise_vals) > 1 else 0.0
            target_mean = statistics.mean(target_vals) if target_vals else None
            over_mean = statistics.mean(overshoot_vals) if overshoot_vals else None
            over_std = statistics.pstdev(overshoot_vals) if len(overshoot_vals) > 1 else 0.0

            writer.writerow(
                [
                    mode,
                    to_num(rise_mean),
                    to_num(rise_std),
                    to_num(target_mean),
                    to_num(over_mean),
                    to_num(over_std),
                ]
            )


def print_summary(metrics: list[RunMetrics]) -> None:
    print("\n=== Resumen por corrida ===")
    for row in metrics:
        print(
            f"- {row.mode:9s} run={row.run_idx:02d} "
            f"t10-90={row.rise_t10_90_s if row.rise_t10_90_s is not None else 'N/A'} "
            f"ttarget={row.time_to_target_s if row.time_to_target_s is not None else 'N/A'} "
            f"overshoot={row.overshoot_kpa:.3f} kPa"
        )

    print("\n=== Promedios ===")
    for mode in ("pid",):
        by_mode = [m for m in metrics if m.mode == mode]
        rise_vals = [m.rise_t10_90_s for m in by_mode if m.rise_t10_90_s is not None]
        target_vals = [m.time_to_target_s for m in by_mode if m.time_to_target_s is not None]
        overshoot_vals = [m.overshoot_kpa for m in by_mode]

        rise_mean = statistics.mean(rise_vals) if rise_vals else None
        target_mean = statistics.mean(target_vals) if target_vals else None
        over_mean = statistics.mean(overshoot_vals) if overshoot_vals else None
        print(
            f"- {mode:9s} "
            f"rise_t10_90_mean={rise_mean if rise_mean is not None else 'N/A'} s "
            f"time_to_target_mean={target_mean if target_mean is not None else 'N/A'} s "
            f"overshoot_mean={over_mean if over_mean is not None else 'N/A'} kPa"
        )


def main() -> None:
    args = parse_args()
    if args.target_kpa <= 0:
        raise SystemExit("target-kpa debe ser > 0 para prueba de inflado.")

    out_dir = month_output_dir()
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    suffix = f"_{args.tag}" if args.tag else ""
    raw_path = os.path.join(out_dir, f"bench_turbo_raw_{timestamp}{suffix}.csv")
    summary_path = os.path.join(out_dir, f"bench_turbo_summary_{timestamp}{suffix}.csv")

    rclpy.init()
    bot = SoftBot()

    all_samples: list[Sample] = []
    all_metrics: list[RunMetrics] = []

    try:
        print("Iniciando rutina de banco PID vs Turbo+PID...")
        print(
            f"target={args.target_kpa} kPa | chamber={args.chamber} | "
            f"runs/mode={args.runs} | timeout={args.timeout_s}s"
        )
        sample_period_s = max(0.001, args.sample_ms / 1000.0)

        modes = ("pid",)
        for mode in modes:
            print(f"\n>> Ejecutando modo: {mode}")
            for run_idx in range(1, args.runs + 1):
                samples, metrics = collect_run(
                    bot=bot,
                    mode=mode,
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
                    f"{metrics.rise_t10_90_s}" if metrics.rise_t10_90_s is not None else "N/A"
                )
                print(
                    f"  run={run_idx:02d} "
                    f"t10-90={rise_txt} "
                    f"overshoot={metrics.overshoot_kpa:.3f} kPa"
                )

        write_outputs(
            raw_path=raw_path,
            summary_path=summary_path,
            all_samples=all_samples,
            all_metrics=all_metrics,
        )
        print_summary(all_metrics)
        print(f"\nCSV raw: {raw_path}")
        print(f"CSV summary: {summary_path}")
    finally:
        bot.stop()
        bot.close()


if __name__ == "__main__":
    main()

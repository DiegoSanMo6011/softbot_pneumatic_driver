#!/usr/bin/env python3
"""Capture open-loop PWM identification datasets for AB chamber."""

from __future__ import annotations

import argparse
import csv
import os
import sys
import time
from dataclasses import dataclass
from datetime import datetime

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
SOFTWARE_ROOT = os.path.join(REPO_ROOT, "software")
if SOFTWARE_ROOT not in sys.path:
    sys.path.append(SOFTWARE_ROOT)

from sdk.protocol import CHAMBER_AB, MODE_PWM_INFLATE, MODE_PWM_SUCTION  # noqa: E402


@dataclass
class Sample:
    run_idx: int
    direction: str
    pwm_cmd: int
    repeat_idx: int
    t_session_s: float
    t_step_s: float
    pressure_kpa: float
    pwm_main: int
    pwm_aux: int
    error_kpa: float
    chamber: int


def month_output_dir() -> str:
    dirname = time.strftime("%Y-%m")
    path = os.path.join(REPO_ROOT, "experiments", dirname)
    os.makedirs(path, exist_ok=True)
    return path


def parse_pwm_levels(raw: str) -> list[int]:
    values: list[int] = []
    for token in str(raw).split(","):
        token = token.strip()
        if not token:
            continue
        values.append(int(token))
    if not values:
        raise SystemExit("--pwm-levels debe incluir al menos un valor")
    for value in values:
        if value < 0 or value > 255:
            raise SystemExit("--pwm-levels debe estar en rango 0..255")
    return values


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture open-loop PWM identification data for inflate/suction."
    )
    parser.add_argument("--pump-label", type=str, default="ab")
    parser.add_argument("--chamber", type=int, default=CHAMBER_AB, choices=[1, 2, 3, 4, 5, 6, 7])
    parser.add_argument("--pwm-levels", type=str, default="80,120,160,200")
    parser.add_argument("--step-duration-s", type=float, default=2.5)
    parser.add_argument("--sample-ms", type=int, default=20)
    parser.add_argument("--vent-s", type=float, default=0.6)
    parser.add_argument("--rest-s", type=float, default=0.4)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument(
        "--directions",
        type=str,
        default="inflate,suction",
        help="Comma-separated directions: inflate,suction",
    )
    parser.add_argument("--tag", type=str, default="")
    parser.add_argument("--demo", action="store_true", help="Run with synthetic pressure model")
    return parser.parse_args()


def resolve_directions(raw: str) -> list[str]:
    values = [item.strip().lower() for item in str(raw).split(",") if item.strip()]
    if not values:
        raise SystemExit("--directions requiere al menos un valor")
    allowed = {"inflate", "suction"}
    invalid = [value for value in values if value not in allowed]
    if invalid:
        raise SystemExit(f"Direcciones inválidas: {', '.join(invalid)}")
    dedup: list[str] = []
    for value in values:
        if value not in dedup:
            dedup.append(value)
    return dedup


def write_csv(path: str, rows: list[Sample]) -> None:
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "run_idx",
                "direction",
                "pwm_cmd",
                "repeat_idx",
                "t_session_s",
                "t_step_s",
                "pressure_kpa",
                "pwm_main",
                "pwm_aux",
                "error_kpa",
                "chamber",
            ]
        )
        for row in rows:
            writer.writerow(
                [
                    row.run_idx,
                    row.direction,
                    row.pwm_cmd,
                    row.repeat_idx,
                    f"{row.t_session_s:.6f}",
                    f"{row.t_step_s:.6f}",
                    f"{row.pressure_kpa:.6f}",
                    row.pwm_main,
                    row.pwm_aux,
                    f"{row.error_kpa:.6f}",
                    row.chamber,
                ]
            )


def sanitize_label(label: str) -> str:
    cleaned = "".join(
        ch if (ch.isalnum() or ch in {"_", "-"}) else "_" for ch in str(label).strip()
    )
    cleaned = cleaned.strip("_")
    return cleaned or "pump"


def main() -> int:
    args = parse_args()

    if args.step_duration_s <= 0:
        raise SystemExit("--step-duration-s debe ser > 0")
    if args.sample_ms <= 0:
        raise SystemExit("--sample-ms debe ser > 0")
    if args.repeats < 1:
        raise SystemExit("--repeats debe ser >= 1")

    pwm_levels = parse_pwm_levels(args.pwm_levels)
    directions = resolve_directions(args.directions)

    sample_period_s = float(args.sample_ms) / 1000.0
    rows: list[Sample] = []

    if args.demo:
        from pump_eval_core import DemoPump  # local import keeps normal path lightweight

        bot = DemoPump(args.pump_label)
        rclpy = None
    else:
        import rclpy

        from sdk.softbot_interface import SoftBot

        rclpy.init()
        bot = SoftBot()

    session_t0 = time.monotonic()
    run_idx = 0

    try:
        print("Starting controller identification capture...")
        print(
            f"chamber={args.chamber} repeats={args.repeats} "
            f"pwm_levels={pwm_levels} directions={directions}"
        )
        for repeat_idx in range(1, int(args.repeats) + 1):
            for direction in directions:
                mode = MODE_PWM_INFLATE if direction == "inflate" else MODE_PWM_SUCTION
                for pwm_cmd in pwm_levels:
                    run_idx += 1
                    bot.stop()
                    bot.set_chamber(int(args.chamber))
                    bot.vent(chamber_id=int(args.chamber), duration_s=float(args.vent_s))
                    time.sleep(max(0.0, float(args.rest_s)))

                    bot.set_pwm(int(pwm_cmd), mode)
                    step_t0 = time.monotonic()

                    while True:
                        now = time.monotonic()
                        t_step_s = now - step_t0
                        state = bot.get_state()
                        rows.append(
                            Sample(
                                run_idx=run_idx,
                                direction=direction,
                                pwm_cmd=int(pwm_cmd),
                                repeat_idx=repeat_idx,
                                t_session_s=now - session_t0,
                                t_step_s=t_step_s,
                                pressure_kpa=float(state.get("pressure", 0.0)),
                                pwm_main=int(state.get("pwm_main", 0)),
                                pwm_aux=int(state.get("pwm_aux", 0)),
                                error_kpa=float(state.get("error", 0.0)),
                                chamber=int(args.chamber),
                            )
                        )
                        if t_step_s >= float(args.step_duration_s):
                            break
                        time.sleep(sample_period_s)

                    bot.stop()
                    time.sleep(max(0.0, float(args.rest_s)))

                    print(
                        f"  run={run_idx:02d} repeat={repeat_idx} "
                        f"direction={direction} pwm={pwm_cmd} "
                        f"samples={int(args.step_duration_s / sample_period_s) + 1}"
                    )

        out_dir = month_output_dir()
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        safe_label = sanitize_label(args.pump_label)
        tag = sanitize_label(args.tag) if args.tag else ""
        suffix = f"_{tag}" if tag else ""
        out_path = os.path.join(out_dir, f"controller_id_raw_{stamp}_{safe_label}{suffix}.csv")
        write_csv(out_path, rows)

        print("Capture completed")
        print(f"- Rows: {len(rows)}")
        print(f"- CSV: {out_path}")
        return 0
    except KeyboardInterrupt:
        print("Interrupted by user")
        return 130
    finally:
        try:
            bot.stop()
            bot.vent(chamber_id=int(args.chamber), duration_s=float(args.vent_s))
            bot.stop()
        except Exception:
            pass
        try:
            bot.close()
        except Exception:
            pass
        if not args.demo and "rclpy" in locals() and rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    raise SystemExit(main())

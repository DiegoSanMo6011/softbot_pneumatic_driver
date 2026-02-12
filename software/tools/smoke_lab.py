#!/usr/bin/env python3
"""Safe smoke-test sequence for SoftBot Lab platform."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import rclpy

SOFTWARE_ROOT = Path(__file__).resolve().parents[1]
if str(SOFTWARE_ROOT) not in sys.path:
    sys.path.append(str(SOFTWARE_ROOT))

from sdk.softbot_interface import SoftBot  # noqa: E402


def load_profile(profile_path: Path) -> dict:
    with profile_path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="SoftBot lab smoke test")
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--chamber", type=int, default=3, choices=[1, 2, 3])
    parser.add_argument("--inflate-kpa", type=float, default=5.0)
    parser.add_argument("--hold-s", type=float, default=1.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    profile = load_profile(args.profile)

    max_kpa = float(profile["safety"]["max_kpa"])
    min_kpa = float(profile["safety"]["min_kpa"])
    test_setpoint = min(max(args.inflate_kpa, 0.0), max_kpa - 1.0)

    print("[smoke] Starting safe command sequence")
    print(f"[smoke] Profile: {args.profile}")
    print(f"[smoke] Chamber={args.chamber} Setpoint={test_setpoint:.2f} kPa")

    rclpy.init()
    bot = SoftBot()

    try:
        bot.stop()
        time.sleep(0.3)

        bot.set_chamber(args.chamber)
        time.sleep(0.2)

        bot.update_tuning(max_safe=max_kpa, min_safe=min_kpa)
        time.sleep(0.2)

        bot.inflate(test_setpoint)
        time.sleep(max(0.1, args.hold_s))

        state = bot.get_state()
        print(
            "[smoke] Feedback: "
            f"P={state['pressure']:.2f} kPa "
            f"PWM=({state['pwm_main']},{state['pwm_aux']})"
        )

        bot.stop()
        time.sleep(0.2)
        print("[smoke] Completed")
        return 0
    except Exception as exc:
        print(f"[smoke] ERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        try:
            bot.stop()
            bot.close()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())

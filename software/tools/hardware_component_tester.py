#!/usr/bin/env python3
"""Hardware component diagnostics for SoftBot pneumatic driver."""

from __future__ import annotations

import argparse
import os
import shlex
import sys
import time

import rclpy

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from sdk.protocol import (
    HW_MUX_CHAMBER_A,
    HW_MUX_CHAMBER_B,
    HW_PUMP_INFLATE_AUX,
    HW_PUMP_INFLATE_MAIN,
    HW_PUMP_SUCTION_AUX,
    HW_PUMP_SUCTION_MAIN,
    HW_VALVE_CHAMBER_C,
    HW_VALVE_INFLATE,
    HW_VALVE_SUCTION,
)
from sdk.softbot_interface import SoftBot

COMPONENT_BITS = {
    "inflate_main": HW_PUMP_INFLATE_MAIN,
    "inflate_aux": HW_PUMP_INFLATE_AUX,
    "suction_main": HW_PUMP_SUCTION_MAIN,
    "suction_aux": HW_PUMP_SUCTION_AUX,
    "valve_inflate": HW_VALVE_INFLATE,
    "valve_suction": HW_VALVE_SUCTION,
    "valve_chamber_c": HW_VALVE_CHAMBER_C,
    "mux_a": HW_MUX_CHAMBER_A,
    "mux_b": HW_MUX_CHAMBER_B,
}


def clamp_pwm(value: int) -> int:
    return max(0, min(255, int(value)))


def parse_components(raw_components: list[str]) -> int:
    mask = 0
    for raw in raw_components:
        for token in raw.split(","):
            name = token.strip().lower()
            if not name:
                continue
            if name == "all":
                for bit in COMPONENT_BITS.values():
                    mask |= bit
                continue
            if name not in COMPONENT_BITS:
                valid = ", ".join(sorted(COMPONENT_BITS))
                raise ValueError(f"Unknown component '{name}'. Valid: {valid}, all")
            mask |= COMPONENT_BITS[name]
    return mask


def active_components(mask: int) -> list[str]:
    return [name for name, bit in COMPONENT_BITS.items() if mask & bit]


def run_once(bot: SoftBot, mask: int, pwm: int, duration_s: float) -> None:
    bot.set_hardware_test(mask, pwm=pwm)
    print(f"[hardware-test] active={active_components(mask)} pwm={pwm} duration={duration_s:.2f}s")
    time.sleep(max(0.05, float(duration_s)))
    bot.stop_hardware_test()


def run_interactive(bot: SoftBot, initial_pwm: int) -> int:
    mask = 0
    pwm = clamp_pwm(initial_pwm)

    help_text = """
Commands:
  status
  on <component>
  off <component>
  toggle <component>
  pwm <0-255>
  apply
  stop
  help
  quit
Components:
  inflate_main, inflate_aux, suction_main, suction_aux,
  valve_inflate, valve_suction, valve_chamber_c, mux_a, mux_b, all
""".strip()

    print(help_text)

    while True:
        active = active_components(mask)
        prompt = f"hwtest[pwm={pwm} active={active}]> "
        try:
            line = input(prompt)
        except EOFError:
            line = "quit"

        tokens = shlex.split(line)
        if not tokens:
            continue

        cmd = tokens[0].lower()

        if cmd in ("quit", "exit", "q"):
            bot.stop_hardware_test()
            print("[hardware-test] exit")
            return 0

        if cmd in ("help", "h", "?"):
            print(help_text)
            continue

        if cmd == "status":
            print(f"[hardware-test] status mask={mask} active={active_components(mask)} pwm={pwm}")
            continue

        if cmd == "pwm":
            if len(tokens) != 2:
                print("Usage: pwm <0-255>")
                continue
            try:
                pwm = clamp_pwm(int(tokens[1]))
                print(f"[hardware-test] pwm={pwm}")
            except ValueError:
                print("Invalid PWM value")
            continue

        if cmd == "apply":
            bot.set_hardware_test(mask, pwm=pwm)
            print(f"[hardware-test] applied mask={mask} active={active_components(mask)}")
            continue

        if cmd == "stop":
            mask = 0
            bot.stop_hardware_test()
            print("[hardware-test] all outputs OFF")
            continue

        if cmd in ("on", "off", "toggle"):
            if len(tokens) != 2:
                print(f"Usage: {cmd} <component>")
                continue
            try:
                bit = parse_components([tokens[1]])
            except ValueError as exc:
                print(exc)
                continue

            if cmd == "on":
                mask |= bit
            elif cmd == "off":
                mask &= ~bit
            else:
                mask ^= bit

            print(f"[hardware-test] mask={mask} active={active_components(mask)}")
            continue

        print("Unknown command. Use 'help'.")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="SoftBot hardware component tester")
    parser.add_argument(
        "--component",
        action="append",
        default=[],
        help="Component to test (can repeat or use comma list).",
    )
    parser.add_argument("--pwm", type=int, default=120, help="PWM for pump components (0-255)")
    parser.add_argument("--duration-s", type=float, default=1.0, help="Duration per cycle")
    parser.add_argument(
        "--repeat",
        type=int,
        default=1,
        help="Repetitions in non-interactive mode",
    )
    parser.add_argument(
        "--interactive",
        action="store_true",
        help="Interactive hardware panel mode",
    )
    parser.add_argument("--off", action="store_true", help="Send all outputs OFF and exit")
    parser.add_argument("--list-components", action="store_true", help="Print available components")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if args.list_components:
        print("Available components:")
        for name in sorted(COMPONENT_BITS):
            print(f"- {name}")
        print("- all")
        return 0

    pwm = clamp_pwm(args.pwm)

    rclpy.init()
    bot = SoftBot()

    try:
        if args.off:
            bot.stop_hardware_test()
            print("[hardware-test] all outputs OFF")
            return 0

        if args.interactive:
            return run_interactive(bot, initial_pwm=pwm)

        if not args.component:
            raise ValueError("Non-interactive mode requires at least one --component")

        mask = parse_components(args.component)
        repeat = max(1, int(args.repeat))

        for idx in range(repeat):
            print(f"[hardware-test] cycle {idx + 1}/{repeat}")
            run_once(bot, mask=mask, pwm=pwm, duration_s=args.duration_s)
            time.sleep(0.2)

        return 0
    except KeyboardInterrupt:
        print("[hardware-test] interrupted")
        return 1
    except Exception as exc:
        print(f"[hardware-test] ERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        try:
            bot.stop_hardware_test()
            bot.close()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())

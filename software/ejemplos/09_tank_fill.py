#!/usr/bin/env python3
"""
Ejemplo 9: Llenado de tanque (modo 3)
------------------------------------
Inicia llenado y espera estado FULL o TIMEOUT.
"""

import os
import sys
import time

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
import rclpy

from sdk.softbot_interface import SoftBot

TARGET_KPA = 35.0
TIMEOUT_S = 15.0

STATE_MAP = {
    0: "IDLE",
    1: "LLENANDO",
    2: "LLENO",
    3: "TIMEOUT",
}


def main():
    rclpy.init()
    bot = SoftBot()

    try:
        bot.set_chamber(0)
        bot.stop()
        print(f"\n🚰 Llenado de tanque: objetivo {TARGET_KPA} kPa")
        bot.fill_tank(TARGET_KPA)

        start = time.time()
        while (time.time() - start) < TIMEOUT_S:
            state = bot.get_state()
            tank_state = state.get("tank_state", 0)
            print(
                f"\rEstado tanque: {STATE_MAP.get(tank_state, 'N/A')} | "
                f"P={state['pressure']:.2f} kPa",
                end="",
            )

            if tank_state in (2, 3):
                break
            time.sleep(0.05)

        print("\n✅ Finalizado.")

    finally:
        bot.stop()
        bot.close()


if __name__ == "__main__":
    main()

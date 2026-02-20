#!/usr/bin/env python3
"""
Ejemplo 9: Prueba de combinaciones de cámaras (A/B/C)
-----------------------------------------------------
Recorre máscaras de cámara y aplica inflado/succión breve.
"""

import os
import sys
import time

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
import rclpy

from sdk.softbot_interface import SoftBot

SEQUENCE_MASKS = [1, 2, 4, 3, 5, 6, 7]
INFLATE_KPA = 12.0
SUCTION_KPA = -8.0
HOLD_S = 0.8


def mask_label(mask: int) -> str:
    parts = []
    if mask & 1:
        parts.append("A")
    if mask & 2:
        parts.append("B")
    if mask & 4:
        parts.append("C")
    return "+".join(parts) if parts else "BLOCKED"


def main():
    rclpy.init()
    bot = SoftBot()

    try:
        bot.stop()
        print("\n🧪 Prueba de máscaras de cámara")
        for mask in SEQUENCE_MASKS:
            label = mask_label(mask)
            print(f"\n➡️  Cámara {label} (mask={mask}) | Inflado {INFLATE_KPA} kPa")
            bot.set_chamber(mask)
            bot.inflate(INFLATE_KPA)
            time.sleep(HOLD_S)

            state = bot.get_state()
            print(f"   Telemetría: P={state['pressure']:.2f} kPa PWM={state['pwm_main']}")

            print(f"⬅️  Cámara {label} (mask={mask}) | Succión {SUCTION_KPA} kPa")
            bot.suction(SUCTION_KPA)
            time.sleep(HOLD_S)
            bot.stop()
            time.sleep(0.25)

        print("\n✅ Secuencia completada.")
    finally:
        bot.stop()
        bot.close()


if __name__ == "__main__":
    main()

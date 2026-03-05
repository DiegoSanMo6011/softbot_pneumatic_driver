#!/usr/bin/env python3
"""
Ejemplo 8: Prueba de habilitación de Cámara C (pin legacy BOOST)
----------------------------------------------------------------
B: habilitar/deshabilitar C, P: pulso breve de C, ESPACIO: stop, Q: salir
"""

import os
import select
import sys
import termios
import time
import tty

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
import rclpy

from sdk.softbot_interface import SoftBot

C_PULSE_S = 0.15
BASE_MASK = 3  # A+B
C_ON_MASK = 7  # A+B+C

msg = """
⚡ PRUEBA CÁMARA C (pin legacy BOOST)
-----------------------------------
B : C ON/OFF
P : Pulso C corto
ESPACIO : STOP
Q : Salir
"""


def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if rlist else ""
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def set_c_state(bot: SoftBot, c_on: bool) -> int:
    mask = C_ON_MASK if c_on else BASE_MASK
    bot.set_chamber(mask)
    return mask


def print_status(bot: SoftBot, c_on: bool, chamber_mask: int):
    state = bot.get_state()
    sys.stdout.write(
        f"\r\033[KC={'ON' if c_on else 'OFF'} | "
        f"Mask={chamber_mask} | P={state['control_pressure_kpa']:.2f} kPa | PWM={state['pwm_main']}"
    )
    sys.stdout.flush()


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()

    try:
        bot = SoftBot()
        chamber_mask = BASE_MASK
        bot.set_chamber(chamber_mask)
        bot.stop()

        print(msg)
        c_on = False

        while True:
            key = getKey()
            if key in ("b", "B"):
                c_on = not c_on
                chamber_mask = set_c_state(bot, c_on)
                print(f"\n⚡ C {'ON' if c_on else 'OFF'}")
            elif key in ("p", "P"):
                chamber_mask = set_c_state(bot, True)
                print(f"\n⚡ PULSO C {C_PULSE_S:.2f}s")
                time.sleep(C_PULSE_S)
                c_on = False
                chamber_mask = set_c_state(bot, c_on)
            elif key == " ":
                bot.stop()
                c_on = False
                chamber_mask = set_c_state(bot, c_on)
                print("\n🛑 STOP")
            elif key in ("q", "Q", "\x03"):
                break

            print_status(bot, c_on, chamber_mask)
            time.sleep(0.05)

    finally:
        bot.stop()
        bot.close()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print("\n👋 Fin de prueba de Cámara C.")

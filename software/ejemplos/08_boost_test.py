#!/usr/bin/env python3
"""
Ejemplo 8: Prueba de BOOST (tanque)
----------------------------------
B: toggle boost, P: pulso breve, ESPACIO: stop, Q: salir
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

BOOST_PULSE_S = 0.15

msg = """
⚡ PRUEBA BOOST (TANQUE)
----------------------
B : BOOST ON/OFF
P : Pulso BOOST corto
ESPACIO : STOP
Q : Salir
"""


def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if rlist else ""
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def print_status(bot, boost_on):
    state = bot.get_state()
    sys.stdout.write(
        f"\r\033[KBoost={'ON' if boost_on else 'OFF'} | "
        f"P={state['pressure']:.2f} kPa | PWM={state['pwm_main']}"
    )
    sys.stdout.flush()


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()

    try:
        bot = SoftBot()
        bot.set_chamber(3)
        bot.stop()

        print(msg)
        boost_on = False

        while True:
            key = getKey()
            if key in ("b", "B"):
                boost_on = not boost_on
                bot.set_boost(boost_on)
                print(f"\n⚡ BOOST {'ON' if boost_on else 'OFF'}")
            elif key in ("p", "P"):
                bot.pulse_boost(BOOST_PULSE_S)
                print(f"\n⚡ PULSO BOOST {BOOST_PULSE_S:.2f}s")
            elif key == " ":
                bot.stop()
                boost_on = False
                print("\n🛑 STOP")
            elif key in ("q", "Q", "\x03"):
                break

            print_status(bot, boost_on)
            time.sleep(0.05)

    finally:
        bot.set_boost(False)
        bot.stop()
        bot.close()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print("\n👋 Fin de prueba BOOST.")

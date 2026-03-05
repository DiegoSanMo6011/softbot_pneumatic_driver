#!/usr/bin/env python3
"""
Auto Locomotion: Un solo movimiento (loop automático)
-----------------------------------------------------
Corre una secuencia fija tipo "SALTO" (A+B sincronizado) automáticamente,
sin teleoperación de modos. Solo permite INICIAR / FRENAR.

Controles:
  [ S ]       : Iniciar / Reanudar
  [ ESPACIO ] : Parar (E-Stop)
  [ Q ]       : Salir
"""

import os
import select
import sys
import termios
import time
import tty

# Importar la interfaz del SoftBot desde el directorio superior
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
import rclpy

from sdk.softbot_interface import SoftBot

msg = """
🐛 SOFTBOT AUTO-LOCOMOTION (Single Move)
----------------------------------------
Modo: Loop automático (un movimiento)
Controles:
   [ S ]       : INICIAR / REANUDAR
   [ ESPACIO ] : PARADA (E-Stop)
   [ Q ]       : Salir
"""


def getKey():
    """Lee tecla sin bloqueo."""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.01)
    key = sys.stdin.read(1) if rlist else ""
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class AutoSingleMoveSequencer:
    """
    Secuencia única tipo "SALTO" sincronizado:
      Fase 0: Succión (contracción)
      Fase 1: Inflado (expansión)
    """

    def __init__(self, bot: SoftBot):
        self.bot = bot
        self.running = False

        self.phase = 0
        self.last_time = time.time()
        self.first_run = True

        # --- CONFIG (edita aquí) ---
        self.T_PREPARACION = 1.5  # s (succión)
        self.T_SALTO = 2.0  # s (inflado)
        self.P_SUCCION = -18.0  # kPa
        self.P_INFLADO = 35.0  # kPa

        # --- PRINT SENSOR (rate-limit) ---
        self.last_print = 0.0
        self.print_period = 0.10  # segundos (0.10=10Hz, 0.20=5Hz)

    def start(self):
        if not self.running:
            self.running = True
            self.phase = 0
            self.last_time = time.time()
            self.first_run = True
            print("\n▶️  INICIADO: Loop automático (single move)")

    def stop(self):
        if self.running:
            print("\n🛑 STOP: Actuadores detenidos")
        self.running = False
        self.bot.stop()
        self.phase = 0
        self.first_run = True
        self.last_time = time.time()

    def _phase_name(self):
        return "SUCCIONANDO" if self.phase == 0 else "INFLANDO"

    def _target_value(self):
        return self.P_SUCCION if self.phase == 0 else self.P_INFLADO

    def _print_status(self, now: float):
        """Imprime presión actual del sensor (una sola línea) a tasa fija."""
        if (now - self.last_print) < self.print_period:
            return
        self.last_print = now

        state = self.bot.get_state()
        p = state.get("control_pressure_kpa", None)

        if p is None:
            sys.stdout.write("\r\033[K📟 Estado: sin 'control_pressure_kpa' en get_state()")
            sys.stdout.flush()
            return

        sys.stdout.write(
            f"\r\033[K📟 Fase: {self._phase_name()} | "
            f"Objetivo: {self._target_value():.2f} kPa | Actual: {p:.2f} kPa"
        )
        sys.stdout.flush()

    def tick(self):
        if not self.running:
            return

        now = time.time()

        # Siempre mostrar presión mientras corre (pero rate-limited)
        self._print_status(now)

        # Fase 0: CONTRACCIÓN (succión) A+B
        if self.phase == 0:
            if self.first_run:
                print(
                    "\n   Fase 1/2: CONTRACCIÓN (succión) "
                    f"{self.T_PREPARACION:.2f}s | {self.P_SUCCION} kPa"
                )
                self.bot.set_chamber(3)
                self.bot.suction(self.P_SUCCION)
                self.first_run = False

            if (now - self.last_time) >= self.T_PREPARACION:
                self.phase = 1
                self.last_time = now
                self.first_run = True

        # Fase 1: EXPANSIÓN (inflado) A+B
        elif self.phase == 1:
            if self.first_run:
                print(
                    f"\n   Fase 2/2: EXPANSIÓN (inflado) {self.T_SALTO:.2f}s | {self.P_INFLADO} kPa"
                )
                self.bot.set_chamber(3)
                self.bot.inflate(self.P_INFLADO)
                self.first_run = False

            if (now - self.last_time) >= self.T_SALTO:
                self.phase = 0
                self.last_time = now
                self.first_run = True


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()

    try:
        bot = SoftBot()
        seq = AutoSingleMoveSequencer(bot)

        bot.set_chamber(3)
        bot.stop()

        print(msg)

        while True:
            key = getKey()

            if key in ("s", "S"):
                seq.start()
            elif key == " ":
                seq.stop()
            elif key in ("q", "Q", "\x03"):
                break

            seq.tick()
            time.sleep(0.05)

    except Exception as e:
        print(f"\nError: {e}")

    finally:
        try:
            bot.stop()
            bot.close()
        except Exception:
            pass
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print("\n👋 Programa finalizado.")

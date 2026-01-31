"""
Auto Locomotion: Un solo movimiento (loop automático) - con asentamiento
-----------------------------------------------------------------------
A+B sincronizado. Transiciones por presión, pero con:
- MIN_TIME: tiempo mínimo por fase para permitir movimiento físico
- SETTLE_TIME: tiempo estable dentro de tolerancia antes de cambiar
- timeout: si no llega, cambia igual
- SNAP opcional: stop corto entre fases
"""

import sys
import select
import termios
import tty
import os
import time

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import rclpy
from sdk.softbot_interface import SoftBot

msg = """
🐛 SOFTBOT AUTO-LOCOMOTION (Single Move - Settle)
-------------------------------------------------
Modo: Loop automático (A+B sincronizado)
Controles:
   [ S ]       : INICIAR / REANUDAR
   [ ESPACIO ] : PARADA (E-Stop)
   [ Q ]       : Salir
"""

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.01)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class AutoSingleMoveSequencer:
    """
    Fase 0: SUCCIÓN
    Fase 1: INFLADO
    Cambio de fase cuando:
      - elapsed >= MIN_TIME  Y
      - (estuvo dentro de tolerancia por SETTLE_TIME)  O (elapsed >= T_MAX)
    """
    def __init__(self, bot: SoftBot):
        self.bot = bot
        self.running = False

        self.phase = 0
        self.phase_start = time.time()
        self.first_run = True

        # =========================
        # CONFIG (ajusta aquí)
        # =========================

        # Setpoints (ajusta según tu robot)
        self.P_SUCCION = -25.0
        self.P_INFLADO =  40.0

        # Tolerancia para considerar "llegué"
        self.TOL = 3.0  # kPa

        # Tiempo mínimo por fase (clave para que sí complete movimiento)
        self.MIN_SUCCION = 0.50   # s
        self.MIN_INFLADO = 1.00   # s

        # Tiempo que debe mantenerse "estable" dentro de tolerancia
        self.SETTLE_TIME = 0.18   # s (150-250ms suele ir bien)

        # Timeout máximo por fase (por si no llega)
        self.T_SUCCION_MAX = 1.60  # s
        self.T_INFLADO_MAX = 1.60  # s

        # SNAP opcional (stop corto para transición)
        self.SNAP_MS = 60  # ms (0 para desactivar)

        # Print sensor
        self.last_print = 0.0
        self.print_period = 0.08  # ~12.5 Hz

        # Internos
        self._pending_snap = False
        self._snap_until = 0.0
        self._within_since = None  # desde cuándo estoy dentro de tolerancia (y estable)

    # -----------------------------
    # Helpers
    # -----------------------------
    def _phase_name(self):
        return "SUCCIONANDO" if self.phase == 0 else "INFLANDO"

    def _target_value(self):
        return self.P_SUCCION if self.phase == 0 else self.P_INFLADO

    def _get_pressure(self):
        state = self.bot.get_state()
        return state.get("pressure", None)

    def _print_status(self, now: float):
        if (now - self.last_print) < self.print_period:
            return
        self.last_print = now

        p = self._get_pressure()
        if p is None:
            sys.stdout.write("\r\033[K📟 Estado: sin 'pressure' en get_state()")
            sys.stdout.flush()
            return

        sys.stdout.write(
            f"\r\033[K📟 Fase: {self._phase_name()} | Obj: {self._target_value():.1f} kPa | Act: {p:.1f} kPa"
        )
        sys.stdout.flush()

    def _arm_snap(self, now: float):
        if self.SNAP_MS <= 0:
            self._pending_snap = False
            return
        self._pending_snap = True
        self._snap_until = now + (self.SNAP_MS / 1000.0)

    def _do_snap_if_needed(self, now: float):
        if not self._pending_snap:
            return False
        if now < self._snap_until:
            self.bot.stop()
            return True
        self._pending_snap = False
        return False

    def _reset_within(self):
        self._within_since = None

    def _update_within_window(self, now: float, p: float, target: float):
        """
        Marca desde cuándo estamos dentro de tolerancia.
        Si salimos de tolerancia, resetea.
        """
        if abs(p - target) <= self.TOL:
            if self._within_since is None:
                self._within_since = now
        else:
            self._within_since = None

    def _settled_enough(self, now: float):
        if self._within_since is None:
            return False
        return (now - self._within_since) >= self.SETTLE_TIME

    # -----------------------------
    # Control
    # -----------------------------
    def start(self):
        if not self.running:
            self.running = True
            self.phase = 0
            self.phase_start = time.time()
            self.first_run = True
            self._pending_snap = False
            self._reset_within()
            print("\n▶️  INICIADO: Loop automático (single move)")

    def stop(self):
        if self.running:
            print("\n🛑 STOP: Actuadores detenidos")
        self.running = False
        self.bot.stop()
        self.phase = 0
        self.first_run = True
        self.phase_start = time.time()
        self._pending_snap = False
        self._reset_within()

    def tick(self):
        if not self.running:
            return

        now = time.time()
        self._print_status(now)

        if self._do_snap_if_needed(now):
            return

        p = self._get_pressure()
        elapsed = now - self.phase_start

        # -------------------------
        # Fase 0: SUCCIÓN A+B
        # -------------------------
        if self.phase == 0:
            if self.first_run:
                print(
                    f"\n   Fase 1/2: SUCCIÓN | Obj {self.P_SUCCION:.1f} kPa | MIN {self.MIN_SUCCION:.2f}s | MAX {self.T_SUCCION_MAX:.2f}s"
                )
                self.bot.set_chamber(3)
                self.bot.suction(self.P_SUCCION)
                self.first_run = False
                self._reset_within()

            if p is not None:
                self._update_within_window(now, p, self.P_SUCCION)

            can_switch = elapsed >= self.MIN_SUCCION
            timeout = elapsed >= self.T_SUCCION_MAX
            settled = self._settled_enough(now)

            if (can_switch and settled) or timeout:
                self.phase = 1
                self.phase_start = now
                self.first_run = True
                self._arm_snap(now)
                self._reset_within()

        # -------------------------
        # Fase 1: INFLADO A+B
        # -------------------------
        elif self.phase == 1:
            if self.first_run:
                print(
                    f"\n   Fase 2/2: INFLADO | Obj {self.P_INFLADO:.1f} kPa | MIN {self.MIN_INFLADO:.2f}s | MAX {self.T_INFLADO_MAX:.2f}s"
                )
                self.bot.set_chamber(3)
                self.bot.inflate(self.P_INFLADO)
                self.first_run = False
                self._reset_within()

            if p is not None:
                self._update_within_window(now, p, self.P_INFLADO)

            can_switch = elapsed >= self.MIN_INFLADO
            timeout = elapsed >= self.T_INFLADO_MAX
            settled = self._settled_enough(now)

            if (can_switch and settled) or timeout:
                self.phase = 0
                self.phase_start = now
                self.first_run = True
                self._arm_snap(now)
                self._reset_within()


if __name__ == '__main__':
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

            if key in ('s', 'S'):
                seq.start()
            elif key == ' ':
                seq.stop()
            elif key in ('q', 'Q', '\x03'):
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

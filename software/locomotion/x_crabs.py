#!/usr/bin/env python3
"""
SOFTBOT AUTO-LOCOMOTION: 5 Estrategias de Movimiento (Optimized Walk)
---------------------------------------------------------------------
Script maestro para pruebas de laboratorio y demos.
- Ganancias PID optimizadas pre-cargadas.
- Estrategia de Caminata (Swim) ajustada para movimientos bruscos.
- Telemetría de tiempo de llegada en terminal.
"""

import os
import random
import select
import sys
import termios
import time
import tty
from dataclasses import dataclass

# Importar la interfaz del SoftBot
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
import rclpy

from sdk.protocol import (
    CHAMBER_A,
    CHAMBER_AB,
    CHAMBER_B,
    MODE_PWM_SUCTION,
)
from sdk.softbot_interface import SoftBot

# -----------------------------
# CONFIGURACIÓN FÍSICA
# -----------------------------
P_HIGH = 35.0  # kPa (Tracción fuerte)
P_LOW = -15.0  # kPa (Levantamiento rápido)
BOOST_ENABLE = False  # Activar turbo (válvula tanque)
BOOST_PULSE_MS = 150.0  # Duración del pulso de boost (ms)

# --- GANANCIAS ÓPTIMAS (RESULTADO DEL BARRIDO) ---
OPTIMAL_KP_POS = 24.0
OPTIMAL_KI_POS = 1500.0
OPTIMAL_KP_NEG = -75.0
OPTIMAL_KI_NEG = -750.0

# Tiempos de seguridad
T_MIN_GLOBAL = 0.3
T_MAX_GLOBAL = 3.0

MSG = """
🚀 SOFTBOT STRATEGIC CONTROLLER v9.0 (Walking Tuned)
----------------------------------------------------
Estrategias de Movimiento:
   [ 1 ] : SYNC AB (Saltar) - Succión/Inflado Sincronizado
   [ 2 ] : CAMINATA (Alternado) - A Tracciona / B Recupera
   [ 3 ] : GIRO IZQUIERDA (Pivote en A)
   [ 4 ] : GIRO DERECHA (Pivote en B)
   [ 5 ] : RANDOM (Desatasque)

Controles:
   [ S ]       : INICIAR Secuencia
   [ ESPACIO ] : PARADA DE EMERGENCIA (Stop)
   [ Q ]       : Salir
"""


def getKey():
    """Lee tecla sin bloqueo."""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.01)
    key = sys.stdin.read(1) if rlist else ""
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


# =========================================================
# 1. Definición de Fase
# =========================================================
@dataclass
class Phase:
    name: str
    action: str  # "suction", "inflate", "stop"
    chamber: int = CHAMBER_AB
    target: float = 0.0
    min_time: float = T_MIN_GLOBAL
    max_time: float = T_MAX_GLOBAL
    tol: float = 4.0  # Tolerancia amplia para detectar "llegada" rápido
    settle_time: float = 0.05
    snap_ms: float = 0.0
    boost_ms: float = 0.0


# =========================================================
# 2. Estrategias de Locomoción
# =========================================================


class LocomotionStrategy:
    name: str = "Base"

    def build_phases(self) -> list[Phase]:
        raise NotImplementedError


# --- MODO 1: SYNC AB (SALTAR) ---
class StrategySyncAB(LocomotionStrategy):
    name = "SYNC AB (Saltar)"

    def build_phases(self) -> list[Phase]:
        return [
            Phase(
                name="SUCCION",
                action="suction",
                chamber=CHAMBER_AB,
                target=P_LOW,
                min_time=0.8,
                max_time=1.5,
                tol=3.0,
                settle_time=0.1,
                snap_ms=50,
            ),
            Phase(
                name="INFLADO",
                action="inflate",
                chamber=CHAMBER_AB,
                target=P_HIGH,
                min_time=1.0,
                max_time=2.0,
                tol=4.0,
                settle_time=0.05,
                snap_ms=50,
                boost_ms=(BOOST_PULSE_MS if BOOST_ENABLE else 0.0),
            ),
        ]


# --- MODO 2: CAMINATA RÁPIDA (CRAWLING BRUSCO) ---
class StrategySwim(LocomotionStrategy):
    name = "CAMINATA (Alternada)"

    def build_phases(self) -> list[Phase]:
        # Para caminar rápido, reducimos tiempos y quitamos pausas (snap=0)
        return [
            # FASE 1: A Tracciona (Infla) / B Recupera (Succiona)
            # El PhaseRunner se encargará de mandar el comando "ciego" a B
            Phase(
                name="PASO A (A:Empuja / B:Sube)",
                action="inflate",
                chamber=CHAMBER_A,
                target=P_HIGH,
                min_time=0.6,  # Ritmo rápido
                max_time=1.5,
                tol=4.0,  # Tolerancia alta para cambio rápido
                settle_time=0.01,  # Sin pausa, movimiento continuo
                snap_ms=0,  # Cero tiempo muerto
                boost_ms=(BOOST_PULSE_MS if BOOST_ENABLE else 0.0),
            ),
            # FASE 2: B Tracciona (Infla) / A Recupera (Succiona)
            Phase(
                name="PASO B (A:Sube / B:Empuja)",
                action="inflate",
                chamber=CHAMBER_B,
                target=P_HIGH,
                min_time=0.6,
                max_time=1.5,
                tol=4.0,
                settle_time=0.01,
                snap_ms=0,
                boost_ms=(BOOST_PULSE_MS if BOOST_ENABLE else 0.0),
            ),
        ]


# --- MODO 3: GIRO IZQUIERDA ---
class StrategyLeft(LocomotionStrategy):
    name = "GIRO IZQUIERDA"

    def build_phases(self) -> list[Phase]:
        return [
            Phase("A_RECOGE", "suction", CHAMBER_A, P_LOW, min_time=0.4, settle_time=0),
            Phase(
                "A_EMPUJA",
                "inflate",
                CHAMBER_A,
                P_HIGH,
                min_time=0.4,
                settle_time=0,
            ),
        ]


# --- MODO 4: GIRO DERECHA ---
class StrategyRight(LocomotionStrategy):
    name = "GIRO DERECHA"

    def build_phases(self) -> list[Phase]:
        return [
            Phase("B_RECOGE", "suction", CHAMBER_B, P_LOW, min_time=0.4, settle_time=0),
            Phase(
                "B_EMPUJA",
                "inflate",
                CHAMBER_B,
                P_HIGH,
                min_time=0.4,
                settle_time=0,
            ),
        ]


# --- MODO 5: RANDOM ---
class StrategyRandom(LocomotionStrategy):
    name = "RANDOM (Desatasque)"

    def build_phases(self) -> list[Phase]:
        phases = []
        for i in range(8):
            action = random.choice(["inflate", "suction"])
            chamber = random.choice([CHAMBER_A, CHAMBER_B, CHAMBER_AB])
            target = P_HIGH if action == "inflate" else P_LOW
            duration = random.uniform(0.3, 0.7)
            phases.append(
                Phase(
                    f"RND_{i}",
                    action,
                    chamber,
                    target,
                    min_time=duration,
                    max_time=duration + 0.2,
                    tol=10.0,
                )
            )
        return phases


# =========================================================
# 3. Motor de Ejecución
# =========================================================
class PhaseRunner:
    def __init__(self, bot: SoftBot):
        self.bot = bot
        self.running = False
        self.strategy = StrategySyncAB()
        self.phases = self.strategy.build_phases()
        self.idx = 0
        self.phase_start = 0
        self.first_run = True
        self._within_since = None
        self._snap_until = 0.0
        self._pending_snap = False
        self.arrival_time = None
        self._boost_until = 0.0

    def _set_boost(self, enabled: bool):
        try:
            self.bot.set_boost(enabled)
        except Exception:
            pass

    def set_strategy(self, strategy: LocomotionStrategy):
        self.stop()
        self.strategy = strategy
        self.phases = self.strategy.build_phases()
        print(f"\n🔄 ESTRATEGIA: {self.strategy.name}")
        self.start()

    def start(self):
        if not self.running:
            self.running = True
            self.idx = 0
            self.phase_start = time.time()
            self.first_run = True
            self._within_since = None
            self._pending_snap = False
            self._boost_until = 0.0
            print("▶️  GO!")

    def stop(self):
        if self.running:
            print("\n🛑 STOP")
        self.running = False
        self.bot.stop()
        self._set_boost(False)

    def tick(self):
        if not self.running:
            return

        now = time.time()

        # --- Lógica SNAP (Frenado) ---
        if self._pending_snap:
            if now < self._snap_until:
                self.bot.stop()
                self._set_boost(False)
                return
            else:
                self._pending_snap = False
                self.idx = (self.idx + 1) % len(self.phases)
                if isinstance(self.strategy, StrategyRandom) and self.idx == 0:
                    self.phases = self.strategy.build_phases()
                self.phase_start = now
                self.first_run = True
                self._within_since = None
                self.arrival_time = None
                self._boost_until = 0.0

        phase = self.phases[self.idx]
        elapsed = now - self.phase_start
        state = self.bot.get_state()
        p_curr = state["pressure"]

        # --- INICIO DE FASE ---
        if self.first_run:
            print(f"\n   👉 {phase.name} | Obj:{phase.target} kPa")
            self.arrival_time = None

            # TRUCO DE SIMULTANEIDAD PARA CAMINATA (CRAWLING)
            # Objetivo: Lograr movimientos opuestos simultáneos
            if self.strategy.name == "CAMINATA (Alternada)":
                if phase.name.startswith("PASO A"):
                    # 1. B Succiona RÁPIDO (PWM Directo 100%)
                    self.bot.set_chamber(CHAMBER_B)
                    self.bot.set_pwm(255.0, MODE_PWM_SUCTION)
                    time.sleep(0.05)
                    # 2. A Infla CONTROLADO (PID)
                    self.bot.set_chamber(CHAMBER_A)
                    if phase.boost_ms > 0:
                        self.bot.inflate_turbo(phase.target)
                    else:
                        self.bot.inflate(phase.target)
                elif phase.name.startswith("PASO B"):
                    # 1. A Succiona RÁPIDO (PWM Directo 100%)
                    self.bot.set_chamber(CHAMBER_A)
                    self.bot.set_pwm(255.0, MODE_PWM_SUCTION)
                    time.sleep(0.05)
                    # 2. B Infla CONTROLADO (PID)
                    self.bot.set_chamber(CHAMBER_B)
                    if phase.boost_ms > 0:
                        self.bot.inflate_turbo(phase.target)
                    else:
                        self.bot.inflate(phase.target)
            else:
                # Estándar para otros modos
                self.bot.set_chamber(phase.chamber)
                if phase.action == "inflate":
                    if phase.boost_ms > 0:
                        self.bot.inflate_turbo(phase.target)
                    else:
                        self.bot.inflate(phase.target)
                elif phase.action == "suction":
                    self.bot.suction(phase.target)
                elif phase.action == "stop":
                    self.bot.stop()

            if phase.action != "inflate" and phase.boost_ms and phase.boost_ms > 0:
                self._set_boost(True)
                self._boost_until = now + (phase.boost_ms / 1000.0)

            self.first_run = False
            self._within_since = None
            return

        # --- BOOST TIMEOUT ---
        if self._boost_until and now >= self._boost_until:
            self._set_boost(False)
            self._boost_until = 0.0

        # --- CHEQUEO DE TÉRMINO ---
        in_target = False
        error = abs(p_curr - phase.target)

        # Telemetría en línea
        metric_str = f"| ⏱️ Llegada: {self.arrival_time:.2f}s" if self.arrival_time else ""
        sys.stdout.write(
            f"\r      P: {p_curr:.1f} kPa (Err: {error:.1f}) | T: {elapsed:.1f}s {metric_str}   "
        )
        sys.stdout.flush()

        if phase.action != "stop":
            if error <= phase.tol:
                if self._within_since is None:
                    self._within_since = now
                    self.arrival_time = elapsed

                if (now - self._within_since) >= phase.settle_time:
                    in_target = True
            else:
                self._within_since = None

        time_ok = elapsed >= phase.min_time
        timeout = elapsed >= phase.max_time

        # --- TRANSICIÓN ---
        if (in_target and time_ok) or timeout:
            if phase.snap_ms > 0:
                self._pending_snap = True
                self._snap_until = now + (phase.snap_ms / 1000.0)
            else:
                self.idx = (self.idx + 1) % len(self.phases)
                if isinstance(self.strategy, StrategyRandom) and self.idx == 0:
                    self.phases = self.strategy.build_phases()
                self.phase_start = now
                self.first_run = True


# =========================================================
# 4. Main Loop
# =========================================================
if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()

    try:
        bot = SoftBot()

        # --- INYECCIÓN DE GANANCIAS ÓPTIMAS ---
        print("💉 Inyectando ganancias óptimas al firmware...")
        bot.update_tuning(
            kp_pos=OPTIMAL_KP_POS,
            ki_pos=OPTIMAL_KI_POS,
            kp_neg=OPTIMAL_KP_NEG,
            ki_neg=OPTIMAL_KI_NEG,
            max_safe=55.0,  # Margen amplio para evitar cortes
        )
        time.sleep(1.0)

        runner = PhaseRunner(bot)

        # Estado seguro inicial
        bot.set_chamber(CHAMBER_AB)
        bot.stop()

        print(MSG)

        while True:
            key = getKey()

            if key == "1":
                runner.set_strategy(StrategySyncAB())
            elif key == "2":
                runner.set_strategy(StrategySwim())
            elif key == "3":
                runner.set_strategy(StrategyLeft())
            elif key == "4":
                runner.set_strategy(StrategyRight())
            elif key == "5":
                runner.set_strategy(StrategyRandom())
            elif key in ["s", "S"]:
                runner.start()
            elif key == " ":
                runner.stop()
            elif key in ["q", "Q", "\x03"]:
                break

            runner.tick()
            time.sleep(0.02)

    except Exception as e:
        print(f"\n❌ Error: {e}")

    finally:
        bot.stop()
        bot.close()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print("\n👋 Bye.")

#!/usr/bin/env python3
"""
Ejemplo 9: Barrido de Sintonización POSITIVA (Inflado)
------------------------------------------------------
Objetivo: Encontrar Kp/Ki para subir a 40 kPa en < 1.0s.
"""

import csv
import os
import sys
import time
from datetime import datetime

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
import rclpy

from sdk.softbot_interface import SoftBot

# --- RANGO DE PRUEBA (Inflado) ---
KP_VALUES = [15.0, 20.0, 24.0, 28.0]
KI_VALUES = [800.0, 1000.0, 1500.0, 2000.0]

# Constantes fijas (Succión segura)
CONST_KP_NEG = -75.0
CONST_KI_NEG = -750.0
MAX_SAFE = 55.0  # Margen para overshoot

# Parámetros del Escalón
TARGET_PRESSURE = 35.0  # kPa
TEST_DURATION = 3.0  # Segundos
TOLERANCE = 2.0  # kPa


def reset_robot(bot):
    """Vacia el robot antes de inflar"""
    print("   🔄 Reset (Vaciando)...")
    bot.set_chamber(3)  # A+B
    bot.suction(-5.0)
    time.sleep(1.5)
    bot.stop()
    time.sleep(0.2)


def run_test(bot, kp, ki, writer):
    print(f"\n🔥 TEST INFLADO: Kp={kp} | Ki={ki}")

    bot.update_tuning(
        kp_pos=kp,
        ki_pos=ki,
        kp_neg=CONST_KP_NEG,
        ki_neg=CONST_KI_NEG,
        max_safe=MAX_SAFE,
    )

    start_time = time.time()
    bot.inflate(TARGET_PRESSURE)

    rise_time = None
    max_pressure = -999.0

    # Bucle de grabación robusto
    while (time.time() - start_time) < TEST_DURATION:
        now = time.time() - start_time
        try:
            state = bot.get_state()
            p_curr = state["pressure"]

            if p_curr > max_pressure:
                max_pressure = p_curr

            if rise_time is None and p_curr >= (TARGET_PRESSURE - TOLERANCE):
                rise_time = now

            time.sleep(0.02)  # 50Hz
        except Exception:
            pass  # Ignorar errores puntuales de lectura

    bot.stop()

    overshoot = max_pressure - TARGET_PRESSURE
    rt_str = f"{rise_time:.3f}s" if rise_time else "FAIL"

    print(f"   📊 Resultado: Rise={rt_str} | Peak={max_pressure:.2f} kPa")
    writer.writerow([kp, ki, rise_time if rise_time else -1, f"{overshoot:.2f}"])


def main():
    rclpy.init()
    bot = SoftBot()

    base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    out_dir = os.path.join(base_dir, "experiments", time.strftime("%Y-%m"))
    os.makedirs(out_dir, exist_ok=True)
    summary_file = os.path.join(
        out_dir, f"inflation_sweep_{datetime.now().strftime('%Y%m%d_%H%M')}.csv"
    )

    try:
        print(f"\n🚀 INICIANDO BARRIDO POSITIVO (Objetivo {TARGET_PRESSURE} kPa)")
        with open(summary_file, mode="w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(["Kp_Pos", "Ki_Pos", "Rise_Time_s", "Overshoot_kPa"])

            for kp in KP_VALUES:
                for ki in KI_VALUES:
                    reset_robot(bot)
                    run_test(bot, kp, ki, writer)

        print("\n🏁 BARRIDO COMPLETADO.")
        print(f"📊 Archivo: {summary_file}")

    except KeyboardInterrupt:
        print("\n🛑 Cancelado.")
        bot.stop()
    finally:
        bot.close()


if __name__ == "__main__":
    main()

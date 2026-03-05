#!/usr/bin/env python3
"""
Ejemplo 2: Identificación de Sistemas Multi-Escalón
---------------------------------------------------
Ejecuta una secuencia de escalones de presión y guarda los datos
telemétricos en un solo archivo CSV para análisis posterior.
"""

import csv
import os
import sys
import time
from datetime import datetime

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
import rclpy

from sdk.softbot_interface import SoftBot

# --- PARÁMETROS DEL EXPERIMENTO ---
# Secuencia de setpoints fisicos en kPa para modo PID (inflate).
STEP_SEQUENCE = [0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0]
# Para subir y bajar (triangular):
# STEP_SEQUENCE = [0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 35.0, 30.0, 25.0]

STEP_DURATION = 3.0
SAMPLING_RATE = 0.05
CHAMBER_ID = 2


def _get_output_dir():
    base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    out_dir = os.path.join(base_dir, "experiments", time.strftime("%Y-%m"))
    os.makedirs(out_dir, exist_ok=True)
    return out_dir


def main():
    rclpy.init()
    bot = SoftBot()

    out_dir = _get_output_dir()
    filename = os.path.join(
        out_dir, f"sweep_response_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    )

    total_duration = len(STEP_SEQUENCE) * STEP_DURATION

    try:
        print("\n📊 INICIANDO BARRIDO DE IDENTIFICACIÓN (MULTI-ESCALÓN)")
        print(f"   Secuencia (kPa): {STEP_SEQUENCE}")
        print(f"   Duración Total Est: {total_duration} s")
        print(f"   Archivo:  {filename}")
        print("------------------------------------------------")

        bot.set_chamber(CHAMBER_ID)
        bot.stop()
        print("⏳ Estabilizando sensores (2s)...")
        time.sleep(2)

        with open(filename, mode="w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(
                [
                    "Timestamp_s",
                    "Setpoint_kPa",
                    "Feedback_kPa",
                    "PWM_Main",
                    "PWM_Aux",
                    "Error_kPa",
                ]
            )

            print("🚀 INICIANDO SECUENCIA... (Grabando)")
            experiment_start_time = time.time()

            for idx, target_p in enumerate(STEP_SEQUENCE):
                print(f"\n👉 Escalón {idx + 1}/{len(STEP_SEQUENCE)}: Objetivo = {target_p} kPa")
                bot.inflate(target_p)

                step_start_time = time.time()
                while (time.time() - step_start_time) < STEP_DURATION:
                    loop_start = time.time()
                    data = bot.get_state()
                    t_curr = time.time() - experiment_start_time

                    writer.writerow(
                        [
                            f"{t_curr:.4f}",
                            target_p,
                            f"{data['control_pressure_kpa']:.3f}",
                            data["pwm_main"],
                            data["pwm_aux"],
                            f"{target_p - float(data['control_pressure_kpa']):.3f}",
                        ]
                    )

                    print(
                        f"\r   T={t_curr:.2f}s | Ref={target_p} | "
                        f"P={data['control_pressure_kpa']:.2f} kPa",
                        end="",
                    )

                    elapsed = time.time() - loop_start
                    if elapsed < SAMPLING_RATE:
                        time.sleep(SAMPLING_RATE - elapsed)

        print("\n\n✅ Experimento de barrido completado exitosamente.")
        print(f"📁 Datos guardados en: {os.path.abspath(filename)}")

    except KeyboardInterrupt:
        print("\n❌ Experimento abortado por el usuario.")
    finally:
        bot.stop()
        bot.close()
        print("🔌 Conexión cerrada.")


if __name__ == "__main__":
    main()

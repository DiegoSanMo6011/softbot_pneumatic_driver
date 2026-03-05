#!/usr/bin/env python3
"""
Ejemplo 2A: Identificación de Sistemas (Escalón simple)
------------------------------------------------------
Ejecuta una prueba de respuesta al escalón y guarda los datos telemétricos
en un CSV para análisis posterior.
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
TARGET_PRESSURE = 15.0  # kPa (Amplitud del escalón)
DURATION = 5.0  # Duración de grabación (s)
SAMPLING_RATE = 0.05  # Periodo de muestreo (20 Hz)
CHAMBER_ID = 2  # 1=A, 2=B


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
        out_dir, f"step_response_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    )

    try:
        print("\n📊 INICIANDO EXPERIMENTO DE IDENTIFICACIÓN (ESCALÓN SIMPLE)")
        print(f"   Objetivo: {TARGET_PRESSURE} kPa en Cámara {CHAMBER_ID}")
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

            print("🚀 APLICANDO ESCALÓN... (Grabando)")
            start_time = time.time()
            bot.inflate(TARGET_PRESSURE)

            while (time.time() - start_time) < DURATION:
                loop_start = time.time()
                data = bot.get_state()
                t_curr = data["timestamp"] - start_time

                writer.writerow(
                    [
                        f"{t_curr:.4f}",
                        TARGET_PRESSURE,
                        f"{data['control_pressure_kpa']:.3f}",
                        data["pwm_main"],
                        data["pwm_aux"],
                        f"{TARGET_PRESSURE - float(data['control_pressure_kpa']):.3f}",
                    ]
                )

                print(
                    f"T={t_curr:.2f}s | P={data['control_pressure_kpa']:.2f} kPa | "
                    f"PWM={data['pwm_main']}"
                )

                elapsed = time.time() - loop_start
                if elapsed < SAMPLING_RATE:
                    time.sleep(SAMPLING_RATE - elapsed)

        print("\n✅ Experimento completado exitosamente.")
        print(f"📁 Datos guardados en: {os.path.abspath(filename)}")

    except KeyboardInterrupt:
        print("\n❌ Experimento abortado.")
    finally:
        bot.close()


if __name__ == "__main__":
    main()

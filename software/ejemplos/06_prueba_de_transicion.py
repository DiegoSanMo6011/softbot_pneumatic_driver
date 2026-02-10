"""
Ejemplo 6: Prueba de Transición Rápida (Step Response Cycling)
--------------------------------------------------------------
Genera una señal de referencia cuadrada (Positivo <-> Negativo)
para medir la velocidad de actuación (Slew Rate) y los tiempos
de subida/bajada del sistema neumático.

Genera un CSV de alta resolución para análisis.
"""

import csv
import os
import sys
import time
from datetime import datetime

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
import rclpy

from sdk.softbot_interface import SoftBot

# --- CONFIGURACIÓN DEL EXPERIMENTO ---
TARGET_POS = 15.0  # kPa (Cresta de la onda)
TARGET_NEG = -20.0  # kPa (Valle de la onda)
HOLD_TIME = 3.0  # Segundos a mantener cada estado (Frecuencia aprox 0.16Hz)
CYCLES = 4  # Cuántas veces repetir el ciclo
CHAMBER = 1  # 1=A, 2=B, 3=Ambas


def main():
    rclpy.init()
    bot = SoftBot()

    base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    out_dir = os.path.join(base_dir, "experiments", time.strftime("%Y-%m"))
    os.makedirs(out_dir, exist_ok=True)
    filename = os.path.join(out_dir, f"fast_switch_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")

    try:
        print("\n⚡ PRUEBA DE TRANSICIÓN RÁPIDA (ONDA CUADRADA) ⚡")
        print(f"   Config: {TARGET_POS} kPa <-> {TARGET_NEG} kPa")
        print(f"   Ciclos: {CYCLES}")
        print(f"   Cámara: {CHAMBER}")

        # 1. Preparación
        print("⏳ Preparando sistema (Reset a 0)...")
        bot.update_tuning(kp_pos=25.0, kp_neg=-75.0)  # Usar ganancias agresivas para velocidad
        bot.set_chamber(CHAMBER)
        bot.stop()
        time.sleep(2)  # Estabilizar comunicación

        # 2. Bucle de Prueba
        print(f"🔴 GRABANDO DATOS EN: {filename}")
        print("   (Presiona Ctrl+C para detener antes)")

        with open(filename, mode="w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(["Timestamp", "Target_kPa", "Pressure_kPa", "PWM_Main", "PWM_Aux"])

            start_time = time.time()

            for i in range(CYCLES):
                print(f"\n--- CICLO {i + 1}/{CYCLES} ---")

                # A) FASE POSITIVA (SUBIDA)
                print(f"   ⬆️  SUBIENDO a {TARGET_POS} kPa...")
                bot.inflate(TARGET_POS)
                target_now = TARGET_POS

                # Grabar durante el tiempo de espera (Alta resolución)
                phase_start = time.time()
                while (time.time() - phase_start) < HOLD_TIME:
                    state = bot.get_state()
                    t_curr = time.time() - start_time
                    writer.writerow(
                        [
                            f"{t_curr:.3f}",
                            target_now,
                            f"{state['pressure']:.2f}",
                            state["pwm_main"],
                            state["pwm_aux"],
                        ]
                    )
                    time.sleep(0.01)  # Muestreo rápido (10ms teóricos)

                # B) FASE NEGATIVA (BAJADA)
                print(f"   ⬇️  BAJANDO a {TARGET_NEG} kPa (Cruce por cero)...")
                bot.suction(TARGET_NEG)
                target_now = TARGET_NEG

                # Grabar
                phase_start = time.time()
                while (time.time() - phase_start) < HOLD_TIME:
                    state = bot.get_state()
                    t_curr = time.time() - start_time
                    writer.writerow(
                        [
                            f"{t_curr:.3f}",
                            target_now,
                            f"{state['pressure']:.2f}",
                            state["pwm_main"],
                            state["pwm_aux"],
                        ]
                    )
                    time.sleep(0.01)

        print("\n✅ Prueba finalizada.")
        print(f"📂 Abre {filename} en Excel o PlotJuggler para medir el tiempo de respuesta.")

    except KeyboardInterrupt:
        print("\n🛑 Interrumpido por usuario.")
    finally:
        bot.stop()
        bot.close()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Ejemplo 2: Identificación de Sistemas y Adquisición de Datos
------------------------------------------------------------
Ejecuta una prueba de respuesta al escalón y guarda los datos telemétricos
en un archivo CSV para análisis posterior (MATLAB/Python/Excel).
Ideal para sintonizar el controlador PID.
"""
"""
import sys
import os
import time
import csv
from datetime import datetime

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import rclpy
from softbot_interface import SoftBot

# --- PARÁMETROS DEL EXPERIMENTO ---
TARGET_PRESSURE = 15.0   # kPa (Amplitud del escalón)
DURATION = 5.0           # Duración de grabación (segundos)
SAMPLING_RATE = 0.05     # Periodo de muestreo (20Hz)
CHAMBER_ID = 2           # 1=A, 2=B

def main():
    rclpy.init()
    bot = SoftBot()
    
    # Nombre de archivo con timestamp
    filename = f"step_response_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    
    try:
        print(f"\n📊 INICIANDO EXPERIMENTO DE IDENTIFICACIÓN")
        print(f"   Objetivo: {TARGET_PRESSURE} kPa en Cámara {CHAMBER_ID}")
        print(f"   Archivo:  {filename}")
        print("------------------------------------------------")

        # 1. Preparación
        bot.set_chamber(CHAMBER_ID)
        bot.stop()
        print("⏳ Estabilizando sensores (2s)...")
        time.sleep(2)

        # 2. Iniciar Grabación
        with open(filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            # Cabecera CSV científica
            writer.writerow(['Timestamp_s', 'Setpoint_kPa', 'Feedback_kPa', 'PWM_Main', 'PWM_Aux', 'Error_Norm'])

            print("🚀 APLICANDO ESCALÓN... (Grabando)")
            start_time = time.time()
            
            # Aplicar estímulo
            bot.inflate(TARGET_PRESSURE)

            # Bucle de muestreo
            while (time.time() - start_time) < DURATION:
                loop_start = time.time()
                
                # Obtener datos atómicos
                data = bot.get_state()
                t_curr = data['timestamp'] - start_time
                
                # Guardar fila
                writer.writerow([
                    f"{t_curr:.4f}",
                    TARGET_PRESSURE,
                    f"{data['pressure']:.3f}",
                    data['pwm_main'],
                    data['pwm_aux'],
                    f"{data['error']:.3f}"
                ])
                
                # Feedback en consola (más lento para no saturar)
                print(f"T={t_curr:.2f}s | P={data['pressure']:.2f} kPa | PWM={data['pwm_main']}")
                
                # Mantener tasa de muestreo constante
                elapsed = time.time() - loop_start
                if elapsed < SAMPLING_RATE:
                    time.sleep(SAMPLING_RATE - elapsed)

        print("\n✅ Experimento completado exitosamente.")
        print(f"📁 Datos guardados en: {os.path.abspath(filename)}")

    except KeyboardInterrupt:
        print("\n❌ Experimento abortado.")
    finally:
        bot.close()

if __name__ == '__main__':
    main()
    """

#!/usr/bin/env python3
"""
Ejemplo 2 (Modificado): Identificación de Sistemas Multi-Escalón
----------------------------------------------------------------
Ejecuta una secuencia de escalones de presión (barrido) y guarda los datos 
telemétricos en un solo archivo CSV para análisis posterior.
Ideal para validar linealidad o histéresis del sistema.
"""

import sys
import os
import time
import csv
from datetime import datetime

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import rclpy
from softbot_interface import SoftBot

# --- PARÁMETROS DEL EXPERIMENTO ---
# Secuencia de presiones a visitar (kPa). 
# Ejemplo: Escalera ascendente y luego bajada a cero.
# --- PARÁMETROS DEL EXPERIMENTO ---

# Genera una lista: 0, 25, 50, 75... hasta 255.
# range(start, stop, step)
STEP_SEQUENCE = list(range(0, 255, 25)) + [255] 

# Si quieres subir y bajar (Triangular): 0 -> 255 -> 0
# STEP_SEQUENCE = list(range(0, 255, 25)) + [255] + list(range(225, -1, -25))

STEP_DURATION = 3.0      # Reduje un poco el tiempo si son muchos pasos
SAMPLING_RATE = 0.05
CHAMBER_ID = 2

def main():
    rclpy.init()
    bot = SoftBot()
    
    # Nombre de archivo con timestamp
    filename = f"sweep_response_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    
    # Calcular duración total estimada para informar al usuario
    total_duration = len(STEP_SEQUENCE) * STEP_DURATION
    
    try:
        print(f"\n📊 INICIANDO BARRIDO DE IDENTIFICACIÓN (MULTIPLES ESCALONES)")
        print(f"   Secuencia (kPa): {STEP_SEQUENCE}")
        print(f"   Duración Total Est: {total_duration} s")
        print(f"   Archivo:  {filename}")
        print("------------------------------------------------")

        # 1. Preparación
        bot.set_chamber(CHAMBER_ID)
        bot.stop()
        print("⏳ Estabilizando sensores (2s)...")
        time.sleep(2)

        # 2. Iniciar Grabación
        with open(filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            # Cabecera CSV científica (misma estructura)
            writer.writerow(['Timestamp_s', 'Setpoint_kPa', 'Feedback_kPa', 'PWM_Main', 'PWM_Aux', 'Error_Norm'])

            print("🚀 INICIANDO SECUENCIA... (Grabando)")
            experiment_start_time = time.time()
            
            # --- BUCLE DE ESCALONES (SWEEP) ---
            for idx, target_p in enumerate(STEP_SEQUENCE):
                print(f"\n👉 Escalón {idx+1}/{len(STEP_SEQUENCE)}: Objetivo = {target_p} kPa")
                
                # Aplicar estímulo actual
                bot.inflate(target_p)
                
                # Tiempo de inicio de este escalón específico
                step_start_time = time.time()

                # Bucle de muestreo para EL ESCALÓN ACTUAL
                while (time.time() - step_start_time) < STEP_DURATION:
                    loop_start = time.time()
                    
                    # Obtener datos atómicos
                    data = bot.get_state()
                    
                    # Timestamp continuo desde el inicio del experimento
                    t_curr = time.time() - experiment_start_time
                    
                    # Guardar fila (Setpoint varía según el escalón actual)
                    writer.writerow([
                        f"{t_curr:.4f}",
                        target_p,               # Setpoint actual de la lista
                        f"{data['pressure']:.3f}",
                        data['pwm_main'],
                        data['pwm_aux'],
                        f"{data['error']:.3f}"
                    ])
                    
                    # Feedback en consola
                    # Mostramos el tiempo total acumulado
                    print(f"\r   T={t_curr:.2f}s | Ref={target_p} | P={data['pressure']:.2f} kPa", end="")
                    
                    # Mantener tasa de muestreo constante
                    elapsed = time.time() - loop_start
                    if elapsed < SAMPLING_RATE:
                        time.sleep(SAMPLING_RATE - elapsed)

        print("\n\n✅ Experimento de barrido completado exitosamente.")
        print(f"📁 Datos guardados en: {os.path.abspath(filename)}")

    except KeyboardInterrupt:
        print("\n❌ Experimento abortado por el usuario.")
    finally:
        # Asegurar que el robot se detenga al terminar o fallar
        bot.stop()
        bot.close()
        print("🔌 Conexión cerrada.")

if __name__ == '__main__':
    main()
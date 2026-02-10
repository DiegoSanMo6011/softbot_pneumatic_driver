"""
Ejemplo 1: Algoritmo de Locomoción "Gusano" (Peristáltico)
----------------------------------------------------------
Demuestra la coordinación secuencial de cámaras para generar movimiento.
Ciclo: Anclar Trasera (A) -> Extender Delantera (B) -> Anclar Delantera (B) -> Retraer Trasera (A)
"""

import os
import sys
import time

# Añadir directorio padre al path para importar la librería
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
import rclpy

from sdk.softbot_interface import SoftBot


def main():
    rclpy.init()
    bot = SoftBot()

    try:
        print("\n🐛 INICIANDO SECUENCIA DE LOCOMOCIÓN (Ctrl+C para parar)")

        # Configuración de marcha
        P_ANCHOR = 30.0  # kPa (Presión para anclar)
        P_RELEASE = -20.0  # kPa (Presión para liberar)
        T_STEP = 5.0  # Segundos por fase

        cycles = 0
        while True:
            cycles += 1
            print(f"\n--- Ciclo {cycles} ---")

            # FASE 1: Anclar Cuerpo (Cámara A)
            print("1. [A] Anclando (Inflar)...")
            bot.set_chamber(1)  # Seleccionar A
            bot.inflate(P_ANCHOR)
            time.sleep(T_STEP)

            # FASE 2: Extender Frente (Cámara B - Succión para avanzar/estirar)
            # Nota: Depende de la morfología física. Asumimos B es el extensor.
            print("2. [B] Extendiendo (Succión)...")
            bot.set_chamber(2)  # Seleccionar B
            bot.suction(P_RELEASE)
            time.sleep(T_STEP)

            # FASE 3: Anclar Frente (Cámara B - Inflar para agarrar suelo)
            print("3. [B] Anclando Frente (Inflar)...")
            bot.set_chamber(2)
            bot.inflate(P_ANCHOR)
            time.sleep(T_STEP)

            # FASE 4: Liberar Cuerpo (Cámara A - Succión para retraer)
            print("4. [A] Retrayendo Cuerpo (Succión)...")
            bot.set_chamber(1)
            bot.suction(P_RELEASE)
            time.sleep(T_STEP)

    except KeyboardInterrupt:
        print("\n🛑 Interrupción de usuario detectada.")
    finally:
        bot.close()
        print("Robot detenido y desconectado.")


if __name__ == "__main__":
    main()

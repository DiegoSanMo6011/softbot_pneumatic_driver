#!/usr/bin/env python3
"""
SoftBot Calibrator (Single Threaded - V3)
-----------------------------------------
Script de caracterización PWM vs Presión.
- Sin hilos (Threads) para máxima estabilidad con ROS 2.
- Configurable al inicio.
- Genera CSV automático.

USO: python3 calibration_runner.py
"""

import sys
import os
import time
import csv
import datetime
import rclpy


# Importar la interfaz del SoftBot
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import rclpy
from softbot_interface import SoftBot

class CalibrationExperiment:
    def __init__(self):
        # ==========================================================
        # ⚙️ CONFIGURACIÓN DEL EXPERIMENTO (EDITAR AQUÍ)
        # ==========================================================
        
        # Rango de PWM
        self.PWM_START = 110      # Inicio solicitado
        self.PWM_END   = 255      # Máximo
        self.PWM_STEP  = 5        # Saltos (Ej: 110, 115, 120...)
        
        # Tiempos
        self.WAIT_TIME = 3.0      # Segundos para estabilizar presión en cada paso
        
        # Hardware
        # 0=Ninguna, 1=CamA, 2=CamB, 3=AMBAS (A+B)
        self.CHAMBER_ID = 3       
        
        # ==========================================================

        # Inicializar ROS 2
        if not rclpy.ok():
            rclpy.init()
            
        self.bot = SoftBot()
        self.data_log = []
        
        # Pequeña espera inicial para conectar
        self._wait_ros(1.0)

    def _wait_ros(self, duration_sec):
        """
        Espera 'duration_sec' segundos PERO mantiene vivo a ROS.
        Esto reemplaza a los hilos complejos.
        """
        start_time = time.time()
        while (time.time() - start_time) < duration_sec:
            # Procesar mensajes de ROS (Sensor de presión)
            rclpy.spin_once(self.bot, timeout_sec=0.05)

    def save_csv(self):
        """Guarda los datos en CSV al terminar."""
        if not self.data_log:
            print("\n⚠️ No hay datos para guardar.")
            return

        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"calibration_raw_{timestamp}.csv"
        
        keys = ['mode', 'pwm_input', 'pressure_kpa', 'timestamp']
        
        try:
            with open(filename, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=keys)
                writer.writeheader()
                writer.writerows(self.data_log)
            print(f"\n💾 DATOS GUARDADOS: {filename}")
        except Exception as e:
            print(f"\n❌ Error guardando CSV: {e}")

    def run_sweep(self, mode_code, label):
        """Ejecuta el barrido de PWM."""
        print(f"\n📈 INICIANDO BARRIDO: {label.upper()} (Modo {mode_code})")
        print(f"   Cámaras Activas ID: {self.CHAMBER_ID} (Ambas)")
        print(f"{'PWM':<10} | {'PRESIÓN (kPa)':<15} | {'ESTADO'}")
        print("-" * 45)

        # 1. Configurar Cámara y Estado Inicial
        self.bot.set_chamber(self.CHAMBER_ID)
        self.bot.set_pwm(0, 0) # Paro inicial
        self._wait_ros(1.0)

        pwm_range = range(self.PWM_START, self.PWM_END + 1, self.PWM_STEP)

        for pwm in pwm_range:
            try:
                # 2. Enviar comando PWM
                self.bot.set_pwm(pwm, mode_code)
                
                # 3. Esperar estabilización (mientras procesamos ROS)
                # Visualización de espera
                sys.stdout.write(f"\r{pwm:<10} | {'...':<15} | Midiendo...")
                sys.stdout.flush()
                
                self._wait_ros(self.WAIT_TIME)
                
                # 4. Leer datos (ya actualizados por _wait_ros)
                state = self.bot.get_state()
                pressure = state.get('pressure', 0.0)
                
                # 5. Registrar
                self.data_log.append({
                    'mode': label,
                    'pwm_input': pwm,
                    'pressure_kpa': pressure,
                    'timestamp': time.time()
                })
                
                # 6. Mostrar en terminal
                sys.stdout.write(f"\r{pwm:<10} | {pressure:.3f} kPa     | ✅ OK    \n")
                sys.stdout.flush()

            except KeyboardInterrupt:
                print("\n⚠️ Interrumpido por usuario.")
                return False 

        # Finalizar barrido: Parar todo y ventear
        print(f"🏁 Barrido {label} completado. Venteando...")
        self.bot.set_pwm(0, 0) # STOP
        self._wait_ros(2.0)
        return True

    def start(self):
        print("\n=== CALIBRADOR DE SISTEMA NEUMÁTICO (V3) ===")
        print(f"Config: Start={self.PWM_START}, Step={self.PWM_STEP}, CamID={self.CHAMBER_ID}")
        
        try:
            # --- FASE 1: INFLADO ---
            input("\n👉 Presiona ENTER para iniciar INFLADO (Presión Positiva)...")
            # Modo 2 es el que tienes en el arduino para inflado PWM
            if self.run_sweep(2, "inflation"):
                
                # --- FASE 2: SUCCIÓN ---
                print("\n" + "="*40)
                input("👉 Presiona ENTER para iniciar SUCCIÓN (Presión Negativa)...")
                # Modo -2 es succión PWM
                self.run_sweep(-2, "suction")
            
            self.save_csv()

        except KeyboardInterrupt:
            print("\n🛑 Cancelado por usuario.")
            self.save_csv()
            
        except Exception as e:
            print(f"\n❌ Error Fatal: {e}")
            
        finally:
            print("\n👋 Deteniendo Robot y Cerrando...")
            self.bot.set_pwm(0, 0)
            self.bot.stop()
            self._wait_ros(0.5) # Dar tiempo a que salga el mensaje
            
            try:
                self.bot.destroy_node()
                rclpy.shutdown()
            except:
                pass

if __name__ == '__main__':
    experiment = CalibrationExperiment()
    experiment.start()
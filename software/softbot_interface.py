"""
SoftBot Python Interface (SDK)
------------------------------
Clase envolvente (wrapper) para interactuar con el SoftBot Pneumatic Driver
a través de micro-ROS. Abstrae la comunicación DDS y provee métodos
de alto nivel para control y telemetría.

Autor: Diego Gerardo Sanchez Moreno y Arturo Lopez Garcia
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int8, Float32MultiArray, Int16MultiArray
import threading
import time
import csv
from datetime import datetime

class SoftBot(Node):
    def __init__(self):
        super().__init__('softbot_client_api')
        
        # --- Configuración de Tópicos (Debe coincidir con Firmware v5.0) ---
        self.TOPIC_CHAMBER = '/active_chamber'
        self.TOPIC_MODE = '/pressure_mode'
        self.TOPIC_SETPOINT = '/pressure_setpoint'
        self.TOPIC_TUNING = '/tuning_params'
        self.TOPIC_FEEDBACK = '/pressure_feedback'
        self.TOPIC_DEBUG = '/system_debug'

        # --- Publicadores ---
        self.pub_chamber = self.create_publisher(Int8, self.TOPIC_CHAMBER, 10)
        self.pub_mode = self.create_publisher(Int8, self.TOPIC_MODE, 10)
        self.pub_setpoint = self.create_publisher(Float32, self.TOPIC_SETPOINT, 10)
        self.pub_tuning = self.create_publisher(Float32MultiArray, self.TOPIC_TUNING, 10)

        # --- Suscriptores ---
        self.sub_feedback = self.create_subscription(Float32, self.TOPIC_FEEDBACK, self._cb_feedback, 10)
        self.sub_debug = self.create_subscription(Int16MultiArray, self.TOPIC_DEBUG, self._cb_debug, 10)

        # --- Estado Interno ---
        self.pressure_kpa = 0.0
        self.debug_vector = [0, 0, 0, 0] # [PWM_Main, PWM_Aux, Error*100, Mode]
        self._telemetry_lock = threading.Lock()

        # Iniciar hilo de comunicación (Spin)
        self._thread = threading.Thread(target=self._spin_node, daemon=True)
        self._thread.start()
        
        # Valores de Tuning por defecto (Sincronizar con Firmware)
        self.tuning_cache = {
            'kp_pos': 12.0, 'ki_pos': 300.0,
            'kp_neg': -75.0, 'ki_neg': -750.0,
            'max_safe': 45.0, 'min_safe': -60.0
        }

        print("✅ [SoftBot API] Interfaz inicializada. Esperando conexión con el driver...")
        time.sleep(2) # Espera técnica para discovery

    def _spin_node(self):
        rclpy.spin(self)

    def _cb_feedback(self, msg):
        with self._telemetry_lock:
            self.pressure_kpa = msg.data

    def _cb_debug(self, msg):
        if len(msg.data) >= 4:
            with self._telemetry_lock:
                self.debug_vector = msg.data

    # --- MÉTODOS DE CONTROL (API PÚBLICA) ---

    def set_chamber(self, chamber_id: int):
        """
        Selecciona la cámara activa.
        1: Cámara A, 2: Cámara B, 3: Ambas, 0: Ninguna (Bloqueo)
        """
        self.pub_chamber.publish(Int8(data=int(chamber_id)))

    def stop(self):
        """Parada de Emergencia / Reset de Modos"""
        self.pub_mode.publish(Int8(data=0))
        self.pub_setpoint.publish(Float32(data=0.0))
        print("🛑 [SoftBot API] Comando STOP enviado.")

    def inflate(self, target_kpa: float):
        """Modo PI: Inflar a presión objetivo (kPa)"""
        self.pub_mode.publish(Int8(data=1))
        self.pub_setpoint.publish(Float32(data=float(target_kpa)))

    def suction(self, target_kpa: float):
        """Modo PI: Succionar a presión objetivo (kPa)"""
        self.pub_mode.publish(Int8(data=-1))
        self.pub_setpoint.publish(Float32(data=float(target_kpa)))

    def set_pwm(self, mode: int, pwm_val: float):
        """
        Control Manual (Lazo Abierto).
        mode: 2 (Inflar), -2 (Succionar)
        pwm_val: 0.0 - 255.0
        """
        if mode not in [2, -2]:
            print("⚠️ [Error] Modo PWM inválido. Use 2 o -2.")
            return
        self.pub_mode.publish(Int8(data=int(mode)))
        self.pub_setpoint.publish(Float32(data=float(pwm_val)))

    def update_tuning(self, **kwargs):
        """
        Actualiza parámetros PID/Seguridad en tiempo real.
        Argumentos opcionales: kp_pos, ki_pos, kp_neg, ki_neg, max_safe, min_safe
        """
        # Actualizar cache local
        for key, value in kwargs.items():
            if key in self.tuning_cache:
                self.tuning_cache[key] = float(value)
        
        # Construir y enviar vector
        msg = Float32MultiArray()
        msg.data = [
            self.tuning_cache['kp_pos'], self.tuning_cache['ki_pos'],
            self.tuning_cache['kp_neg'], self.tuning_cache['ki_neg'],
            self.tuning_cache['max_safe'], self.tuning_cache['min_safe']
        ]
        self.pub_tuning.publish(msg)
        print(f"🔧 [Tuning] Parámetros actualizados: {kwargs}")

    def get_state(self):
        """Devuelve un diccionario con el estado actual del robot"""
        with self._telemetry_lock:
            return {
                'timestamp': time.time(),
                'pressure': self.pressure_kpa,
                'pwm_main': self.debug_vector[0],
                'pwm_aux': self.debug_vector[1],
                'error': self.debug_vector[2] / 100.0,
                'mode': self.debug_vector[3]
            }

    def close(self):
        self.stop()
        self.destroy_node()
        rclpy.shutdown()
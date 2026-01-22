"""
SoftBot Python Interface (SDK) v2.0
-----------------------------------
Wrapper optimizado para SoftBot Pneumatic Driver.
Incluye métodos de alto nivel para locomoción y control directo.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int8, Float32MultiArray, Int16MultiArray
import threading
import time
import csv

class SoftBot(Node):
    def __init__(self):
        super().__init__('softbot_client_api')
        
        # --- Configuración de Tópicos ---
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
        self.debug_vector = [0, 0, 0, 0]
        self._telemetry_lock = threading.Lock()
        self.tuning_cache = {
            'kp_pos': 15.0, 'ki_pos': 5.0,
            'kp_neg': 20.0, 'ki_neg': 10.0,
            'max_safe': 45.0, 'min_safe': -60.0
        }
        
        # Iniciar thread de escucha automática
        self._stop_event = threading.Event()
        self._spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._spin_thread.start()
        
        # Esperar conexión inicial (opcional)
        time.sleep(1.0)

    def _spin_loop(self):
        while not self._stop_event.is_set() and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    def _cb_feedback(self, msg):
        with self._telemetry_lock:
            self.pressure_kpa = msg.data

    def _cb_debug(self, msg):
        with self._telemetry_lock:
            if len(msg.data) >= 4:
                self.debug_vector = list(msg.data)

    # ==========================================================
    # MÉTODOS DE ALTO NIVEL (LOCOMOCIÓN)
    # ==========================================================

    def inflate(self, pressure_kpa):
        """Activa modo inflado (PID) con setpoint en kPa"""
        self.pub_mode.publish(Int8(data=1))
        self.pub_setpoint.publish(Float32(data=float(pressure_kpa)))

    def suction(self, pressure_kpa):
        """Activa modo succión (PID) con setpoint en kPa (negativo)"""
        self.pub_mode.publish(Int8(data=-1))
        self.pub_setpoint.publish(Float32(data=float(pressure_kpa)))

    def stop(self):
        """Detiene bombas y válvulas (Modo 0)"""
        self.pub_mode.publish(Int8(data=0))
        self.pub_setpoint.publish(Float32(data=0.0))

    def set_chamber(self, chamber_id):
        """Selecciona cámara activa (0=Todas, 1-4=Específica)"""
        self.pub_chamber.publish(Int8(data=int(chamber_id)))

    # ==========================================================
    # MÉTODOS DE BAJO NIVEL / CALIBRACIÓN
    # ==========================================================

    def set_pwm(self, pwm_val, mode):
        """Control directo PWM (Open Loop). Mode: 2=Inflar, -2=Succión"""
        self.pub_mode.publish(Int8(data=int(mode)))
        self.pub_setpoint.publish(Float32(data=float(pwm_val)))

    def get_state(self):
        """Retorna telemetría actual"""
        with self._telemetry_lock:
            return {
                'timestamp': time.time(),
                'pressure': self.pressure_kpa,
                'pwm_main': self.debug_vector[0],
                'pwm_aux': self.debug_vector[1],
                'logic_state': self.debug_vector[3]
            }

    def close(self):
        """Limpieza y cierre"""
        self.stop()
        self._stop_event.set()
        if self._spin_thread.is_alive():
            self._spin_thread.join(timeout=1.0)
        self.destroy_node()
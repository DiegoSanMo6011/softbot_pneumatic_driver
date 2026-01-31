#!/usr/bin/env python3
"""
SoftBot Python Interface (SDK) v2.1 (Fix: update_tuning added)
--------------------------------------------------------------
Wrapper optimizado para SoftBot Pneumatic Driver.
Incluye métodos de alto nivel para locomoción, control directo y sintonización.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int8, Float32MultiArray, Int16MultiArray
import threading
import time

class SoftBot(Node):
    def __init__(self):
        super().__init__('softbot_client_api')
        
        # --- Configuración de Tópicos ---
        self.TOPIC_CHAMBER = '/active_chamber'
        self.TOPIC_MODE = '/pressure_mode'
        self.TOPIC_SETPOINT = '/pressure_setpoint'
        self.TOPIC_TUNING = '/tuning_params'
        self.TOPIC_BOOST = '/boost_valve'
        self.TOPIC_TANK_STATE = '/tank_state'
        self.TOPIC_FEEDBACK = '/pressure_feedback'
        self.TOPIC_DEBUG = '/system_debug'

        # --- Publicadores ---
        self.pub_chamber = self.create_publisher(Int8, self.TOPIC_CHAMBER, 10)
        self.pub_mode = self.create_publisher(Int8, self.TOPIC_MODE, 10)
        self.pub_setpoint = self.create_publisher(Float32, self.TOPIC_SETPOINT, 10)
        self.pub_tuning = self.create_publisher(Float32MultiArray, self.TOPIC_TUNING, 10)
        self.pub_boost = self.create_publisher(Int8, self.TOPIC_BOOST, 10)
        
        # --- Suscriptores ---
        self.sub_feedback = self.create_subscription(Float32, self.TOPIC_FEEDBACK, self._cb_feedback, 10)
        self.sub_debug = self.create_subscription(Int16MultiArray, self.TOPIC_DEBUG, self._cb_debug, 10)
        self.sub_tank_state = self.create_subscription(Int8, self.TOPIC_TANK_STATE, self._cb_tank_state, 10)

        # --- Estado Interno ---
        self.pressure_kpa = 0.0
        self.debug_vector = [0, 0, 0, 0]
        self.tank_state = 0
        self._telemetry_lock = threading.Lock()
        
        # Cache de Tuning (Valores por defecto del firmware para sincronización)
        self.tuning_cache = {
            'kp_pos': 12.0, 'ki_pos': 300.0,
            'kp_neg': -75.0, 'ki_neg': -750.0,
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

    def _cb_tank_state(self, msg):
        with self._telemetry_lock:
            self.tank_state = int(msg.data)

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
        self.set_boost(False)

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

    def set_boost(self, enabled: bool):
        """Activa/desactiva la válvula de boost (tanque)"""
        self.pub_boost.publish(Int8(data=1 if enabled else 0))

    def pulse_boost(self, duration_s: float):
        """Pulso de boost por duración (segundos)"""
        self.set_boost(True)
        time.sleep(max(0.0, float(duration_s)))
        self.set_boost(False)

    def fill_tank(self, target_kpa: float):
        """Inicia llenado de tanque (modo 3) con setpoint en kPa"""
        self.pub_mode.publish(Int8(data=3))
        self.pub_setpoint.publish(Float32(data=float(target_kpa)))

    def vent(self, chamber_id: int = 3, duration_s: float | None = None):
        """Libera presión a atmósfera (modo 4)."""
        self.pub_chamber.publish(Int8(data=int(chamber_id)))
        self.pub_mode.publish(Int8(data=4))
        self.pub_setpoint.publish(Float32(data=0.0))
        if duration_s is not None:
            time.sleep(max(0.0, float(duration_s)))
            self.stop()

    def stop_fill(self):
        """Detiene llenado de tanque"""
        self.stop()

    def update_tuning(self, **kwargs):
        """
        Envía nuevos parámetros de control al firmware.
        Args opcionales: kp_pos, ki_pos, kp_neg, ki_neg, max_safe, min_safe
        """
        # 1. Actualizar memoria local con los valores nuevos
        for key, value in kwargs.items():
            if key in self.tuning_cache:
                self.tuning_cache[key] = float(value)
        
        # 2. Construir mensaje vectorial (El orden es CRÍTICO para el firmware)
        # Orden: [KP_POS, KI_POS, KP_NEG, KI_NEG, MAX_SAFE, MIN_SAFE]
        msg = Float32MultiArray()
        msg.data = [
            self.tuning_cache['kp_pos'], 
            self.tuning_cache['ki_pos'],
            self.tuning_cache['kp_neg'], 
            self.tuning_cache['ki_neg'],
            self.tuning_cache['max_safe'], 
            self.tuning_cache['min_safe']
        ]
        
        self.pub_tuning.publish(msg)
        # print(f"🔧 [API] Tuning enviado: {kwargs}") # Debug opcional

    def get_state(self):
        """Retorna telemetría actual"""
        with self._telemetry_lock:
            error_raw = self.debug_vector[2]
            return {
                'timestamp': time.time(),
                'pressure': self.pressure_kpa,
                'pwm_main': self.debug_vector[0],
                'pwm_aux': self.debug_vector[1],
                'logic_state': self.debug_vector[3],
                'error_raw': error_raw,
                'error': error_raw / 10.0,
                'tank_state': self.tank_state
            }

    def close(self):
        """Limpieza y cierre"""
        self.stop()
        self._stop_event.set()
        if self._spin_thread.is_alive():
            self._spin_thread.join(timeout=1.0)
        self.destroy_node()

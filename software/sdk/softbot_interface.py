#!/usr/bin/env python3
"""
SoftBot Python Interface (SDK).

Wrapper para SoftBot Pneumatic Driver con control de locomocion, tuning y
operaciones con turbo pre-PID.
"""

from __future__ import annotations

import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, Int8, Int16MultiArray

try:
    from .protocol import (
        CHAMBER_AB,
        CHAMBER_BLOCKED,
        MODE_PID_INFLATE,
        MODE_PID_INFLATE_TURBO,
        MODE_PID_SUCTION,
        MODE_PWM_INFLATE,
        MODE_PWM_SUCTION,
        MODE_STOP,
        MODE_TANK_FILL,
        MODE_VENT,
        VALID_CHAMBERS,
    )
except ImportError:
    from protocol import (  # type: ignore
        CHAMBER_AB,
        CHAMBER_BLOCKED,
        MODE_PID_INFLATE,
        MODE_PID_INFLATE_TURBO,
        MODE_PID_SUCTION,
        MODE_PWM_INFLATE,
        MODE_PWM_SUCTION,
        MODE_STOP,
        MODE_TANK_FILL,
        MODE_VENT,
        VALID_CHAMBERS,
    )


class SoftBot(Node):
    def __init__(self):
        super().__init__("softbot_client_api")

        # Topic names
        self.TOPIC_CHAMBER = "/active_chamber"
        self.TOPIC_MODE = "/pressure_mode"
        self.TOPIC_SETPOINT = "/pressure_setpoint"
        self.TOPIC_TUNING = "/tuning_params"
        self.TOPIC_BOOST = "/boost_valve"
        self.TOPIC_TANK_STATE = "/tank_state"
        self.TOPIC_FEEDBACK = "/pressure_feedback"
        self.TOPIC_DEBUG = "/system_debug"

        # Publishers
        self.pub_chamber = self.create_publisher(Int8, self.TOPIC_CHAMBER, 10)
        self.pub_mode = self.create_publisher(Int8, self.TOPIC_MODE, 10)
        self.pub_setpoint = self.create_publisher(Float32, self.TOPIC_SETPOINT, 10)
        self.pub_tuning = self.create_publisher(Float32MultiArray, self.TOPIC_TUNING, 10)
        self.pub_boost = self.create_publisher(Int8, self.TOPIC_BOOST, 10)

        # Subscribers
        self.sub_feedback = self.create_subscription(
            Float32,
            self.TOPIC_FEEDBACK,
            self._cb_feedback,
            10,
        )
        self.sub_debug = self.create_subscription(
            Int16MultiArray,
            self.TOPIC_DEBUG,
            self._cb_debug,
            10,
        )
        self.sub_tank_state = self.create_subscription(
            Int8,
            self.TOPIC_TANK_STATE,
            self._cb_tank_state,
            10,
        )

        # Telemetry state
        self.pressure_kpa = 0.0
        self.debug_vector = [0, 0, 0, 0]
        self.tank_state = 0
        self._telemetry_lock = threading.Lock()

        # Matches firmware defaults (softbot_controller.ino).
        self.tuning_cache = {
            "kp_pos": 24.0,
            "ki_pos": 1500.0,
            "kp_neg": -75.0,
            "ki_neg": -750.0,
            "max_safe": 55.0,
            "min_safe": -60.0,
        }

        # Background spin thread for async telemetry.
        self._stop_event = threading.Event()
        self._spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._spin_thread.start()

        # Optional short wait for first telemetry.
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

    def _publish_with_retries(
        self,
        publisher,
        msg,
        repeats: int = 3,
        interval_s: float = 0.03,
    ):
        repeats = max(1, int(repeats))
        interval_s = max(0.0, float(interval_s))
        for i in range(repeats):
            publisher.publish(msg)
            if i < repeats - 1:
                time.sleep(interval_s)

    def _send_command_reliable(
        self,
        chamber_id: int,
        mode: int,
        setpoint: float,
        repeats: int = 3,
        interval_s: float = 0.03,
    ):
        chamber_msg = Int8(data=int(chamber_id))
        mode_msg = Int8(data=int(mode))
        setpoint_msg = Float32(data=float(setpoint))

        repeats = max(1, int(repeats))
        interval_s = max(0.0, float(interval_s))
        for i in range(repeats):
            self.pub_chamber.publish(chamber_msg)
            self.pub_mode.publish(mode_msg)
            self.pub_setpoint.publish(setpoint_msg)
            if i < repeats - 1:
                time.sleep(interval_s)

    def _send_mode_setpoint_reliable(
        self,
        mode: int,
        setpoint: float,
        repeats: int = 3,
        interval_s: float = 0.03,
    ):
        mode_msg = Int8(data=int(mode))
        setpoint_msg = Float32(data=float(setpoint))
        self._publish_with_retries(self.pub_mode, mode_msg, repeats, interval_s)
        self._publish_with_retries(self.pub_setpoint, setpoint_msg, repeats, interval_s)

    def inflate(self, pressure_kpa):
        """Activate PID inflation with kPa setpoint."""
        self._send_mode_setpoint_reliable(MODE_PID_INFLATE, float(pressure_kpa))

    def inflate_turbo(self, pressure_kpa: float):
        """Activate turbo pre-PID inflation (mode 5)."""
        self._send_mode_setpoint_reliable(MODE_PID_INFLATE_TURBO, float(pressure_kpa))

    def suction(self, pressure_kpa):
        """Activate PID suction with a negative kPa setpoint."""
        self._send_mode_setpoint_reliable(MODE_PID_SUCTION, float(pressure_kpa))

    def stop(self):
        """Stop pumps and valves."""
        self._send_mode_setpoint_reliable(MODE_STOP, 0.0, repeats=2, interval_s=0.02)
        self.set_boost(False)

    def set_chamber(self, chamber_id: int):
        """Select active chamber (0=blocked, 1=A, 2=B, 3=A+B)."""
        chamber_id = int(chamber_id)
        if chamber_id not in VALID_CHAMBERS:
            raise ValueError(f"Invalid chamber {chamber_id}. Valid values: {VALID_CHAMBERS}")
        self._publish_with_retries(
            self.pub_chamber,
            Int8(data=chamber_id),
            repeats=2,
            interval_s=0.02,
        )

    def set_pwm(self, pwm_val, mode):
        """Direct open-loop PWM control. mode: 2 inflate, -2 suction."""
        mode = int(mode)
        if mode not in (MODE_PWM_INFLATE, MODE_PWM_SUCTION):
            raise ValueError(
                f"Invalid PWM mode {mode}. Use {MODE_PWM_INFLATE} or {MODE_PWM_SUCTION}."
            )
        self._send_mode_setpoint_reliable(mode, float(pwm_val))

    def set_boost(self, enabled: bool, repeats: int = 3, interval_s: float = 0.03):
        """Set boost valve state with retries."""
        msg = Int8(data=1 if enabled else 0)
        self._publish_with_retries(
            self.pub_boost,
            msg,
            repeats=repeats,
            interval_s=interval_s,
        )

    def pulse_boost(self, duration_s: float):
        """Trigger a timed boost pulse."""
        self.set_boost(True)
        time.sleep(max(0.0, float(duration_s)))
        self.set_boost(False)

    def fill_tank(self, target_kpa: float):
        """Start tank fill mode with a kPa setpoint."""
        self._send_command_reliable(CHAMBER_BLOCKED, MODE_TANK_FILL, float(target_kpa))

    def vent(self, chamber_id: int = CHAMBER_AB, duration_s: float | None = None):
        """Release pressure to atmosphere."""
        chamber_id = int(chamber_id)
        if chamber_id not in VALID_CHAMBERS:
            raise ValueError(f"Invalid chamber {chamber_id}. Valid values: {VALID_CHAMBERS}")
        self._send_command_reliable(chamber_id, MODE_VENT, 0.0)
        if duration_s is not None:
            time.sleep(max(0.0, float(duration_s)))
            self.stop()

    def stop_fill(self):
        """Stop tank fill mode."""
        self.stop()

    def update_tuning(self, **kwargs):
        """
        Send new control parameters to firmware.
        Optional keys: kp_pos, ki_pos, kp_neg, ki_neg, max_safe, min_safe.
        """
        for key, value in kwargs.items():
            if key in self.tuning_cache:
                self.tuning_cache[key] = float(value)

        msg = Float32MultiArray()
        msg.data = [
            self.tuning_cache["kp_pos"],
            self.tuning_cache["ki_pos"],
            self.tuning_cache["kp_neg"],
            self.tuning_cache["ki_neg"],
            self.tuning_cache["max_safe"],
            self.tuning_cache["min_safe"],
        ]
        self.pub_tuning.publish(msg)

    def get_state(self):
        """Return current telemetry snapshot."""
        with self._telemetry_lock:
            error_raw = self.debug_vector[2]
            return {
                "timestamp": time.time(),
                "pressure": self.pressure_kpa,
                "pwm_main": self.debug_vector[0],
                "pwm_aux": self.debug_vector[1],
                "logic_state": self.debug_vector[3],
                "error_raw": error_raw,
                "error": error_raw / 10.0,
                "tank_state": self.tank_state,
            }

    def close(self):
        """Cleanup and shutdown."""
        self.stop()
        self._stop_event.set()
        if self._spin_thread.is_alive():
            self._spin_thread.join(timeout=1.0)
        self.destroy_node()

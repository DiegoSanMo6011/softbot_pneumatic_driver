#!/usr/bin/env python3
"""
SoftBot Python Interface (SDK).

Wrapper para SoftBot Pneumatic Driver con protocolo atomico de precarga,
tuning y diagnostico de hardware.
"""

from __future__ import annotations

import threading
import time
from collections.abc import Iterable

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, Int8, Int16, Int16MultiArray

try:
    from .protocol import (
        CHAMBER_ABC,
        CHAMBER_BLOCKED,
        MODE_HARDWARE_DIAGNOSTIC,
        MODE_LABELS,
        MODE_PID_INFLATE,
        MODE_PID_SUCTION,
        MODE_PWM_INFLATE,
        MODE_PWM_SUCTION,
        MODE_STOP,
        MODE_VENT,
        PID_CONTROL_MODES,
        PNEUMATIC_BEHAVIOR_ARM,
        PNEUMATIC_BEHAVIOR_AUTO,
        PNEUMATIC_BEHAVIOR_DIRECT,
        PNEUMATIC_BEHAVIOR_FIRE,
        PNEUMATIC_BEHAVIOR_LABELS,
        PNEUMATIC_FLAG_ARMED,
        PNEUMATIC_FLAG_COMMAND_REJECTED,
        PNEUMATIC_FLAG_DELIVERING,
        PNEUMATIC_FLAG_EMERGENCY_STOP,
        PNEUMATIC_FLAG_LEGACY_INPUT,
        PNEUMATIC_FLAG_READY,
        PNEUMATIC_FLAG_TIMEOUT,
        PNEUMATIC_STATE_IDLE,
        PNEUMATIC_STATE_LABELS,
        VALID_CHAMBERS,
        PneumaticStateFrame,
        build_hardware_mask,
        build_pneumatic_command_payload,
        decode_pneumatic_state_payload,
    )
except ImportError:
    from protocol import (  # type: ignore
        CHAMBER_ABC,
        CHAMBER_BLOCKED,
        MODE_HARDWARE_DIAGNOSTIC,
        MODE_LABELS,
        MODE_PID_INFLATE,
        MODE_PID_SUCTION,
        MODE_PWM_INFLATE,
        MODE_PWM_SUCTION,
        MODE_STOP,
        MODE_VENT,
        PID_CONTROL_MODES,
        PNEUMATIC_BEHAVIOR_ARM,
        PNEUMATIC_BEHAVIOR_AUTO,
        PNEUMATIC_BEHAVIOR_DIRECT,
        PNEUMATIC_BEHAVIOR_FIRE,
        PNEUMATIC_BEHAVIOR_LABELS,
        PNEUMATIC_FLAG_ARMED,
        PNEUMATIC_FLAG_COMMAND_REJECTED,
        PNEUMATIC_FLAG_DELIVERING,
        PNEUMATIC_FLAG_EMERGENCY_STOP,
        PNEUMATIC_FLAG_LEGACY_INPUT,
        PNEUMATIC_FLAG_READY,
        PNEUMATIC_FLAG_TIMEOUT,
        PNEUMATIC_STATE_IDLE,
        PNEUMATIC_STATE_LABELS,
        VALID_CHAMBERS,
        PneumaticStateFrame,
        build_hardware_mask,
        build_pneumatic_command_payload,
        decode_pneumatic_state_payload,
    )


BEHAVIOR_NAME_TO_ID = {
    "direct": PNEUMATIC_BEHAVIOR_DIRECT,
    "auto": PNEUMATIC_BEHAVIOR_AUTO,
    "arm": PNEUMATIC_BEHAVIOR_ARM,
    "fire": PNEUMATIC_BEHAVIOR_FIRE,
}


class SoftBot(Node):
    def __init__(self):
        super().__init__("softbot_client_api")

        # Topic names.
        self.TOPIC_CHAMBER = "/active_chamber"
        self.TOPIC_MODE = "/pressure_mode"
        self.TOPIC_SETPOINT = "/pressure_setpoint"
        self.TOPIC_PNEUMATIC_COMMAND = "/pneumatic_command"
        self.TOPIC_PNEUMATIC_STATE = "/pneumatic_state"
        self.TOPIC_TUNING = "/tuning_params"
        self.TOPIC_HARDWARE_TEST = "/hardware_test"
        self.TOPIC_SENSOR_PRESSURE = "/sensor/pressure"
        self.TOPIC_SENSOR_VACUUM = "/sensor/vacuum"
        self.TOPIC_DEBUG = "/system_debug"

        # Publishers.
        self.pub_chamber = self.create_publisher(Int8, self.TOPIC_CHAMBER, 10)
        self.pub_mode = self.create_publisher(Int8, self.TOPIC_MODE, 10)
        self.pub_setpoint = self.create_publisher(Float32, self.TOPIC_SETPOINT, 10)
        self.pub_pneumatic_command = self.create_publisher(
            Int16MultiArray, self.TOPIC_PNEUMATIC_COMMAND, 10
        )
        self.pub_tuning = self.create_publisher(Float32MultiArray, self.TOPIC_TUNING, 10)
        self.pub_hwtest = self.create_publisher(Int16, self.TOPIC_HARDWARE_TEST, 10)

        # Subscribers.
        self.sub_sensor_pressure = self.create_subscription(
            Float32,
            self.TOPIC_SENSOR_PRESSURE,
            self._cb_sensor_pressure,
            10,
        )
        self.sub_sensor_vacuum = self.create_subscription(
            Float32,
            self.TOPIC_SENSOR_VACUUM,
            self._cb_sensor_vacuum,
            10,
        )
        self.sub_debug = self.create_subscription(
            Int16MultiArray,
            self.TOPIC_DEBUG,
            self._cb_debug,
            10,
        )
        self.sub_pneumatic_state = self.create_subscription(
            Int16MultiArray,
            self.TOPIC_PNEUMATIC_STATE,
            self._cb_pneumatic_state,
            10,
        )

        # Telemetry state.
        self.sensor_pressure_kpa = 0.0
        self.sensor_vacuum_kpa = 0.0
        self.debug_vector = [0, 0, 0, 0, 0, 0]
        self.pneumatic_state = PneumaticStateFrame(
            version=1,
            state=PNEUMATIC_STATE_IDLE,
            mode=MODE_STOP,
            behavior=PNEUMATIC_BEHAVIOR_DIRECT,
            requested_chamber_mask=CHAMBER_BLOCKED,
            applied_chamber_mask=CHAMBER_BLOCKED,
            flags=0,
            target_raw=0,
            source_pressure_raw=0,
            command_token=0,
        )
        self._telemetry_lock = threading.Lock()

        # Cached operator context for wrapper-style API.
        self._legacy_chamber_cache = CHAMBER_BLOCKED
        self._armed_mode_cache = MODE_PID_INFLATE

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

    def _cb_sensor_pressure(self, msg):
        with self._telemetry_lock:
            self.sensor_pressure_kpa = float(msg.data)

    def _cb_sensor_vacuum(self, msg):
        with self._telemetry_lock:
            self.sensor_vacuum_kpa = float(msg.data)

    def _cb_debug(self, msg):
        with self._telemetry_lock:
            if len(msg.data) >= 6:
                self.debug_vector = list(msg.data)

    def _cb_pneumatic_state(self, msg):
        with self._telemetry_lock:
            try:
                self.pneumatic_state = decode_pneumatic_state_payload(msg.data)
            except ValueError:
                return

    def _publish_with_retries(
        self,
        publisher,
        msg,
        repeats: int = 3,
        interval_s: float = 0.03,
    ):
        repeats = max(1, int(repeats))
        interval_s = max(0.0, float(interval_s))
        for idx in range(repeats):
            publisher.publish(msg)
            if idx < repeats - 1:
                time.sleep(interval_s)

    def _normalize_behavior(self, behavior: str | int) -> int:
        if isinstance(behavior, str):
            key = behavior.strip().lower()
            if key not in BEHAVIOR_NAME_TO_ID:
                valid = ", ".join(sorted(BEHAVIOR_NAME_TO_ID))
                raise ValueError(f"Invalid pneumatic behavior '{behavior}'. Valid: {valid}")
            return BEHAVIOR_NAME_TO_ID[key]
        return int(behavior)

    def send_pneumatic_command(
        self,
        *,
        mode: int,
        chamber_mask: int,
        target: float,
        behavior: str | int = PNEUMATIC_BEHAVIOR_AUTO,
        token: int = 0,
        repeats: int = 3,
        interval_s: float = 0.03,
    ) -> list[int]:
        behavior_id = self._normalize_behavior(behavior)
        payload = build_pneumatic_command_payload(
            mode=int(mode),
            chamber_mask=int(chamber_mask),
            behavior=behavior_id,
            target=float(target),
            token=int(token),
        )
        msg = Int16MultiArray()
        msg.data = payload
        self._publish_with_retries(self.pub_pneumatic_command, msg, repeats, interval_s)

        if behavior_id == PNEUMATIC_BEHAVIOR_ARM and int(mode) in PID_CONTROL_MODES:
            self._armed_mode_cache = int(mode)
        return payload

    def direct_command(
        self,
        *,
        mode: int,
        chamber_mask: int,
        target: float,
        token: int = 0,
        repeats: int = 3,
        interval_s: float = 0.03,
    ) -> list[int]:
        return self.send_pneumatic_command(
            mode=mode,
            chamber_mask=chamber_mask,
            target=target,
            behavior=PNEUMATIC_BEHAVIOR_DIRECT,
            token=token,
            repeats=repeats,
            interval_s=interval_s,
        )

    def arm_inflate(
        self,
        pressure_kpa: float,
        *,
        chamber_mask: int | None = None,
        token: int = 0,
    ) -> list[int]:
        chamber = self._legacy_chamber_cache if chamber_mask is None else int(chamber_mask)
        return self.send_pneumatic_command(
            mode=MODE_PID_INFLATE,
            chamber_mask=chamber,
            target=abs(float(pressure_kpa)),
            behavior=PNEUMATIC_BEHAVIOR_ARM,
            token=token,
        )

    def arm_suction(
        self,
        pressure_kpa: float,
        *,
        chamber_mask: int | None = None,
        token: int = 0,
    ) -> list[int]:
        chamber = self._legacy_chamber_cache if chamber_mask is None else int(chamber_mask)
        return self.send_pneumatic_command(
            mode=MODE_PID_SUCTION,
            chamber_mask=chamber,
            target=-abs(float(pressure_kpa)),
            behavior=PNEUMATIC_BEHAVIOR_ARM,
            token=token,
        )

    def fire(
        self,
        *,
        chamber_mask: int = CHAMBER_BLOCKED,
        token: int = 0,
        mode: int | None = None,
    ) -> list[int]:
        fire_mode = self._armed_mode_cache if mode is None else int(mode)
        return self.send_pneumatic_command(
            mode=fire_mode,
            chamber_mask=int(chamber_mask),
            target=0.0,
            behavior=PNEUMATIC_BEHAVIOR_FIRE,
            token=token,
        )

    def inflate(self, pressure_kpa: float):
        """Activate PID inflation using the cached chamber mask and AUTO behavior."""
        return self.send_pneumatic_command(
            mode=MODE_PID_INFLATE,
            chamber_mask=self._legacy_chamber_cache,
            target=abs(float(pressure_kpa)),
            behavior=PNEUMATIC_BEHAVIOR_AUTO,
        )

    def suction(self, pressure_kpa: float):
        """Activate PID suction using the cached chamber mask and AUTO behavior."""
        return self.send_pneumatic_command(
            mode=MODE_PID_SUCTION,
            chamber_mask=self._legacy_chamber_cache,
            target=-abs(float(pressure_kpa)),
            behavior=PNEUMATIC_BEHAVIOR_AUTO,
        )

    def stop(self):
        """Stop pumps and valves through the atomic command topic."""
        return self.send_pneumatic_command(
            mode=MODE_STOP,
            chamber_mask=CHAMBER_BLOCKED,
            target=0.0,
            behavior=PNEUMATIC_BEHAVIOR_DIRECT,
            repeats=2,
            interval_s=0.02,
        )

    def set_chamber(self, chamber_id: int):
        """Cache and publish the legacy chamber bitmask."""
        chamber_id = int(chamber_id)
        if chamber_id not in VALID_CHAMBERS:
            raise ValueError(f"Invalid chamber {chamber_id}. Valid values: {VALID_CHAMBERS}")
        self._legacy_chamber_cache = chamber_id
        self._publish_with_retries(
            self.pub_chamber,
            Int8(data=chamber_id),
            repeats=2,
            interval_s=0.02,
        )

    def set_pwm(self, pwm_val, mode):
        """Direct open-loop PWM control on the cached chamber mask."""
        mode = int(mode)
        if mode not in (MODE_PWM_INFLATE, MODE_PWM_SUCTION):
            raise ValueError(
                f"Invalid PWM mode {mode}. Use {MODE_PWM_INFLATE} or {MODE_PWM_SUCTION}."
            )
        return self.direct_command(
            mode=mode,
            chamber_mask=self._legacy_chamber_cache,
            target=float(pwm_val),
        )

    def set_hardware_test(
        self,
        bitmask: int,
        pwm: int = 120,
        repeats: int = 3,
        interval_s: float = 0.03,
    ):
        """
        Enter hardware diagnostic mode and apply output bitmask.

        bitmask bits:
          0 inflate_main, 1 inflate_aux, 2 suction_main, 3 suction_aux,
          4 valve_inflate, 5 valve_suction, 6 valve_chamber_c,
          7 mux_A, 8 mux_B.
        """
        pwm = int(max(0, min(255, int(pwm))))
        repeats = max(1, int(repeats))
        interval_s = max(0.0, float(interval_s))

        mode_msg = Int8(data=MODE_HARDWARE_DIAGNOSTIC)
        setpoint_msg = Float32(data=float(pwm))
        mask_msg = Int16(data=int(bitmask))

        for idx in range(repeats):
            self.pub_mode.publish(mode_msg)
            self.pub_setpoint.publish(setpoint_msg)
            self.pub_hwtest.publish(mask_msg)
            if idx < repeats - 1:
                time.sleep(interval_s)

    def set_hardware_components(
        self,
        component_ids: Iterable[str],
        pwm: int = 120,
        repeats: int = 3,
        interval_s: float = 0.03,
    ) -> int:
        """Apply hardware diagnostics from declarative component ids."""
        mask = build_hardware_mask(component_ids=component_ids)
        self.set_hardware_test(bitmask=mask, pwm=pwm, repeats=repeats, interval_s=interval_s)
        return mask

    def set_hardware_groups(
        self,
        pressure_on: bool,
        vacuum_on: bool,
        valves: dict[str, bool] | None = None,
        mux: dict[str, bool] | None = None,
        pwm: int = 120,
        repeats: int = 3,
        interval_s: float = 0.03,
    ) -> int:
        """
        Apply grouped hardware diagnostics.

        `pressure_on` and `vacuum_on` enable pump groups.
        `valves` accepts keys: inflate, suction, chamber_c or full ids (valve_*).
        `mux` accepts keys: a, b or full ids (mux_*).
        """
        groups: list[str] = []
        if pressure_on:
            groups.append("pressure")
        if vacuum_on:
            groups.append("vacuum")

        valve_map = {
            "inflate": "valve_inflate",
            "suction": "valve_suction",
            "chamber_c": "valve_chamber_c",
            "valve_inflate": "valve_inflate",
            "valve_suction": "valve_suction",
            "valve_chamber_c": "valve_chamber_c",
        }
        mux_map = {
            "a": "mux_a",
            "b": "mux_b",
            "mux_a": "mux_a",
            "mux_b": "mux_b",
        }

        component_ids: list[str] = []
        for raw_key, raw_value in (valves or {}).items():
            if not bool(raw_value):
                continue
            key = str(raw_key).strip().lower()
            if key not in valve_map:
                valid = ", ".join(sorted(valve_map))
                raise ValueError(f"Unknown valve key '{raw_key}'. Valid: {valid}")
            component_ids.append(valve_map[key])

        for raw_key, raw_value in (mux or {}).items():
            if not bool(raw_value):
                continue
            key = str(raw_key).strip().lower()
            if key not in mux_map:
                valid = ", ".join(sorted(mux_map))
                raise ValueError(f"Unknown mux key '{raw_key}'. Valid: {valid}")
            component_ids.append(mux_map[key])

        mask = build_hardware_mask(component_ids=component_ids, groups=groups)
        self.set_hardware_test(bitmask=mask, pwm=pwm, repeats=repeats, interval_s=interval_s)
        return mask

    def stop_hardware_test(self):
        """Exit hardware diagnostic mode and stop all outputs."""
        self.set_hardware_test(0, pwm=0, repeats=2, interval_s=0.02)
        self.stop()

    def vent(self, chamber_id: int = CHAMBER_ABC, duration_s: float | None = None):
        """Release pressure to atmosphere on the requested chamber bitmask."""
        chamber_id = int(chamber_id)
        if chamber_id not in VALID_CHAMBERS:
            raise ValueError(f"Invalid chamber {chamber_id}. Valid values: {VALID_CHAMBERS}")
        self.send_pneumatic_command(
            mode=MODE_VENT,
            chamber_mask=chamber_id,
            target=0.0,
            behavior=PNEUMATIC_BEHAVIOR_DIRECT,
        )
        if duration_s is not None:
            time.sleep(max(0.0, float(duration_s)))
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
            frame = self.pneumatic_state
            logic_state = int(self.debug_vector[4])
            if logic_state == 0 and frame.mode != MODE_STOP:
                logic_state = int(frame.mode)
            status_flags = int(self.debug_vector[5])
            if logic_state in (MODE_PID_SUCTION, MODE_PWM_SUCTION):
                control_pressure = float(self.sensor_vacuum_kpa)
            else:
                control_pressure = float(self.sensor_pressure_kpa)

            pneumatic_flags = int(frame.flags)
            return {
                "timestamp": time.time(),
                "sensor_pressure_kpa": float(self.sensor_pressure_kpa),
                "sensor_vacuum_kpa": float(self.sensor_vacuum_kpa),
                "control_pressure_kpa": control_pressure,
                "source_pressure_kpa": float(frame.source_pressure_kpa),
                "pwm_main": int(self.debug_vector[0]),
                "pwm_aux": int(self.debug_vector[1]),
                "logic_state": logic_state,
                "logic_state_label": MODE_LABELS.get(logic_state, str(logic_state)),
                "status_flags": status_flags,
                "pneumatic_state": int(frame.state),
                "pneumatic_state_label": frame.state_label,
                "pneumatic_behavior": int(frame.behavior),
                "pneumatic_behavior_label": frame.behavior_label,
                "requested_chamber_mask": int(frame.requested_chamber_mask),
                "applied_chamber_mask": int(frame.applied_chamber_mask),
                "pneumatic_flags": pneumatic_flags,
                "pneumatic_target": float(frame.target_value),
                "command_token": int(frame.command_token),
                "flag_ready": bool(pneumatic_flags & PNEUMATIC_FLAG_READY),
                "flag_armed": bool(pneumatic_flags & PNEUMATIC_FLAG_ARMED),
                "flag_delivering": bool(pneumatic_flags & PNEUMATIC_FLAG_DELIVERING),
                "flag_timeout": bool(pneumatic_flags & PNEUMATIC_FLAG_TIMEOUT),
                "flag_command_rejected": bool(pneumatic_flags & PNEUMATIC_FLAG_COMMAND_REJECTED),
                "flag_emergency_stop": bool(pneumatic_flags & PNEUMATIC_FLAG_EMERGENCY_STOP),
                "flag_legacy_input": bool(pneumatic_flags & PNEUMATIC_FLAG_LEGACY_INPUT),
                "debug_mode_label": MODE_LABELS.get(
                    int(self.debug_vector[4]), str(self.debug_vector[4])
                ),
                "behavior_label": PNEUMATIC_BEHAVIOR_LABELS.get(
                    int(frame.behavior), str(frame.behavior)
                ),
                "state_label": PNEUMATIC_STATE_LABELS.get(int(frame.state), str(frame.state)),
            }

    def close(self):
        """Cleanup and shutdown."""
        try:
            self.stop()
        except Exception:
            pass
        self._stop_event.set()
        if self._spin_thread.is_alive():
            self._spin_thread.join(timeout=1.0)
        self.destroy_node()

#!/usr/bin/env python3
"""Dedicated MOSFET diagnostics GUI for SoftBot pneumatic hardware."""

from __future__ import annotations

import csv
import os
import sys
import time
from dataclasses import dataclass
from datetime import datetime

try:
    from PySide6 import QtCore, QtWidgets
except Exception:
    from PyQt5 import QtCore, QtWidgets

import rclpy

BASE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
SOFTWARE_DIR = os.path.join(BASE_DIR, "software")
if SOFTWARE_DIR not in sys.path:
    sys.path.append(SOFTWARE_DIR)

from sdk.protocol import (  # noqa: E402
    CHAMBER_BLOCKED,
    HARDWARE_PUMP_GROUPS,
    build_hardware_mask,
    decode_hardware_mask,
)
from sdk.softbot_interface import SoftBot  # noqa: E402

LOG_DIR = os.path.join(BASE_DIR, "experiments", "logs", "hardware_diag")
AUTO_OFF_TIMEOUT_S = 10.0

PRESSURE_COMPONENTS = set(HARDWARE_PUMP_GROUPS["pressure"])
VACUUM_COMPONENTS = set(HARDWARE_PUMP_GROUPS["vacuum"])

CARD_COMPONENT_ORDER = (
    "inflate_main",
    "inflate_aux",
    "suction_main",
    "suction_aux",
    "valve_inflate",
    "valve_suction",
    "valve_boost",
    "mux_a",
    "mux_b",
)

COMPONENT_LABELS = {
    "inflate_main": "Pump Inflate Main",
    "inflate_aux": "Pump Inflate Aux",
    "suction_main": "Pump Suction Main",
    "suction_aux": "Pump Suction Aux",
    "valve_inflate": "Valve Inflate",
    "valve_suction": "Valve Suction",
    "valve_boost": "Valve Boost",
    "mux_a": "Mux Chamber A",
    "mux_b": "Mux Chamber B",
}


@dataclass(frozen=True)
class ScenarioStep:
    step_id: str
    label: str
    instruction: str
    groups: tuple[str, ...] = ()
    components: tuple[str, ...] = ()


SCENARIO_STEPS = (
    ScenarioStep(
        step_id="pressure_group",
        label="Bombas de Presión",
        instruction=(
            "Verifica que se enciendan los LEDs MOSFET de inflado y que ambas bombas "
            "de presión respondan."
        ),
        groups=("pressure",),
    ),
    ScenarioStep(
        step_id="vacuum_group",
        label="Bombas de Vacío",
        instruction=(
            "Verifica que se enciendan los LEDs MOSFET de succión y que ambas bombas "
            "de vacío respondan."
        ),
        groups=("vacuum",),
    ),
    ScenarioStep(
        step_id="valve_inflate",
        label="Válvula Inflate",
        instruction="Verifica LED MOSFET y actuación de la válvula de inflado.",
        components=("valve_inflate",),
    ),
    ScenarioStep(
        step_id="valve_suction",
        label="Válvula Suction",
        instruction="Verifica LED MOSFET y actuación de la válvula de succión.",
        components=("valve_suction",),
    ),
    ScenarioStep(
        step_id="valve_boost",
        label="Válvula Boost",
        instruction="Verifica LED MOSFET y actuación de la válvula boost.",
        components=("valve_boost",),
    ),
    ScenarioStep(
        step_id="mux_a",
        label="MUX A",
        instruction="Verifica LED MOSFET y señal de selección del MUX A.",
        components=("mux_a",),
    ),
    ScenarioStep(
        step_id="mux_b",
        label="MUX B",
        instruction="Verifica LED MOSFET y señal de selección del MUX B.",
        components=("mux_b",),
    ),
    ScenarioStep(
        step_id="all_off",
        label="Todo OFF",
        instruction="Verifica que todos los LEDs MOSFET queden apagados.",
    ),
)


class LedCard(QtWidgets.QFrame):
    """Visual card with LED-like indicator."""

    def __init__(self, title: str):
        super().__init__()
        self.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.setStyleSheet(
            """
            QFrame {
                border: 1px solid #9aa0a6;
                border-radius: 10px;
                background: #f7f9fc;
            }
            """
        )

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(12, 10, 12, 10)

        self.title_label = QtWidgets.QLabel(title)
        self.title_label.setWordWrap(True)
        self.state_label = QtWidgets.QLabel("OFF")
        self.state_label.setAlignment(QtCore.Qt.AlignCenter)
        self.state_label.setStyleSheet(
            """
            QLabel {
                border-radius: 14px;
                font-weight: 700;
                color: #5f6368;
                background: #ffd7d7;
                padding: 6px;
            }
            """
        )

        layout.addWidget(self.title_label)
        layout.addStretch(1)
        layout.addWidget(self.state_label)

    def set_active(self, enabled: bool):
        if enabled:
            self.state_label.setText("ON")
            self.state_label.setStyleSheet(
                """
                QLabel {
                    border-radius: 14px;
                    font-weight: 700;
                    color: #0f5132;
                    background: #c8f7dc;
                    padding: 6px;
                }
                """
            )
            return

        self.state_label.setText("OFF")
        self.state_label.setStyleSheet(
            """
            QLabel {
                border-radius: 14px;
                font-weight: 700;
                color: #5f6368;
                background: #ffd7d7;
                padding: 6px;
            }
            """
        )


class HardwareMosfetGUI(QtWidgets.QMainWindow):
    def __init__(self, bot: SoftBot):
        super().__init__()
        self.bot = bot
        self.current_mask = 0
        self.last_activity = time.monotonic()
        self.scenario_index = -1
        self.scenario_marks: dict[str, str] = {}

        self.log_file = None
        self.log_writer = None
        self._open_log()

        self.setWindowTitle("SoftBot MOSFET Hardware Diagnostics")
        self.resize(1280, 760)
        self._build_ui()
        self._wire_activity_signals()
        self._sync_ui_from_mask(0)
        self._update_led_cards(set())
        self._update_watchdog_label(AUTO_OFF_TIMEOUT_S)
        self._set_scenario_status("Escenario no iniciado.")

        self.watchdog_timer = QtCore.QTimer()
        self.watchdog_timer.timeout.connect(self._on_watchdog_tick)
        self.watchdog_timer.start(200)

        self._log_event("session_start", notes="hardware_mosfet_gui_opened")

    def _open_log(self):
        os.makedirs(LOG_DIR, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        log_path = os.path.join(LOG_DIR, f"mosfet_diag_{stamp}.csv")
        self.log_file = open(log_path, "w", newline="", encoding="utf-8")
        self.log_writer = csv.writer(self.log_file)
        self.log_writer.writerow(
            [
                "timestamp",
                "event",
                "pwm",
                "mask",
                "components",
                "pressure_group",
                "vacuum_group",
                "valve_inflate",
                "valve_suction",
                "valve_boost",
                "mux_a",
                "mux_b",
                "scenario_step",
                "scenario_result",
                "notes",
            ]
        )
        self.log_file.flush()

    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QHBoxLayout(central)

        left_panel = QtWidgets.QVBoxLayout()
        right_panel = QtWidgets.QVBoxLayout()

        left_panel.addWidget(self._build_controls_group())
        left_panel.addWidget(self._build_watchdog_group())
        left_panel.addWidget(self._build_scenario_group())
        left_panel.addStretch(1)

        right_panel.addWidget(self._build_led_panel())

        layout.addLayout(left_panel, 2)
        layout.addLayout(right_panel, 3)

    def _build_controls_group(self) -> QtWidgets.QGroupBox:
        box = QtWidgets.QGroupBox("Control Manual")
        grid = QtWidgets.QGridLayout(box)

        self.cb_pressure_group = QtWidgets.QCheckBox("Bombas de Presión (juntas)")
        self.cb_vacuum_group = QtWidgets.QCheckBox("Bombas de Vacío (juntas)")
        self.cb_valve_inflate = QtWidgets.QCheckBox("Valve Inflate")
        self.cb_valve_suction = QtWidgets.QCheckBox("Valve Suction")
        self.cb_valve_boost = QtWidgets.QCheckBox("Valve Boost")
        self.cb_mux_a = QtWidgets.QCheckBox("Mux A")
        self.cb_mux_b = QtWidgets.QCheckBox("Mux B")

        self.spin_pwm = QtWidgets.QSpinBox()
        self.spin_pwm.setRange(0, 255)
        self.spin_pwm.setValue(120)

        self.btn_apply = QtWidgets.QPushButton("Aplicar")
        self.btn_all_off = QtWidgets.QPushButton("Todo OFF")
        self.btn_estop = QtWidgets.QPushButton("E-STOP diagnóstico")

        self.mask_label = QtWidgets.QLabel("Mask: 0")
        self.mask_label.setStyleSheet("font-family: monospace;")

        self.btn_apply.clicked.connect(self.on_apply_manual)
        self.btn_all_off.clicked.connect(self.on_all_off)
        self.btn_estop.clicked.connect(self.on_estop)

        grid.addWidget(self.cb_pressure_group, 0, 0, 1, 2)
        grid.addWidget(self.cb_vacuum_group, 1, 0, 1, 2)
        grid.addWidget(self.cb_valve_inflate, 2, 0)
        grid.addWidget(self.cb_valve_suction, 2, 1)
        grid.addWidget(self.cb_valve_boost, 3, 0)
        grid.addWidget(self.cb_mux_a, 3, 1)
        grid.addWidget(self.cb_mux_b, 4, 0)
        grid.addWidget(QtWidgets.QLabel("PWM bombas"), 4, 1)
        grid.addWidget(self.spin_pwm, 4, 2)
        grid.addWidget(self.btn_apply, 5, 0)
        grid.addWidget(self.btn_all_off, 5, 1)
        grid.addWidget(self.btn_estop, 5, 2)
        grid.addWidget(self.mask_label, 6, 0, 1, 3)
        return box

    def _build_watchdog_group(self) -> QtWidgets.QGroupBox:
        box = QtWidgets.QGroupBox("Seguridad")
        layout = QtWidgets.QVBoxLayout(box)
        self.watchdog_label = QtWidgets.QLabel()
        self.watchdog_label.setWordWrap(True)
        layout.addWidget(self.watchdog_label)
        return box

    def _build_scenario_group(self) -> QtWidgets.QGroupBox:
        box = QtWidgets.QGroupBox("Pre-Competition MOSFET Check v1")
        layout = QtWidgets.QVBoxLayout(box)

        self.scenario_step_label = QtWidgets.QLabel("Paso: -")
        self.scenario_instruction_label = QtWidgets.QLabel("Instrucción: -")
        self.scenario_instruction_label.setWordWrap(True)
        self.scenario_status_label = QtWidgets.QLabel()
        self.scenario_status_label.setWordWrap(True)

        self.btn_scenario_start = QtWidgets.QPushButton("Iniciar escenario")
        self.btn_scenario_prev = QtWidgets.QPushButton("Paso anterior")
        self.btn_scenario_next = QtWidgets.QPushButton("Paso siguiente")
        self.btn_scenario_pass = QtWidgets.QPushButton("Marcar PASS")
        self.btn_scenario_fail = QtWidgets.QPushButton("Marcar FAIL")

        self.btn_scenario_start.clicked.connect(self.on_scenario_start)
        self.btn_scenario_prev.clicked.connect(self.on_scenario_prev)
        self.btn_scenario_next.clicked.connect(self.on_scenario_next)
        self.btn_scenario_pass.clicked.connect(self.on_scenario_pass)
        self.btn_scenario_fail.clicked.connect(self.on_scenario_fail)

        layout.addWidget(self.scenario_step_label)
        layout.addWidget(self.scenario_instruction_label)
        layout.addWidget(self.scenario_status_label)
        layout.addWidget(self.btn_scenario_start)
        layout.addWidget(self.btn_scenario_prev)
        layout.addWidget(self.btn_scenario_next)
        layout.addWidget(self.btn_scenario_pass)
        layout.addWidget(self.btn_scenario_fail)
        return box

    def _build_led_panel(self) -> QtWidgets.QGroupBox:
        box = QtWidgets.QGroupBox("Indicadores MOSFET")
        grid = QtWidgets.QGridLayout(box)
        self.cards: dict[str, LedCard] = {}

        for idx, component_id in enumerate(CARD_COMPONENT_ORDER):
            row = idx // 3
            col = idx % 3
            card = LedCard(COMPONENT_LABELS[component_id])
            self.cards[component_id] = card
            grid.addWidget(card, row, col)

        return box

    def _wire_activity_signals(self):
        widgets = [
            self.cb_pressure_group,
            self.cb_vacuum_group,
            self.cb_valve_inflate,
            self.cb_valve_suction,
            self.cb_valve_boost,
            self.cb_mux_a,
            self.cb_mux_b,
        ]
        for widget in widgets:
            widget.toggled.connect(self._touch_activity)
        self.spin_pwm.valueChanged.connect(self._touch_activity)

    def _touch_activity(self, *_args):
        self.last_activity = time.monotonic()

    def _manual_state(self) -> dict[str, bool]:
        return {
            "pressure": self.cb_pressure_group.isChecked(),
            "vacuum": self.cb_vacuum_group.isChecked(),
            "valve_inflate": self.cb_valve_inflate.isChecked(),
            "valve_suction": self.cb_valve_suction.isChecked(),
            "valve_boost": self.cb_valve_boost.isChecked(),
            "mux_a": self.cb_mux_a.isChecked(),
            "mux_b": self.cb_mux_b.isChecked(),
        }

    def _sync_ui_from_mask(self, mask: int):
        active = set(decode_hardware_mask(mask))
        pressure_on = PRESSURE_COMPONENTS.issubset(active)
        vacuum_on = VACUUM_COMPONENTS.issubset(active)

        widgets = [
            self.cb_pressure_group,
            self.cb_vacuum_group,
            self.cb_valve_inflate,
            self.cb_valve_suction,
            self.cb_valve_boost,
            self.cb_mux_a,
            self.cb_mux_b,
        ]
        for widget in widgets:
            widget.blockSignals(True)

        self.cb_pressure_group.setChecked(pressure_on)
        self.cb_vacuum_group.setChecked(vacuum_on)
        self.cb_valve_inflate.setChecked("valve_inflate" in active)
        self.cb_valve_suction.setChecked("valve_suction" in active)
        self.cb_valve_boost.setChecked("valve_boost" in active)
        self.cb_mux_a.setChecked("mux_a" in active)
        self.cb_mux_b.setChecked("mux_b" in active)

        for widget in widgets:
            widget.blockSignals(False)

        self.mask_label.setText(f"Mask: {mask}")

    def _update_led_cards(self, active_components: set[str]):
        for component_id, card in self.cards.items():
            card.set_active(component_id in active_components)

    def _apply_mask(self, mask: int, event_name: str, notes: str = ""):
        pwm = int(self.spin_pwm.value())
        self.bot.set_hardware_test(bitmask=mask, pwm=pwm)
        self.current_mask = int(mask)
        active = set(decode_hardware_mask(self.current_mask))
        self._update_led_cards(active)
        self._sync_ui_from_mask(self.current_mask)
        self._touch_activity()
        self._log_event(
            event_name,
            mask=self.current_mask,
            pwm=pwm,
            scenario_step=self._current_scenario_step_id(),
            notes=notes,
        )

    def _turn_all_off(self, event_name: str, notes: str = ""):
        self.bot.stop_hardware_test()
        self.current_mask = 0
        self._sync_ui_from_mask(0)
        self._update_led_cards(set())
        self._touch_activity()
        self._log_event(
            event_name,
            mask=0,
            pwm=0,
            scenario_step=self._current_scenario_step_id(),
            notes=notes,
        )

    def _log_event(
        self,
        event: str,
        mask: int | None = None,
        pwm: int | None = None,
        scenario_step: str = "",
        scenario_result: str = "",
        notes: str = "",
    ):
        if not self.log_writer:
            return

        if mask is None:
            mask = int(self.current_mask)
        if pwm is None:
            pwm = int(self.spin_pwm.value()) if hasattr(self, "spin_pwm") else 0

        active = set(decode_hardware_mask(mask))
        self.log_writer.writerow(
            [
                datetime.now().isoformat(timespec="seconds"),
                event,
                pwm,
                int(mask),
                ",".join(sorted(active)),
                int(PRESSURE_COMPONENTS.issubset(active)),
                int(VACUUM_COMPONENTS.issubset(active)),
                int("valve_inflate" in active),
                int("valve_suction" in active),
                int("valve_boost" in active),
                int("mux_a" in active),
                int("mux_b" in active),
                scenario_step,
                scenario_result,
                notes,
            ]
        )
        self.log_file.flush()

    def _current_scenario_step(self) -> ScenarioStep | None:
        if self.scenario_index < 0 or self.scenario_index >= len(SCENARIO_STEPS):
            return None
        return SCENARIO_STEPS[self.scenario_index]

    def _current_scenario_step_id(self) -> str:
        step = self._current_scenario_step()
        return step.step_id if step else ""

    def _set_scenario_status(self, text: str):
        self.scenario_status_label.setText(f"Estado escenario: {text}")

    def _apply_scenario_step(self):
        step = self._current_scenario_step()
        if not step:
            return
        mask = build_hardware_mask(component_ids=step.components, groups=step.groups)
        self._apply_mask(mask=mask, event_name="scenario_step_apply", notes=step.step_id)
        self.scenario_step_label.setText(
            f"Paso: {self.scenario_index + 1}/{len(SCENARIO_STEPS)} - {step.label}"
        )
        self.scenario_instruction_label.setText(f"Instrucción: {step.instruction}")
        self._set_scenario_status(f"Activo ({step.step_id})")

    def _record_scenario_mark(self, result: str):
        step = self._current_scenario_step()
        if not step:
            QtWidgets.QMessageBox.warning(
                self,
                "Escenario no iniciado",
                "Inicia el escenario antes de registrar PASS/FAIL.",
            )
            return
        self.scenario_marks[step.step_id] = result
        self._set_scenario_status(f"{step.step_id}: {result}")
        self._log_event(
            event="scenario_mark",
            mask=self.current_mask,
            pwm=int(self.spin_pwm.value()),
            scenario_step=step.step_id,
            scenario_result=result,
        )

    def _update_watchdog_label(self, remaining_s: float):
        if self.current_mask == 0:
            self.watchdog_label.setText(
                f"Auto-OFF: {AUTO_OFF_TIMEOUT_S:.0f}s (salidas actualmente en OFF)."
            )
            return
        self.watchdog_label.setText(f"Auto-OFF en {max(0.0, remaining_s):.1f}s sin interacción.")

    def _on_watchdog_tick(self):
        elapsed = time.monotonic() - self.last_activity
        remaining = AUTO_OFF_TIMEOUT_S - elapsed
        if self.current_mask != 0 and remaining <= 0.0:
            self._turn_all_off("auto_off", notes="watchdog_timeout")
            remaining = AUTO_OFF_TIMEOUT_S
        self._update_watchdog_label(remaining)

    def on_apply_manual(self):
        state = self._manual_state()
        mask = self.bot.set_hardware_groups(
            pressure_on=state["pressure"],
            vacuum_on=state["vacuum"],
            valves={
                "inflate": state["valve_inflate"],
                "suction": state["valve_suction"],
                "boost": state["valve_boost"],
            },
            mux={
                "a": state["mux_a"],
                "b": state["mux_b"],
            },
            pwm=int(self.spin_pwm.value()),
        )
        self.current_mask = int(mask)
        active = set(decode_hardware_mask(self.current_mask))
        self._update_led_cards(active)
        self._sync_ui_from_mask(self.current_mask)
        self._touch_activity()
        self._log_event(
            event="manual_apply",
            mask=self.current_mask,
            pwm=int(self.spin_pwm.value()),
            scenario_step=self._current_scenario_step_id(),
        )

    def on_all_off(self):
        self._turn_all_off(event_name="manual_all_off")

    def on_estop(self):
        self.bot.set_chamber(CHAMBER_BLOCKED)
        self._turn_all_off(event_name="diagnostic_estop")

    def on_scenario_start(self):
        self.scenario_index = 0
        self.scenario_marks = {}
        self._set_scenario_status("Iniciado")
        self._log_event(
            event="scenario_start",
            mask=self.current_mask,
            pwm=int(self.spin_pwm.value()),
            scenario_step=self._current_scenario_step_id(),
        )
        self._apply_scenario_step()

    def on_scenario_prev(self):
        if self.scenario_index <= 0:
            return
        self.scenario_index -= 1
        self._apply_scenario_step()

    def on_scenario_next(self):
        if self.scenario_index < 0:
            QtWidgets.QMessageBox.information(
                self,
                "Escenario",
                "Inicia el escenario para avanzar por los pasos.",
            )
            return
        if self.scenario_index >= len(SCENARIO_STEPS) - 1:
            self._set_scenario_status("Escenario completado.")
            return
        self.scenario_index += 1
        self._apply_scenario_step()

    def on_scenario_pass(self):
        self._record_scenario_mark("PASS")

    def on_scenario_fail(self):
        self._record_scenario_mark("FAIL")

    def closeEvent(self, event):
        try:
            self._turn_all_off(event_name="session_close")
        except Exception:
            pass

        try:
            if self.log_file:
                self.log_file.close()
        except Exception:
            pass

        try:
            self.bot.close()
        except Exception:
            pass

        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass
        event.accept()


def main():
    rclpy.init()
    bot = SoftBot()
    app = QtWidgets.QApplication(sys.argv)
    window = HardwareMosfetGUI(bot)
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()

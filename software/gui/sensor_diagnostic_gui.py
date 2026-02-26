#!/usr/bin/env python3
"""Dual-sensor diagnostic GUI for SoftBot pneumatic hardware.

Shows live readings from both ADS1115 channels and provides manual
control buttons for pumps and valves to detect leaks and sensor faults.
"""

from __future__ import annotations

import os
import sys
import time
from collections import deque

try:
    from PySide6 import QtCore, QtGui, QtWidgets
except Exception:
    from PyQt5 import QtCore, QtGui, QtWidgets

import rclpy
from std_msgs.msg import Float32, Int16MultiArray

BASE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
SOFTWARE_DIR = os.path.join(BASE_DIR, "software")
if SOFTWARE_DIR not in sys.path:
    sys.path.append(SOFTWARE_DIR)

from sdk.softbot_interface import SoftBot  # noqa: E402

# ── Styling ──────────────────────────────────────────────────────────
DARK_BG = "#1a1a2e"
PANEL_BG = "#16213e"
CARD_BG = "#0f3460"
ACCENT_POS = "#00d2ff"
ACCENT_NEG = "#ff6b6b"
ACCENT_GREEN = "#00e676"
ACCENT_WARN = "#ffab40"
TEXT_PRIMARY = "#e0e0e0"
TEXT_DIM = "#8d99ae"
BORDER_COLOR = "#2b4570"

GLOBAL_STYLE = f"""
    QMainWindow {{
        background: {DARK_BG};
    }}
    QGroupBox {{
        background: {PANEL_BG};
        border: 1px solid {BORDER_COLOR};
        border-radius: 12px;
        margin-top: 18px;
        padding: 16px 12px 12px 12px;
        color: {TEXT_PRIMARY};
        font-weight: 600;
        font-size: 13px;
    }}
    QGroupBox::title {{
        subcontrol-origin: margin;
        subcontrol-position: top left;
        padding: 2px 12px;
        background: {CARD_BG};
        border-radius: 6px;
        color: {ACCENT_POS};
    }}
    QLabel {{
        color: {TEXT_PRIMARY};
        font-size: 13px;
    }}
    QPushButton {{
        background: {CARD_BG};
        color: {TEXT_PRIMARY};
        border: 1px solid {BORDER_COLOR};
        border-radius: 8px;
        padding: 10px 18px;
        font-weight: 600;
        font-size: 12px;
        min-height: 28px;
    }}
    QPushButton:hover {{
        background: #1a5276;
        border-color: {ACCENT_POS};
    }}
    QPushButton:pressed {{
        background: #0d3b66;
    }}
    QPushButton:checked {{
        background: #1b5e20;
        border-color: {ACCENT_GREEN};
        color: {ACCENT_GREEN};
    }}
    QSpinBox {{
        background: {CARD_BG};
        color: {TEXT_PRIMARY};
        border: 1px solid {BORDER_COLOR};
        border-radius: 6px;
        padding: 6px;
        font-size: 13px;
    }}
"""

HISTORY_LEN = 200  # ~4 seconds at 50Hz


class PressureGauge(QtWidgets.QFrame):
    """Large pressure display with color-coded value and mini sparkline."""

    def __init__(self, title: str, accent: str):
        super().__init__()
        self.accent = accent
        self.history: deque[float] = deque(maxlen=HISTORY_LEN)
        self.setFixedHeight(180)
        self.setStyleSheet(f"""
            QFrame {{
                background: {CARD_BG};
                border: 1px solid {BORDER_COLOR};
                border-radius: 14px;
            }}
        """)

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(16, 12, 16, 8)

        self.title_label = QtWidgets.QLabel(title)
        self.title_label.setStyleSheet(f"color: {TEXT_DIM}; font-size: 11px; font-weight: 600;")
        self.title_label.setAlignment(QtCore.Qt.AlignCenter)

        self.value_label = QtWidgets.QLabel("0.00")
        self.value_label.setAlignment(QtCore.Qt.AlignCenter)
        self.value_label.setStyleSheet(f"""
            color: {accent};
            font-size: 42px;
            font-weight: 800;
            font-family: 'Consolas', 'Courier New', monospace;
        """)

        self.unit_label = QtWidgets.QLabel("kPa")
        self.unit_label.setAlignment(QtCore.Qt.AlignCenter)
        self.unit_label.setStyleSheet(f"color: {TEXT_DIM}; font-size: 12px;")

        self.min_max_label = QtWidgets.QLabel("Min: — / Max: —")
        self.min_max_label.setAlignment(QtCore.Qt.AlignCenter)
        self.min_max_label.setStyleSheet(f"color: {TEXT_DIM}; font-size: 10px;")

        layout.addWidget(self.title_label)
        layout.addWidget(self.value_label)
        layout.addWidget(self.unit_label)
        layout.addWidget(self.min_max_label)

        self._min_val = None
        self._max_val = None

    def update_value(self, val: float):
        self.history.append(val)
        self.value_label.setText(f"{val:+.2f}")

        if self._min_val is None or val < self._min_val:
            self._min_val = val
        if self._max_val is None or val > self._max_val:
            self._max_val = val
        self.min_max_label.setText(
            f"Min: {self._min_val:+.2f} / Max: {self._max_val:+.2f}"
        )

    def reset_minmax(self):
        self._min_val = None
        self._max_val = None
        self.min_max_label.setText("Min: — / Max: —")


class ToggleButton(QtWidgets.QPushButton):
    """Checkable toggle button with ON/OFF visual states."""

    def __init__(self, label: str):
        super().__init__(label)
        self.setCheckable(True)
        self.setMinimumWidth(130)


class SensorDiagnosticGUI(QtWidgets.QMainWindow):
    """Main diagnostic window for dual-sensor leak testing."""

    def __init__(self, bot: SoftBot):
        super().__init__()
        self.bot = bot
        self.setWindowTitle("SoftBot — Sensor & Leak Diagnostic")
        self.resize(900, 700)
        self.setStyleSheet(GLOBAL_STYLE)

        self._build_ui()
        self._connect_signals()

        # Telemetry polling timer
        self.poll_timer = QtCore.QTimer()
        self.poll_timer.timeout.connect(self._on_poll)
        self.poll_timer.start(40)  # 25 Hz UI refresh

        # Safety: auto stop if window is left unattended
        self.last_interaction = time.monotonic()

    # ── UI Construction ──────────────────────────────────────────────

    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        root.setSpacing(10)
        root.setContentsMargins(14, 14, 14, 14)

        # ── Sensor Gauges Row ────────────────────────────────────────
        gauge_row = QtWidgets.QHBoxLayout()
        self.gauge_pos = PressureGauge("SENSOR PRESIÓN  (ADS Ch0)", ACCENT_POS)
        self.gauge_neg = PressureGauge("SENSOR VACÍO  (ADS Ch1)", ACCENT_NEG)
        gauge_row.addWidget(self.gauge_pos)
        gauge_row.addWidget(self.gauge_neg)
        root.addLayout(gauge_row)

        # ── Status Bar ───────────────────────────────────────────────
        self.status_label = QtWidgets.QLabel("Esperando datos del firmware…")
        self.status_label.setAlignment(QtCore.Qt.AlignCenter)
        self.status_label.setStyleSheet(f"""
            background: {CARD_BG};
            border: 1px solid {BORDER_COLOR};
            border-radius: 8px;
            padding: 8px;
            color: {ACCENT_WARN};
            font-size: 12px;
            font-weight: 600;
        """)
        root.addWidget(self.status_label)

        # ── Controls area ────────────────────────────────────────────
        controls_row = QtWidgets.QHBoxLayout()

        # Left: Pumps
        pump_box = QtWidgets.QGroupBox("Bombas (Hardware Diag Mode 9)")
        pump_layout = QtWidgets.QVBoxLayout(pump_box)

        pwm_row = QtWidgets.QHBoxLayout()
        pwm_label = QtWidgets.QLabel("PWM:")
        self.spin_pwm = QtWidgets.QSpinBox()
        self.spin_pwm.setRange(0, 255)
        self.spin_pwm.setValue(120)
        self.spin_pwm.setFixedWidth(80)
        pwm_row.addWidget(pwm_label)
        pwm_row.addWidget(self.spin_pwm)
        pwm_row.addStretch()
        pump_layout.addLayout(pwm_row)

        self.btn_inflate_main = ToggleButton("Inflate Main")
        self.btn_inflate_aux = ToggleButton("Inflate Aux")
        self.btn_suction_main = ToggleButton("Suction Main")
        self.btn_suction_aux = ToggleButton("Suction Aux")

        pump_grid = QtWidgets.QGridLayout()
        pump_grid.addWidget(self.btn_inflate_main, 0, 0)
        pump_grid.addWidget(self.btn_inflate_aux, 0, 1)
        pump_grid.addWidget(self.btn_suction_main, 1, 0)
        pump_grid.addWidget(self.btn_suction_aux, 1, 1)
        pump_layout.addLayout(pump_grid)

        controls_row.addWidget(pump_box)

        # Center: Valves
        valve_box = QtWidgets.QGroupBox("Válvulas")
        valve_layout = QtWidgets.QVBoxLayout(valve_box)

        self.btn_valve_inflate = ToggleButton("Valve Inflate")
        self.btn_valve_suction = ToggleButton("Valve Suction")
        self.btn_valve_chamber_c = ToggleButton("Valve Chamber C")

        valve_layout.addWidget(self.btn_valve_inflate)
        valve_layout.addWidget(self.btn_valve_suction)
        valve_layout.addWidget(self.btn_valve_chamber_c)
        valve_layout.addStretch()

        controls_row.addWidget(valve_box)

        # Right: Mux + Actions
        action_box = QtWidgets.QGroupBox("MUX / Acciones")
        action_layout = QtWidgets.QVBoxLayout(action_box)

        self.btn_mux_a = ToggleButton("MUX A")
        self.btn_mux_b = ToggleButton("MUX B")
        action_layout.addWidget(self.btn_mux_a)
        action_layout.addWidget(self.btn_mux_b)

        action_layout.addSpacing(12)

        self.btn_all_off = QtWidgets.QPushButton("⏹  TODO OFF")
        self.btn_all_off.setStyleSheet(f"""
            QPushButton {{
                background: #b71c1c;
                color: white;
                border-radius: 8px;
                padding: 12px;
                font-weight: 700;
                font-size: 14px;
            }}
            QPushButton:hover {{ background: #d32f2f; }}
        """)
        action_layout.addWidget(self.btn_all_off)

        self.btn_reset_minmax = QtWidgets.QPushButton("↺  Reset Min/Max")
        action_layout.addWidget(self.btn_reset_minmax)
        action_layout.addStretch()

        controls_row.addWidget(action_box)

        root.addLayout(controls_row)

        # ── Debug telemetry ──────────────────────────────────────────
        debug_box = QtWidgets.QGroupBox("Telemetría Debug")
        debug_layout = QtWidgets.QHBoxLayout(debug_box)

        self.lbl_pwm_main = QtWidgets.QLabel("PWM Main: —")
        self.lbl_pwm_aux = QtWidgets.QLabel("PWM Aux: —")
        self.lbl_error = QtWidgets.QLabel("Error: —")
        self.lbl_mode = QtWidgets.QLabel("Mode: —")
        self.lbl_mask = QtWidgets.QLabel("Mask: 0")
        self.lbl_mask.setStyleSheet("font-family: monospace;")

        for lbl in (self.lbl_pwm_main, self.lbl_pwm_aux, self.lbl_error, self.lbl_mode, self.lbl_mask):
            lbl.setStyleSheet(f"color: {TEXT_DIM}; font-size: 11px; font-family: monospace;")
            debug_layout.addWidget(lbl)

        root.addWidget(debug_box)

    # ── Signal Wiring ────────────────────────────────────────────────

    def _connect_signals(self):
        toggle_buttons = [
            self.btn_inflate_main,
            self.btn_inflate_aux,
            self.btn_suction_main,
            self.btn_suction_aux,
            self.btn_valve_inflate,
            self.btn_valve_suction,
            self.btn_valve_chamber_c,
            self.btn_mux_a,
            self.btn_mux_b,
        ]
        for btn in toggle_buttons:
            btn.toggled.connect(self._on_toggle_changed)

        self.spin_pwm.valueChanged.connect(self._on_toggle_changed)
        self.btn_all_off.clicked.connect(self._on_all_off)
        self.btn_reset_minmax.clicked.connect(self._on_reset_minmax)

    # ── Hardware Control ─────────────────────────────────────────────

    def _build_mask(self) -> int:
        """Build bitmask from the toggle buttons."""
        mask = 0
        if self.btn_inflate_main.isChecked():
            mask |= (1 << 0)
        if self.btn_inflate_aux.isChecked():
            mask |= (1 << 1)
        if self.btn_suction_main.isChecked():
            mask |= (1 << 2)
        if self.btn_suction_aux.isChecked():
            mask |= (1 << 3)
        if self.btn_valve_inflate.isChecked():
            mask |= (1 << 4)
        if self.btn_valve_suction.isChecked():
            mask |= (1 << 5)
        if self.btn_valve_chamber_c.isChecked():
            mask |= (1 << 6)
        if self.btn_mux_a.isChecked():
            mask |= (1 << 7)
        if self.btn_mux_b.isChecked():
            mask |= (1 << 8)
        return mask

    def _apply_current_state(self):
        mask = self._build_mask()
        pwm = self.spin_pwm.value()
        self.bot.set_hardware_test(bitmask=mask, pwm=pwm)
        self.lbl_mask.setText(f"Mask: {mask} (0b{mask:09b})")
        self.last_interaction = time.monotonic()

    def _on_toggle_changed(self, *_args):
        self._apply_current_state()

    def _on_all_off(self):
        # Uncheck all toggle buttons
        for btn in (
            self.btn_inflate_main, self.btn_inflate_aux,
            self.btn_suction_main, self.btn_suction_aux,
            self.btn_valve_inflate, self.btn_valve_suction,
            self.btn_valve_chamber_c, self.btn_mux_a, self.btn_mux_b,
        ):
            btn.blockSignals(True)
            btn.setChecked(False)
            btn.blockSignals(False)

        self.bot.stop_hardware_test()
        self.lbl_mask.setText("Mask: 0")
        self.last_interaction = time.monotonic()

    def _on_reset_minmax(self):
        self.gauge_pos.reset_minmax()
        self.gauge_neg.reset_minmax()

    # ── Telemetry Polling ────────────────────────────────────────────

    def _on_poll(self):
        try:
            state = self.bot.get_state()
        except Exception:
            return

        pressure = state.get("pressure", 0.0)
        pwm_main = state.get("pwm_main", 0)
        pwm_aux = state.get("pwm_aux", 0)
        error = state.get("error", 0.0)
        mode = state.get("logic_state", 0)

        # The firmware only publishes the ACTIVE sensor on pressure_feedback.
        # Mode 9 = hardware diagnostic, modes -1/-2 = suction, modes 1/2 = inflate
        if mode == 9 or mode in (1, 2, 0, 4):
            self.gauge_pos.update_value(pressure)
            self.status_label.setText(
                f"Feedback activo => {pressure:+.2f} kPa  |  "
                f"Mode: {mode}  |  PWM: {pwm_main}/{pwm_aux}"
            )
        elif mode in (-1, -2):
            self.gauge_neg.update_value(pressure)
            self.status_label.setText(
                f"Feedback activo => {pressure:+.2f} kPa  |  "
                f"Mode: {mode}  |  PWM: {pwm_main}/{pwm_aux}"
            )
        else:
            self.status_label.setText(
                f"Pressure: {pressure:+.2f} kPa  |  Mode: {mode}"
            )

        self.lbl_pwm_main.setText(f"PWM Main: {pwm_main}")
        self.lbl_pwm_aux.setText(f"PWM Aux: {pwm_aux}")
        self.lbl_error.setText(f"Error: {error:+.2f}")
        self.lbl_mode.setText(f"Mode: {mode}")

    # ── Cleanup ──────────────────────────────────────────────────────

    def closeEvent(self, event):
        try:
            self.poll_timer.stop()
            self.bot.stop_hardware_test()
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
    window = SensorDiagnosticGUI(bot)
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()

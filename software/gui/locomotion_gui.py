#!/usr/bin/env python3
"""Locomotion sequence studio for SoftBot (3-chamber aware)."""

from __future__ import annotations

import json
import os
import sys
import time
from collections import deque
from dataclasses import asdict, dataclass
from datetime import datetime

try:
    from PySide6 import QtCore, QtGui, QtWidgets
except Exception:
    from PyQt5 import QtCore, QtGui, QtWidgets

try:
    import pyqtgraph as pg
except Exception as exc:
    raise SystemExit(
        "Missing dependency: pyqtgraph. Install with: python3 -m pip install pyqtgraph PySide6"
    ) from exc

import rclpy

BASE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
SOFTWARE_DIR = os.path.join(BASE_DIR, "software")
if SOFTWARE_DIR not in sys.path:
    sys.path.append(SOFTWARE_DIR)

from sdk.protocol import (  # noqa: E402
    CHAMBER_A,
    CHAMBER_AB,
    CHAMBER_ABC,
    CHAMBER_AC,
    CHAMBER_B,
    CHAMBER_BC,
    CHAMBER_BLOCKED,
    CHAMBER_C,
    MODE_PWM_INFLATE,
    MODE_PWM_SUCTION,
)
from sdk.softbot_interface import SoftBot  # noqa: E402


ACTION_PID_INFLATE = "pid_inflate"
ACTION_PID_SUCTION = "pid_suction"
ACTION_PWM_INFLATE = "pwm_inflate"
ACTION_PWM_SUCTION = "pwm_suction"
ACTION_VENT = "vent"
ACTION_STOP = "stop"

ACTION_OPTIONS = (
    ("PID Inflate", ACTION_PID_INFLATE),
    ("PID Suction", ACTION_PID_SUCTION),
    ("PWM Inflate", ACTION_PWM_INFLATE),
    ("PWM Suction", ACTION_PWM_SUCTION),
    ("Vent", ACTION_VENT),
    ("Stop", ACTION_STOP),
)
ACTION_LABELS = {value: label for label, value in ACTION_OPTIONS}

CHAMBER_OPTIONS = (
    ("A (1)", CHAMBER_A),
    ("B (2)", CHAMBER_B),
    ("C (4)", CHAMBER_C),
    ("A+B (3)", CHAMBER_AB),
    ("A+C (5)", CHAMBER_AC),
    ("B+C (6)", CHAMBER_BC),
    ("A+B+C (7)", CHAMBER_ABC),
    ("Blocked (0)", CHAMBER_BLOCKED),
)
CHAMBER_VALUES = {value for _, value in CHAMBER_OPTIONS}
CHAMBER_LABELS = {value: label for label, value in CHAMBER_OPTIONS}

PRESSURE_ACTIONS = {ACTION_PID_INFLATE, ACTION_PID_SUCTION}
PWM_ACTIONS = {ACTION_PWM_INFLATE, ACTION_PWM_SUCTION}


COL_ENABLED = 0
COL_NAME = 1
COL_CHAMBER = 2
COL_ACTION = 3
COL_TARGET = 4
COL_MIN = 5
COL_MAX = 6
COL_TOL = 7
COL_SETTLE = 8
COL_SNAP = 9


@dataclass
class PhaseSpec:
    name: str
    chamber: int
    action: str
    target: float
    min_time_s: float
    max_time_s: float
    tol_kpa: float = 3.0
    settle_s: float = 0.08
    snap_ms: int = 0
    enabled: bool = True

    def normalized(self) -> "PhaseSpec":
        name = str(self.name).strip() or "phase"
        chamber = int(self.chamber)
        if chamber not in CHAMBER_VALUES:
            raise ValueError(f"Invalid chamber mask: {chamber}")

        action = str(self.action).strip().lower()
        valid_actions = {value for _, value in ACTION_OPTIONS}
        if action not in valid_actions:
            raise ValueError(f"Invalid action: {action}")

        target = float(self.target)
        min_time_s = max(0.0, float(self.min_time_s))
        max_time_s = max(min_time_s, float(self.max_time_s))
        tol_kpa = max(0.0, float(self.tol_kpa))
        settle_s = max(0.0, float(self.settle_s))
        snap_ms = max(0, int(round(float(self.snap_ms))))

        return PhaseSpec(
            name=name,
            chamber=chamber,
            action=action,
            target=target,
            min_time_s=min_time_s,
            max_time_s=max_time_s,
            tol_kpa=tol_kpa,
            settle_s=settle_s,
            snap_ms=snap_ms,
            enabled=bool(self.enabled),
        )

    def to_dict(self) -> dict:
        return asdict(self)

    @staticmethod
    def from_dict(raw: dict) -> "PhaseSpec":
        return PhaseSpec(
            name=raw.get("name", "phase"),
            chamber=raw.get("chamber", CHAMBER_AB),
            action=raw.get("action", ACTION_PID_INFLATE),
            target=raw.get("target", raw.get("target_kpa", 0.0)),
            min_time_s=raw.get("min_time_s", raw.get("min_time", 0.3)),
            max_time_s=raw.get("max_time_s", raw.get("max_time", 1.5)),
            tol_kpa=raw.get("tol_kpa", raw.get("tol", 3.0)),
            settle_s=raw.get("settle_s", raw.get("settle_time", 0.08)),
            snap_ms=raw.get("snap_ms", 0),
            enabled=raw.get("enabled", True),
        ).normalized()


def _clone_phases(phases: list[PhaseSpec]) -> list[PhaseSpec]:
    return [PhaseSpec(**phase.to_dict()) for phase in phases]


def preset_ab_settle() -> list[PhaseSpec]:
    return [
        PhaseSpec(
            name="AB suction settle",
            chamber=CHAMBER_AB,
            action=ACTION_PID_SUCTION,
            target=-25.0,
            min_time_s=0.50,
            max_time_s=1.60,
            tol_kpa=3.0,
            settle_s=0.18,
            snap_ms=60,
        ),
        PhaseSpec(
            name="AB inflate settle",
            chamber=CHAMBER_AB,
            action=ACTION_PID_INFLATE,
            target=40.0,
            min_time_s=1.00,
            max_time_s=1.60,
            tol_kpa=3.0,
            settle_s=0.18,
            snap_ms=60,
        ),
    ]


def preset_sync_ab_xcrabs() -> list[PhaseSpec]:
    return [
        PhaseSpec(
            name="xcrabs suction AB",
            chamber=CHAMBER_AB,
            action=ACTION_PID_SUCTION,
            target=-15.0,
            min_time_s=0.80,
            max_time_s=1.50,
            tol_kpa=3.0,
            settle_s=0.10,
            snap_ms=50,
        ),
        PhaseSpec(
            name="xcrabs inflate AB",
            chamber=CHAMBER_AB,
            action=ACTION_PID_INFLATE,
            target=35.0,
            min_time_s=1.00,
            max_time_s=2.00,
            tol_kpa=4.0,
            settle_s=0.05,
            snap_ms=50,
        ),
    ]


def preset_swim_ab() -> list[PhaseSpec]:
    return [
        PhaseSpec(
            name="step A push",
            chamber=CHAMBER_A,
            action=ACTION_PID_INFLATE,
            target=35.0,
            min_time_s=0.60,
            max_time_s=1.50,
            tol_kpa=4.0,
            settle_s=0.02,
            snap_ms=0,
        ),
        PhaseSpec(
            name="step B push",
            chamber=CHAMBER_B,
            action=ACTION_PID_INFLATE,
            target=35.0,
            min_time_s=0.60,
            max_time_s=1.50,
            tol_kpa=4.0,
            settle_s=0.02,
            snap_ms=0,
        ),
    ]


def preset_turn_left() -> list[PhaseSpec]:
    return [
        PhaseSpec(
            name="A recover",
            chamber=CHAMBER_A,
            action=ACTION_PID_SUCTION,
            target=-15.0,
            min_time_s=0.40,
            max_time_s=1.20,
            tol_kpa=4.0,
            settle_s=0.01,
            snap_ms=0,
        ),
        PhaseSpec(
            name="A push",
            chamber=CHAMBER_A,
            action=ACTION_PID_INFLATE,
            target=35.0,
            min_time_s=0.40,
            max_time_s=1.20,
            tol_kpa=4.0,
            settle_s=0.01,
            snap_ms=0,
        ),
    ]


def preset_turn_right() -> list[PhaseSpec]:
    return [
        PhaseSpec(
            name="B recover",
            chamber=CHAMBER_B,
            action=ACTION_PID_SUCTION,
            target=-15.0,
            min_time_s=0.40,
            max_time_s=1.20,
            tol_kpa=4.0,
            settle_s=0.01,
            snap_ms=0,
        ),
        PhaseSpec(
            name="B push",
            chamber=CHAMBER_B,
            action=ACTION_PID_INFLATE,
            target=35.0,
            min_time_s=0.40,
            max_time_s=1.20,
            tol_kpa=4.0,
            settle_s=0.01,
            snap_ms=0,
        ),
    ]


def preset_abc_wave() -> list[PhaseSpec]:
    return [
        PhaseSpec(
            name="A traction",
            chamber=CHAMBER_A,
            action=ACTION_PID_INFLATE,
            target=32.0,
            min_time_s=0.45,
            max_time_s=1.20,
            tol_kpa=3.5,
            settle_s=0.05,
            snap_ms=35,
        ),
        PhaseSpec(
            name="B traction",
            chamber=CHAMBER_B,
            action=ACTION_PID_INFLATE,
            target=32.0,
            min_time_s=0.45,
            max_time_s=1.20,
            tol_kpa=3.5,
            settle_s=0.05,
            snap_ms=35,
        ),
        PhaseSpec(
            name="C traction",
            chamber=CHAMBER_C,
            action=ACTION_PID_INFLATE,
            target=32.0,
            min_time_s=0.45,
            max_time_s=1.20,
            tol_kpa=3.5,
            settle_s=0.05,
            snap_ms=35,
        ),
        PhaseSpec(
            name="ABC recover",
            chamber=CHAMBER_ABC,
            action=ACTION_PID_SUCTION,
            target=-18.0,
            min_time_s=0.75,
            max_time_s=1.60,
            tol_kpa=3.0,
            settle_s=0.08,
            snap_ms=45,
        ),
    ]


def preset_abc_pulse() -> list[PhaseSpec]:
    return [
        PhaseSpec(
            name="ABC inflate pulse",
            chamber=CHAMBER_ABC,
            action=ACTION_PID_INFLATE,
            target=36.0,
            min_time_s=0.90,
            max_time_s=1.80,
            tol_kpa=4.0,
            settle_s=0.06,
            snap_ms=40,
        ),
        PhaseSpec(
            name="ABC vent",
            chamber=CHAMBER_ABC,
            action=ACTION_VENT,
            target=0.0,
            min_time_s=0.45,
            max_time_s=0.80,
            tol_kpa=0.0,
            settle_s=0.0,
            snap_ms=25,
        ),
        PhaseSpec(
            name="ABC suction reset",
            chamber=CHAMBER_ABC,
            action=ACTION_PID_SUCTION,
            target=-20.0,
            min_time_s=0.60,
            max_time_s=1.40,
            tol_kpa=3.0,
            settle_s=0.08,
            snap_ms=20,
        ),
    ]


PRESETS: dict[str, tuple[str, list[PhaseSpec]]] = {
    "AB settle (locomocion_ab)": (
        "Cycle based on locomocion_ab.py with settle + timeout logic.",
        preset_ab_settle(),
    ),
    "x_crabs sync AB": (
        "Two-phase AB sync from x_crabs strategic runner.",
        preset_sync_ab_xcrabs(),
    ),
    "x_crabs swim AB": (
        "Alternating A/B push rhythm (simplified for GUI sequence runner).",
        preset_swim_ab(),
    ),
    "x_crabs turn left": (
        "Pivot around chamber A.",
        preset_turn_left(),
    ),
    "x_crabs turn right": (
        "Pivot around chamber B.",
        preset_turn_right(),
    ),
    "3-chamber wave": (
        "Sequential A->B->C traction with ABC recover.",
        preset_abc_wave(),
    ),
    "3-chamber pulse": (
        "ABC pulse profile with vent and suction reset.",
        preset_abc_pulse(),
    ),
}


@dataclass
class EngineSnapshot:
    running: bool = False
    paused: bool = False
    pressure_kpa: float = 0.0
    target_kpa: float = 0.0
    chamber: int = CHAMBER_BLOCKED
    action: str = ACTION_STOP
    phase_name: str = "idle"
    phase_index: int = -1
    phase_elapsed_s: float = 0.0
    total_elapsed_s: float = 0.0
    error_kpa: float = 0.0
    in_tolerance: bool = False
    loops_done: int = 0
    loop_limit: int = 0
    arrival_s: float | None = None


class SequenceEngine:
    """Real-time runner for editable locomotion phase sequences."""

    def __init__(self, bot: SoftBot, log_fn):
        self.bot = bot
        self.log = log_fn
        self.snapshot = EngineSnapshot()

        self.phases: list[PhaseSpec] = []
        self.loop_limit = 0
        self._phase_index = -1
        self._phase_first_entry = False
        self._phase_started_mono = 0.0
        self._sequence_started_mono = 0.0
        self._pending_snap_until = 0.0
        self._within_since = None

    def configure(self, phases: list[PhaseSpec], loop_limit: int) -> None:
        normalized = [phase.normalized() for phase in phases if phase.enabled]
        if not normalized:
            raise ValueError("At least one enabled phase is required.")

        self.phases = normalized
        self.loop_limit = max(0, int(loop_limit))
        self.snapshot.loop_limit = self.loop_limit
        self.snapshot.loops_done = 0

    def start(self) -> None:
        if not self.phases:
            raise ValueError("No sequence configured.")

        now = time.monotonic()
        self._phase_index = 0
        self._phase_first_entry = True
        self._phase_started_mono = now
        self._sequence_started_mono = now
        self._pending_snap_until = 0.0
        self._within_since = None

        self.snapshot = EngineSnapshot(
            running=True,
            paused=False,
            loop_limit=self.loop_limit,
            loops_done=0,
        )
        self.log(f"Sequence started. phases={len(self.phases)} loop_limit={self.loop_limit or 'inf'}")

    def pause(self) -> None:
        if not self.snapshot.running or self.snapshot.paused:
            return
        self.snapshot.paused = True
        self.bot.stop()
        self.log("Sequence paused.")

    def resume(self) -> None:
        if not self.snapshot.running or not self.snapshot.paused:
            return
        now = time.monotonic()
        self.snapshot.paused = False
        self._phase_first_entry = True
        self._phase_started_mono = now
        self._within_since = None
        self.snapshot.arrival_s = None
        self.log("Sequence resumed.")

    def stop(self, estop: bool = False, reason: str = "") -> None:
        if not self.snapshot.running and not estop:
            return

        try:
            if estop:
                self.bot.set_chamber(CHAMBER_BLOCKED)
            self.bot.stop()
        except Exception as exc:
            self.log(f"Stop warning: {exc}")

        self.snapshot.running = False
        self.snapshot.paused = False
        self.snapshot.phase_name = "idle"
        self.snapshot.phase_index = -1
        self.snapshot.phase_elapsed_s = 0.0
        self.snapshot.target_kpa = 0.0
        self.snapshot.action = ACTION_STOP
        self.snapshot.chamber = CHAMBER_BLOCKED
        self._phase_index = -1
        self._phase_first_entry = False
        self._pending_snap_until = 0.0
        self._within_since = None

        if estop:
            self.log("E-STOP triggered.")
        elif reason:
            self.log(f"Sequence stopped: {reason}")
        else:
            self.log("Sequence stopped.")

    def _apply_phase_command(self, phase: PhaseSpec) -> float:
        chamber = int(phase.chamber)
        self.bot.set_chamber(chamber)

        target_value = float(phase.target)
        if phase.action == ACTION_PID_INFLATE:
            target_value = abs(target_value)
            self.bot.inflate(target_value)
        elif phase.action == ACTION_PID_SUCTION:
            target_value = -abs(target_value)
            self.bot.suction(target_value)
        elif phase.action == ACTION_PWM_INFLATE:
            target_value = max(0.0, min(255.0, abs(target_value)))
            self.bot.set_pwm(target_value, MODE_PWM_INFLATE)
        elif phase.action == ACTION_PWM_SUCTION:
            target_value = max(0.0, min(255.0, abs(target_value)))
            self.bot.set_pwm(target_value, MODE_PWM_SUCTION)
        elif phase.action == ACTION_VENT:
            target_value = 0.0
            self.bot.vent(chamber_id=chamber, duration_s=None)
        elif phase.action == ACTION_STOP:
            target_value = 0.0
            self.bot.stop()
        else:
            raise ValueError(f"Unsupported action: {phase.action}")

        return target_value

    def _advance_phase(self, now: float) -> None:
        if not self.snapshot.running:
            return

        self._phase_index += 1
        if self._phase_index >= len(self.phases):
            self._phase_index = 0
            self.snapshot.loops_done += 1
            self.log(f"Loop completed: {self.snapshot.loops_done}")

            if self.loop_limit > 0 and self.snapshot.loops_done >= self.loop_limit:
                self.stop(reason="loop target reached")
                return

        self._phase_first_entry = True
        self._phase_started_mono = now
        self._within_since = None
        self.snapshot.arrival_s = None

    def tick(self) -> EngineSnapshot:
        try:
            state = self.bot.get_state()
            self.snapshot.pressure_kpa = float(state.get("pressure", 0.0))
        except Exception as exc:
            self.log(f"Telemetry warning: {exc}")

        now = time.monotonic()

        if not self.snapshot.running:
            return self.snapshot

        self.snapshot.total_elapsed_s = max(0.0, now - self._sequence_started_mono)

        if self.snapshot.paused:
            return self.snapshot

        if self._pending_snap_until > now:
            self.snapshot.action = ACTION_STOP
            self.snapshot.phase_name = "snap"
            self.snapshot.target_kpa = 0.0
            self.snapshot.error_kpa = 0.0
            self.snapshot.in_tolerance = False
            self.snapshot.phase_elapsed_s = max(0.0, now - self._phase_started_mono)
            return self.snapshot

        if self._pending_snap_until > 0.0 and now >= self._pending_snap_until:
            self._pending_snap_until = 0.0
            self._advance_phase(now)
            if not self.snapshot.running:
                return self.snapshot

        phase = self.phases[self._phase_index]

        if self._phase_first_entry:
            command_target = self._apply_phase_command(phase)
            self._phase_first_entry = False
            self._phase_started_mono = now
            self._within_since = None

            self.snapshot.phase_index = self._phase_index
            self.snapshot.phase_name = phase.name
            self.snapshot.action = phase.action
            self.snapshot.chamber = phase.chamber
            self.snapshot.target_kpa = command_target
            self.snapshot.arrival_s = None
            self.log(
                f"Phase {self._phase_index + 1}/{len(self.phases)}: {phase.name} | "
                f"{ACTION_LABELS.get(phase.action, phase.action)} | "
                f"ch={phase.chamber} target={command_target:.2f}"
            )

        elapsed = max(0.0, now - self._phase_started_mono)
        self.snapshot.phase_elapsed_s = elapsed

        in_tolerance = False
        error_kpa = 0.0
        if phase.action in PRESSURE_ACTIONS:
            error_kpa = abs(self.snapshot.pressure_kpa - self.snapshot.target_kpa)
            if error_kpa <= phase.tol_kpa:
                if self._within_since is None:
                    self._within_since = now
                    self.snapshot.arrival_s = elapsed
                if (now - self._within_since) >= phase.settle_s:
                    in_tolerance = True
            else:
                self._within_since = None
        else:
            self._within_since = None

        self.snapshot.error_kpa = error_kpa
        self.snapshot.in_tolerance = in_tolerance

        time_ok = elapsed >= phase.min_time_s
        timeout = elapsed >= phase.max_time_s

        if phase.action in PRESSURE_ACTIONS:
            should_advance = (in_tolerance and time_ok) or timeout
        else:
            should_advance = time_ok or timeout

        if should_advance:
            if phase.snap_ms > 0:
                self.bot.stop()
                self._pending_snap_until = now + (phase.snap_ms / 1000.0)
                self._phase_started_mono = now
            else:
                self._advance_phase(now)

        return self.snapshot


class LocomotionGUI(QtWidgets.QMainWindow):
    def __init__(self, bot: SoftBot):
        super().__init__()
        self.bot = bot
        self.engine = SequenceEngine(bot=bot, log_fn=self._append_log)

        self._shutdown_done = False
        self._plot_refresh_counter = 0
        self.telemetry_start_mono = time.monotonic()
        self.time_hist: deque[float] = deque(maxlen=9000)
        self.pressure_hist: deque[float] = deque(maxlen=9000)
        self.target_hist: deque[float] = deque(maxlen=9000)
        self.telemetry_rows: deque[tuple] = deque(maxlen=120000)

        self.setWindowTitle("SoftBot Locomotion Studio (3-chamber)")
        self.resize(1580, 920)
        self._setup_style()
        self._build_ui()

        self.control_timer = QtCore.QTimer(self)
        self.control_timer.setInterval(20)
        self.control_timer.timeout.connect(self._on_control_tick)
        self.control_timer.start()

        self._load_preset_to_table("3-chamber wave")
        self._append_log("GUI ready. PID remains embedded in firmware for PID actions.")

    def _setup_style(self) -> None:
        self.setStyleSheet(
            """
            QMainWindow { background: #f1f5f9; }
            QWidget#leftPanel, QWidget#leftPanelContent {
                background: #eef2f7;
            }
            QScrollArea#leftScroll {
                border: none;
                background: #eef2f7;
            }
            QGroupBox {
                font-weight: 600;
                border: 1px solid #d7dce5;
                border-radius: 12px;
                margin-top: 10px;
                padding-top: 16px;
                background: #f8fafc;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                left: 12px;
                top: 2px;
                padding: 0 2px;
                color: #0f172a;
                background: transparent;
                border: none;
                font-weight: 700;
            }
            QLabel {
                color: #1f2937;
            }
            QCheckBox {
                color: #1f2937;
                spacing: 6px;
            }
            QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox {
                background: #ffffff;
                color: #0f172a;
                border: 1px solid #cbd5e1;
                border-radius: 6px;
                padding: 4px 8px;
                min-height: 28px;
                selection-background-color: #0f4c81;
                selection-color: #ffffff;
            }
            QComboBox::drop-down {
                border: none;
                width: 24px;
            }
            QComboBox QAbstractItemView {
                background: #ffffff;
                color: #0f172a;
                selection-background-color: #0f4c81;
                selection-color: #ffffff;
            }
            QSpinBox::up-button, QSpinBox::down-button,
            QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {
                width: 18px;
                border: none;
            }
            QPushButton {
                background: #0f4c81;
                color: #ffffff;
                border-radius: 6px;
                padding: 6px 10px;
                min-height: 28px;
            }
            QPushButton:hover { background: #145a98; }
            QPushButton:disabled { background: #9db0c5; }
            QPushButton#dangerButton { background: #9f1239; }
            QPushButton#dangerButton:hover { background: #be123c; }
            QPushButton#neutralButton { background: #3f4b61; }
            QPushButton#neutralButton:hover { background: #4b5b75; }
            QTableWidget {
                background: #ffffff;
                color: #0f172a;
                alternate-background-color: #f8fafc;
                gridline-color: #e2e8f0;
            }
            QHeaderView::section {
                background: #e2e8f0;
                color: #0f172a;
                border: 1px solid #cbd5e1;
                padding: 4px 6px;
                font-weight: 600;
            }
            QTableWidget::item {
                color: #0f172a;
            }
            QTableWidget::item:selected {
                background: #0f766e;
                color: #ffffff;
            }
            QTextEdit {
                background: #0f172a;
                color: #e2e8f0;
                border-radius: 8px;
                border: 1px solid #1e293b;
            }
            """
        )

    def _build_ui(self) -> None:
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QHBoxLayout(central)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(10)

        splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        root.addWidget(splitter)

        splitter.addWidget(self._build_left_panel())
        splitter.addWidget(self._build_right_panel())
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([430, 1150])

    def _build_left_panel(self) -> QtWidgets.QWidget:
        panel = QtWidgets.QWidget()
        panel.setObjectName("leftPanel")
        layout = QtWidgets.QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)

        scroll = QtWidgets.QScrollArea()
        scroll.setObjectName("leftScroll")
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QtWidgets.QFrame.NoFrame)

        content = QtWidgets.QWidget()
        content.setObjectName("leftPanelContent")
        content_layout = QtWidgets.QVBoxLayout(content)
        content_layout.setContentsMargins(0, 0, 0, 0)
        content_layout.setSpacing(10)

        content_layout.addWidget(self._build_run_group())
        content_layout.addWidget(self._build_manual_group())
        content_layout.addWidget(self._build_tuning_group())
        content_layout.addStretch(1)

        scroll.setWidget(content)
        layout.addWidget(scroll)
        panel.setMinimumWidth(420)
        return panel

    def _build_right_panel(self) -> QtWidgets.QWidget:
        panel = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)

        layout.addWidget(self._build_sequence_editor_group(), 4)
        layout.addWidget(self._build_plot_group(), 3)
        layout.addWidget(self._build_log_group(), 2)
        return panel

    def _build_run_group(self) -> QtWidgets.QGroupBox:
        box = QtWidgets.QGroupBox("Run control")
        form = QtWidgets.QFormLayout(box)
        form.setLabelAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)
        form.setFormAlignment(QtCore.Qt.AlignTop)
        form.setFieldGrowthPolicy(QtWidgets.QFormLayout.AllNonFixedFieldsGrow)
        form.setHorizontalSpacing(10)
        form.setVerticalSpacing(7)

        self.input_seq_name = QtWidgets.QLineEdit("locomotion_session")

        self.combo_preset = QtWidgets.QComboBox()
        for preset_name in PRESETS:
            self.combo_preset.addItem(preset_name)
        self.btn_load_preset = QtWidgets.QPushButton("Load preset")
        self.btn_load_preset.clicked.connect(self._on_load_preset)

        preset_row = QtWidgets.QHBoxLayout()
        preset_row.addWidget(self.combo_preset, 1)
        preset_row.addWidget(self.btn_load_preset)
        preset_widget = QtWidgets.QWidget()
        preset_widget.setLayout(preset_row)

        self.spin_loops = QtWidgets.QSpinBox()
        self.spin_loops.setRange(0, 100000)
        self.spin_loops.setValue(0)
        self.spin_loops.setSpecialValueText("infinite")

        self.spin_tick_ms = QtWidgets.QSpinBox()
        self.spin_tick_ms.setRange(10, 200)
        self.spin_tick_ms.setSingleStep(5)
        self.spin_tick_ms.setValue(20)
        self.spin_tick_ms.valueChanged.connect(self._on_tick_interval_changed)

        self.btn_start = QtWidgets.QPushButton("Start")
        self.btn_pause = QtWidgets.QPushButton("Pause")
        self.btn_stop = QtWidgets.QPushButton("Stop")
        self.btn_estop = QtWidgets.QPushButton("E-STOP")
        self.btn_pause.setObjectName("neutralButton")
        self.btn_stop.setObjectName("neutralButton")
        self.btn_estop.setObjectName("dangerButton")

        self.btn_start.clicked.connect(self._on_start)
        self.btn_pause.clicked.connect(self._on_pause_resume)
        self.btn_stop.clicked.connect(self._on_stop)
        self.btn_estop.clicked.connect(self._on_estop)

        btn_row = QtWidgets.QGridLayout()
        btn_row.addWidget(self.btn_start, 0, 0)
        btn_row.addWidget(self.btn_pause, 0, 1)
        btn_row.addWidget(self.btn_stop, 1, 0)
        btn_row.addWidget(self.btn_estop, 1, 1)
        btn_widget = QtWidgets.QWidget()
        btn_widget.setLayout(btn_row)

        self.label_status = QtWidgets.QLabel("IDLE")
        self.label_phase = QtWidgets.QLabel("-")
        self.label_loop = QtWidgets.QLabel("0")
        self.label_pressure = QtWidgets.QLabel("0.00 kPa")
        self.label_error = QtWidgets.QLabel("0.00")

        self.btn_export_telemetry = QtWidgets.QPushButton("Export telemetry CSV")
        self.btn_export_telemetry.clicked.connect(self._export_telemetry_csv)

        status_grid = QtWidgets.QGridLayout()
        status_grid.setHorizontalSpacing(8)
        status_grid.setVerticalSpacing(4)
        status_grid.addWidget(QtWidgets.QLabel("State"), 0, 0)
        status_grid.addWidget(self.label_status, 0, 1)
        status_grid.addWidget(QtWidgets.QLabel("Phase"), 1, 0)
        status_grid.addWidget(self.label_phase, 1, 1)
        status_grid.addWidget(QtWidgets.QLabel("Loop"), 2, 0)
        status_grid.addWidget(self.label_loop, 2, 1)
        status_grid.addWidget(QtWidgets.QLabel("Pressure"), 3, 0)
        status_grid.addWidget(self.label_pressure, 3, 1)
        status_grid.addWidget(QtWidgets.QLabel("Error"), 4, 0)
        status_grid.addWidget(self.label_error, 4, 1)
        status_widget = QtWidgets.QWidget()
        status_widget.setLayout(status_grid)

        hint = QtWidgets.QLabel(
            "PID actions keep control embedded in firmware. PWM/VENT/STOP can be mixed per phase."
        )
        hint.setWordWrap(True)
        hint.setStyleSheet("color: #334155;")

        form.addRow("Sequence name", self.input_seq_name)
        form.addRow("Preset", preset_widget)
        form.addRow("Loop count", self.spin_loops)
        form.addRow("Tick interval", self.spin_tick_ms)
        form.addRow(btn_widget)
        form.addRow(status_widget)
        form.addRow(self.btn_export_telemetry)
        form.addRow(hint)
        return box

    def _build_manual_group(self) -> QtWidgets.QGroupBox:
        box = QtWidgets.QGroupBox("Manual command")
        form = QtWidgets.QFormLayout(box)
        form.setHorizontalSpacing(10)
        form.setVerticalSpacing(7)

        self.combo_manual_chamber = QtWidgets.QComboBox()
        for label, value in CHAMBER_OPTIONS:
            self.combo_manual_chamber.addItem(label, value)
        default_chamber_idx = self.combo_manual_chamber.findData(CHAMBER_ABC)
        self.combo_manual_chamber.setCurrentIndex(max(0, default_chamber_idx))

        self.combo_manual_action = QtWidgets.QComboBox()
        for label, value in ACTION_OPTIONS:
            self.combo_manual_action.addItem(label, value)

        self.spin_manual_value = QtWidgets.QDoubleSpinBox()
        self.spin_manual_value.setRange(-255.0, 255.0)
        self.spin_manual_value.setDecimals(2)
        self.spin_manual_value.setValue(20.0)

        self.btn_manual_send = QtWidgets.QPushButton("Send manual command")
        self.btn_manual_send.clicked.connect(self._on_send_manual)

        form.addRow("Chamber", self.combo_manual_chamber)
        form.addRow("Action", self.combo_manual_action)
        form.addRow("Value", self.spin_manual_value)
        form.addRow(self.btn_manual_send)
        return box

    def _build_tuning_group(self) -> QtWidgets.QGroupBox:
        box = QtWidgets.QGroupBox("Firmware PID tuning")
        form = QtWidgets.QFormLayout(box)
        form.setHorizontalSpacing(10)
        form.setVerticalSpacing(7)

        cache = dict(self.bot.tuning_cache)

        self.spin_kp_pos = QtWidgets.QDoubleSpinBox()
        self.spin_kp_pos.setRange(-3000.0, 3000.0)
        self.spin_kp_pos.setDecimals(3)
        self.spin_kp_pos.setValue(float(cache.get("kp_pos", 24.0)))

        self.spin_ki_pos = QtWidgets.QDoubleSpinBox()
        self.spin_ki_pos.setRange(-10000.0, 10000.0)
        self.spin_ki_pos.setDecimals(3)
        self.spin_ki_pos.setValue(float(cache.get("ki_pos", 1500.0)))

        self.spin_kp_neg = QtWidgets.QDoubleSpinBox()
        self.spin_kp_neg.setRange(-3000.0, 3000.0)
        self.spin_kp_neg.setDecimals(3)
        self.spin_kp_neg.setValue(float(cache.get("kp_neg", -75.0)))

        self.spin_ki_neg = QtWidgets.QDoubleSpinBox()
        self.spin_ki_neg.setRange(-10000.0, 10000.0)
        self.spin_ki_neg.setDecimals(3)
        self.spin_ki_neg.setValue(float(cache.get("ki_neg", -750.0)))

        self.spin_max_safe = QtWidgets.QDoubleSpinBox()
        self.spin_max_safe.setRange(0.0, 120.0)
        self.spin_max_safe.setDecimals(2)
        self.spin_max_safe.setValue(float(cache.get("max_safe", 55.0)))

        self.spin_min_safe = QtWidgets.QDoubleSpinBox()
        self.spin_min_safe.setRange(-120.0, 0.0)
        self.spin_min_safe.setDecimals(2)
        self.spin_min_safe.setValue(float(cache.get("min_safe", -60.0)))

        self.btn_apply_tuning = QtWidgets.QPushButton("Apply tuning")
        self.btn_apply_tuning.clicked.connect(self._on_apply_tuning)

        form.addRow("Kp+", self.spin_kp_pos)
        form.addRow("Ki+", self.spin_ki_pos)
        form.addRow("Kp-", self.spin_kp_neg)
        form.addRow("Ki-", self.spin_ki_neg)
        form.addRow("Max safe", self.spin_max_safe)
        form.addRow("Min safe", self.spin_min_safe)
        form.addRow(self.btn_apply_tuning)
        return box

    def _build_sequence_editor_group(self) -> QtWidgets.QGroupBox:
        box = QtWidgets.QGroupBox("Sequence editor")
        layout = QtWidgets.QVBoxLayout(box)
        layout.setSpacing(8)

        self.table = QtWidgets.QTableWidget(0, 10)
        self.table.setHorizontalHeaderLabels(
            [
                "On",
                "Name",
                "Chamber",
                "Action",
                "Target",
                "Min s",
                "Max s",
                "Tol",
                "Settle s",
                "Snap ms",
            ]
        )
        self.table.setAlternatingRowColors(True)
        self.table.verticalHeader().setVisible(False)
        self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QtWidgets.QAbstractItemView.ExtendedSelection)
        self.table.setEditTriggers(
            QtWidgets.QAbstractItemView.DoubleClicked
            | QtWidgets.QAbstractItemView.EditKeyPressed
            | QtWidgets.QAbstractItemView.SelectedClicked
        )

        header = self.table.horizontalHeader()
        header.setSectionResizeMode(COL_ENABLED, QtWidgets.QHeaderView.ResizeToContents)
        header.setSectionResizeMode(COL_NAME, QtWidgets.QHeaderView.Stretch)
        header.setSectionResizeMode(COL_CHAMBER, QtWidgets.QHeaderView.ResizeToContents)
        header.setSectionResizeMode(COL_ACTION, QtWidgets.QHeaderView.ResizeToContents)
        for idx in (COL_TARGET, COL_MIN, COL_MAX, COL_TOL, COL_SETTLE, COL_SNAP):
            header.setSectionResizeMode(idx, QtWidgets.QHeaderView.ResizeToContents)

        row_btns = QtWidgets.QHBoxLayout()
        self.btn_add_row = QtWidgets.QPushButton("Add")
        self.btn_remove_row = QtWidgets.QPushButton("Remove")
        self.btn_duplicate_row = QtWidgets.QPushButton("Duplicate")
        self.btn_move_up = QtWidgets.QPushButton("Up")
        self.btn_move_down = QtWidgets.QPushButton("Down")
        self.btn_clear_rows = QtWidgets.QPushButton("Clear")

        self.btn_add_row.clicked.connect(self._on_add_row)
        self.btn_remove_row.clicked.connect(self._on_remove_rows)
        self.btn_duplicate_row.clicked.connect(self._on_duplicate_rows)
        self.btn_move_up.clicked.connect(self._on_move_up)
        self.btn_move_down.clicked.connect(self._on_move_down)
        self.btn_clear_rows.clicked.connect(self._on_clear_rows)

        for button in (
            self.btn_add_row,
            self.btn_remove_row,
            self.btn_duplicate_row,
            self.btn_move_up,
            self.btn_move_down,
            self.btn_clear_rows,
        ):
            row_btns.addWidget(button)
        row_btns.addStretch(1)

        file_btns = QtWidgets.QHBoxLayout()
        self.btn_import_json = QtWidgets.QPushButton("Import JSON")
        self.btn_export_json = QtWidgets.QPushButton("Export JSON")
        self.btn_import_json.clicked.connect(self._on_import_json)
        self.btn_export_json.clicked.connect(self._on_export_json)
        file_btns.addWidget(self.btn_import_json)
        file_btns.addWidget(self.btn_export_json)
        file_btns.addStretch(1)

        layout.addLayout(row_btns)
        layout.addWidget(self.table, 1)
        layout.addLayout(file_btns)
        return box

    def _build_plot_group(self) -> QtWidgets.QGroupBox:
        box = QtWidgets.QGroupBox("Live telemetry")
        layout = QtWidgets.QVBoxLayout(box)

        self.plot = pg.PlotWidget()
        self.plot.setBackground("#0b1221")
        self.plot.showGrid(x=True, y=True, alpha=0.25)
        self.plot.setLabel("left", "Pressure", units="kPa")
        self.plot.setLabel("bottom", "Time", units="s")

        self.curve_pressure = self.plot.plot(
            pen=pg.mkPen(color="#38bdf8", width=2),
            name="pressure",
        )
        self.curve_target = self.plot.plot(
            pen=pg.mkPen(color="#f59e0b", width=2, style=QtCore.Qt.DashLine),
            name="target",
        )

        legend = self.plot.addLegend(offset=(10, 10))
        legend.setParentItem(self.plot.getPlotItem())

        layout.addWidget(self.plot)
        return box

    def _build_log_group(self) -> QtWidgets.QGroupBox:
        box = QtWidgets.QGroupBox("Event log")
        layout = QtWidgets.QVBoxLayout(box)
        self.log_text = QtWidgets.QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setLineWrapMode(QtWidgets.QTextEdit.NoWrap)
        layout.addWidget(self.log_text)
        return box

    def _on_tick_interval_changed(self, value: int) -> None:
        self.control_timer.setInterval(int(value))

    def _numeric_item(self, value: float) -> QtWidgets.QTableWidgetItem:
        return QtWidgets.QTableWidgetItem(f"{float(value):.3f}")

    def _int_item(self, value: int) -> QtWidgets.QTableWidgetItem:
        return QtWidgets.QTableWidgetItem(str(int(value)))

    def _insert_phase_row(self, phase: PhaseSpec, row: int | None = None) -> None:
        phase = phase.normalized()
        if row is None:
            row = self.table.rowCount()
        self.table.insertRow(row)

        enabled_item = QtWidgets.QTableWidgetItem("")
        enabled_item.setFlags(
            QtCore.Qt.ItemIsEnabled | QtCore.Qt.ItemIsSelectable | QtCore.Qt.ItemIsUserCheckable
        )
        enabled_item.setCheckState(QtCore.Qt.Checked if phase.enabled else QtCore.Qt.Unchecked)
        self.table.setItem(row, COL_ENABLED, enabled_item)

        self.table.setItem(row, COL_NAME, QtWidgets.QTableWidgetItem(phase.name))

        chamber_combo = QtWidgets.QComboBox()
        for label, value in CHAMBER_OPTIONS:
            chamber_combo.addItem(label, value)
        chamber_index = chamber_combo.findData(phase.chamber)
        chamber_combo.setCurrentIndex(max(0, chamber_index))
        self.table.setCellWidget(row, COL_CHAMBER, chamber_combo)

        action_combo = QtWidgets.QComboBox()
        for label, value in ACTION_OPTIONS:
            action_combo.addItem(label, value)
        action_index = action_combo.findData(phase.action)
        action_combo.setCurrentIndex(max(0, action_index))
        self.table.setCellWidget(row, COL_ACTION, action_combo)

        self.table.setItem(row, COL_TARGET, self._numeric_item(phase.target))
        self.table.setItem(row, COL_MIN, self._numeric_item(phase.min_time_s))
        self.table.setItem(row, COL_MAX, self._numeric_item(phase.max_time_s))
        self.table.setItem(row, COL_TOL, self._numeric_item(phase.tol_kpa))
        self.table.setItem(row, COL_SETTLE, self._numeric_item(phase.settle_s))
        self.table.setItem(row, COL_SNAP, self._int_item(phase.snap_ms))

    def _phase_from_row(self, row: int, strict: bool = True) -> PhaseSpec:
        def read_text(col: int, default: str = "") -> str:
            item = self.table.item(row, col)
            if item is None:
                return default
            return item.text().strip()

        def read_float(col: int, default: float = 0.0, strict_parse: bool = True) -> float:
            txt = read_text(col, "")
            if txt == "":
                return default
            try:
                return float(txt)
            except ValueError:
                if strict_parse:
                    raise ValueError(f"Row {row + 1}: invalid numeric value in column {col + 1}")
                return default

        enabled_item = self.table.item(row, COL_ENABLED)
        enabled = True
        if enabled_item is not None:
            enabled = enabled_item.checkState() == QtCore.Qt.Checked
        strict_values = bool(strict and enabled)

        chamber_combo = self.table.cellWidget(row, COL_CHAMBER)
        action_combo = self.table.cellWidget(row, COL_ACTION)
        if chamber_combo is None or action_combo is None:
            raise ValueError(f"Row {row + 1}: missing combo widget")

        chamber = int(chamber_combo.currentData())
        action = str(action_combo.currentData())

        phase = PhaseSpec(
            name=read_text(COL_NAME, f"phase_{row + 1}"),
            chamber=chamber,
            action=action,
            target=read_float(COL_TARGET, 0.0, strict_parse=strict_values),
            min_time_s=read_float(COL_MIN, 0.2, strict_parse=strict_values),
            max_time_s=read_float(COL_MAX, 1.5, strict_parse=strict_values),
            tol_kpa=read_float(COL_TOL, 3.0, strict_parse=strict_values),
            settle_s=read_float(COL_SETTLE, 0.05, strict_parse=strict_values),
            snap_ms=int(round(read_float(COL_SNAP, 0.0, strict_parse=strict_values))),
            enabled=enabled,
        )

        return phase.normalized()

    def _read_phases_from_table(self, strict: bool = True) -> list[PhaseSpec]:
        phases: list[PhaseSpec] = []
        for row in range(self.table.rowCount()):
            phases.append(self._phase_from_row(row, strict=strict))
        return phases

    def _load_phase_table(self, phases: list[PhaseSpec]) -> None:
        self.table.setRowCount(0)
        for phase in phases:
            self._insert_phase_row(phase)

    def _load_preset_to_table(self, preset_name: str) -> None:
        if preset_name not in PRESETS:
            raise ValueError(f"Unknown preset: {preset_name}")
        description, preset_phases = PRESETS[preset_name]
        self._load_phase_table(_clone_phases(preset_phases))
        self.input_seq_name.setText(preset_name.replace(" ", "_"))
        self._append_log(f"Preset loaded: {preset_name} | {description}")

    def _selected_rows(self) -> list[int]:
        model = self.table.selectionModel()
        if model is not None:
            rows = {index.row() for index in model.selectedRows()}
            if rows:
                return sorted(rows)
        rows_fallback = {item.row() for item in self.table.selectedItems()}
        return sorted(rows_fallback)

    def _on_add_row(self) -> None:
        selected = self._selected_rows()
        if selected:
            base = self._phase_from_row(selected[-1], strict=False)
        else:
            base = PhaseSpec(
                name="new_phase",
                chamber=CHAMBER_ABC,
                action=ACTION_PID_INFLATE,
                target=25.0,
                min_time_s=0.4,
                max_time_s=1.2,
                tol_kpa=3.0,
                settle_s=0.05,
                snap_ms=0,
                enabled=True,
            )
        base.name = f"{base.name}_copy"
        insert_at = selected[-1] + 1 if selected else self.table.rowCount()
        self._insert_phase_row(base, row=insert_at)

    def _on_remove_rows(self) -> None:
        rows = self._selected_rows()
        if not rows:
            return
        for row in reversed(rows):
            self.table.removeRow(row)

    def _on_duplicate_rows(self) -> None:
        rows = self._selected_rows()
        if not rows:
            return
        phases = self._read_phases_from_table(strict=False)
        insert_offset = 0
        for row in rows:
            clone = PhaseSpec(**phases[row].to_dict())
            clone.name = f"{clone.name}_copy"
            phases.insert(row + 1 + insert_offset, clone)
            insert_offset += 1
        self._load_phase_table(phases)

    def _on_move_up(self) -> None:
        rows = self._selected_rows()
        if not rows or rows[0] == 0:
            return
        phases = self._read_phases_from_table(strict=False)
        for row in rows:
            phases[row - 1], phases[row] = phases[row], phases[row - 1]
        self._load_phase_table(phases)
        self.table.clearSelection()
        for row in [r - 1 for r in rows]:
            self.table.selectRow(row)

    def _on_move_down(self) -> None:
        rows = self._selected_rows()
        if not rows or rows[-1] >= self.table.rowCount() - 1:
            return
        phases = self._read_phases_from_table(strict=False)
        for row in reversed(rows):
            phases[row + 1], phases[row] = phases[row], phases[row + 1]
        self._load_phase_table(phases)
        self.table.clearSelection()
        for row in [r + 1 for r in rows]:
            self.table.selectRow(row)

    def _on_clear_rows(self) -> None:
        self.table.setRowCount(0)

    def _on_load_preset(self) -> None:
        preset_name = self.combo_preset.currentText()
        try:
            self._load_preset_to_table(preset_name)
        except Exception as exc:
            self._show_error(str(exc))

    def _sequence_payload(self) -> dict:
        phases = self._read_phases_from_table(strict=True)
        payload = {
            "version": 1,
            "name": self.input_seq_name.text().strip() or "locomotion_sequence",
            "loop_count": int(self.spin_loops.value()),
            "created_at": datetime.utcnow().isoformat() + "Z",
            "supports_3_chambers": True,
            "phases": [phase.to_dict() for phase in phases],
        }
        return payload

    def _default_sequence_dir(self) -> str:
        path = os.path.join(BASE_DIR, "experiments", "sequences")
        os.makedirs(path, exist_ok=True)
        return path

    def _on_export_json(self) -> None:
        try:
            payload = self._sequence_payload()
        except Exception as exc:
            self._show_error(str(exc))
            return

        default_name = (self.input_seq_name.text().strip() or "locomotion_sequence") + ".json"
        default_path = os.path.join(self._default_sequence_dir(), default_name)
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self,
            "Export sequence",
            default_path,
            "JSON (*.json)",
        )
        if not path:
            return

        with open(path, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2)
        self._append_log(f"Sequence exported: {path}")

    def _on_import_json(self) -> None:
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self,
            "Import sequence",
            self._default_sequence_dir(),
            "JSON (*.json)",
        )
        if not path:
            return

        try:
            with open(path, "r", encoding="utf-8") as handle:
                payload = json.load(handle)
            phases_raw = payload.get("phases", [])
            if not phases_raw:
                raise ValueError("No phases found in JSON file.")
            phases = [PhaseSpec.from_dict(entry) for entry in phases_raw]
            self._load_phase_table(phases)

            name = str(payload.get("name", "")).strip()
            if name:
                self.input_seq_name.setText(name)
            loop_count = payload.get("loop_count")
            if loop_count is not None:
                self.spin_loops.setValue(max(0, int(loop_count)))

            self._append_log(f"Sequence imported: {path}")
        except Exception as exc:
            self._show_error(f"Import failed: {exc}")

    def _on_apply_tuning(self) -> None:
        try:
            self.bot.update_tuning(
                kp_pos=float(self.spin_kp_pos.value()),
                ki_pos=float(self.spin_ki_pos.value()),
                kp_neg=float(self.spin_kp_neg.value()),
                ki_neg=float(self.spin_ki_neg.value()),
                max_safe=float(self.spin_max_safe.value()),
                min_safe=float(self.spin_min_safe.value()),
            )
            self._append_log("Firmware tuning updated.")
        except Exception as exc:
            self._show_error(f"Failed to send tuning: {exc}")

    def _on_send_manual(self) -> None:
        if self.engine.snapshot.running:
            self._show_error("Stop the sequence before sending manual commands.")
            return

        chamber = int(self.combo_manual_chamber.currentData())
        action = str(self.combo_manual_action.currentData())
        value = float(self.spin_manual_value.value())

        try:
            phase = PhaseSpec(
                name="manual",
                chamber=chamber,
                action=action,
                target=value,
                min_time_s=0.0,
                max_time_s=0.2,
            ).normalized()

            target_applied = self.engine._apply_phase_command(phase)
            self._append_log(
                "Manual command sent: "
                f"{ACTION_LABELS.get(action, action)} ch={chamber} value={target_applied:.2f}"
            )
        except Exception as exc:
            self._show_error(f"Manual command failed: {exc}")

    def _on_start(self) -> None:
        try:
            phases = self._read_phases_from_table(strict=True)
            enabled_count = sum(1 for phase in phases if phase.enabled)
            if enabled_count == 0:
                raise ValueError("Enable at least one phase in the table.")

            self.engine.configure(phases=phases, loop_limit=int(self.spin_loops.value()))
            self.engine.start()
            self.btn_pause.setText("Pause")
        except Exception as exc:
            self._show_error(str(exc))

    def _on_pause_resume(self) -> None:
        if not self.engine.snapshot.running:
            return

        if self.engine.snapshot.paused:
            self.engine.resume()
            self.btn_pause.setText("Pause")
        else:
            self.engine.pause()
            self.btn_pause.setText("Resume")

    def _on_stop(self) -> None:
        self.engine.stop(reason="manual stop")
        self.btn_pause.setText("Pause")

    def _on_estop(self) -> None:
        self.engine.stop(estop=True)
        self.btn_pause.setText("Pause")

    def _on_control_tick(self) -> None:
        try:
            snapshot = self.engine.tick()
        except Exception as exc:
            self._append_log(f"Engine error: {exc}")
            self.engine.stop(reason="runtime error")
            snapshot = self.engine.snapshot

        self._update_status(snapshot)
        self._capture_telemetry(snapshot)

    def _update_status(self, snapshot: EngineSnapshot) -> None:
        if snapshot.running and snapshot.paused:
            state = "PAUSED"
        elif snapshot.running:
            state = "RUNNING"
        else:
            state = "IDLE"

        self.label_status.setText(state)
        self.label_phase.setText(
            f"{snapshot.phase_index + 1 if snapshot.phase_index >= 0 else '-'} | {snapshot.phase_name}"
        )
        loop_text = "inf" if snapshot.loop_limit == 0 else str(snapshot.loop_limit)
        self.label_loop.setText(f"{snapshot.loops_done}/{loop_text}")
        self.label_pressure.setText(f"{snapshot.pressure_kpa:.2f} kPa")
        self.label_error.setText(f"{snapshot.error_kpa:.2f}")

        if snapshot.running and snapshot.phase_index >= 0:
            self.table.blockSignals(True)
            self.table.selectRow(snapshot.phase_index)
            self.table.blockSignals(False)

    def _capture_telemetry(self, snapshot: EngineSnapshot) -> None:
        t_s = max(0.0, time.monotonic() - self.telemetry_start_mono)
        target = float(snapshot.target_kpa if snapshot.running else 0.0)

        self.time_hist.append(t_s)
        self.pressure_hist.append(float(snapshot.pressure_kpa))
        self.target_hist.append(target)

        self.telemetry_rows.append(
            (
                datetime.utcnow().isoformat() + "Z",
                t_s,
                float(snapshot.pressure_kpa),
                target,
                int(snapshot.chamber),
                str(snapshot.action),
                str(snapshot.phase_name),
                float(snapshot.phase_elapsed_s),
                float(snapshot.total_elapsed_s),
                int(snapshot.loops_done),
                int(snapshot.loop_limit),
            )
        )

        self._plot_refresh_counter += 1
        if self._plot_refresh_counter % 2 == 0:
            self.curve_pressure.setData(list(self.time_hist), list(self.pressure_hist))
            self.curve_target.setData(list(self.time_hist), list(self.target_hist))

    def _export_telemetry_csv(self) -> None:
        if not self.telemetry_rows:
            self._show_error("No telemetry samples captured yet.")
            return

        default_name = (
            (self.input_seq_name.text().strip() or "locomotion")
            + "_telemetry_"
            + datetime.now().strftime("%Y%m%d_%H%M%S")
            + ".csv"
        )
        default_path = os.path.join(self._default_sequence_dir(), default_name)
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self,
            "Export telemetry CSV",
            default_path,
            "CSV (*.csv)",
        )
        if not path:
            return

        import csv

        with open(path, "w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(
                [
                    "timestamp_utc",
                    "t_s",
                    "pressure_kpa",
                    "target_kpa",
                    "chamber_mask",
                    "action",
                    "phase_name",
                    "phase_elapsed_s",
                    "sequence_elapsed_s",
                    "loops_done",
                    "loop_limit",
                ]
            )
            writer.writerows(self.telemetry_rows)

        self._append_log(f"Telemetry CSV exported: {path}")

    def _append_log(self, message: str) -> None:
        stamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.append(f"[{stamp}] {message}")
        self.log_text.moveCursor(QtGui.QTextCursor.End)

    def _show_error(self, message: str) -> None:
        self._append_log(f"ERROR: {message}")
        QtWidgets.QMessageBox.critical(self, "Locomotion GUI", message)

    def closeEvent(self, event):  # type: ignore[override]
        self.shutdown()
        event.accept()

    def shutdown(self) -> None:
        if self._shutdown_done:
            return
        self._shutdown_done = True

        try:
            self.control_timer.stop()
        except Exception:
            pass

        try:
            self.engine.stop(reason="gui closing")
        except Exception:
            pass

        try:
            self.bot.close()
        except Exception:
            pass

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


def main() -> int:
    rclpy.init(args=None)
    bot = SoftBot()

    pg.setConfigOptions(antialias=True)

    app = QtWidgets.QApplication(sys.argv)
    app.setApplicationName("SoftBot Locomotion Studio")

    window = LocomotionGUI(bot)
    window.show()

    exec_fn = getattr(app, "exec", None)
    if exec_fn is None:
        exec_fn = app.exec_  # type: ignore[attr-defined]
    code = int(exec_fn())

    if not window._shutdown_done:
        window.shutdown()
    return code


if __name__ == "__main__":
    raise SystemExit(main())

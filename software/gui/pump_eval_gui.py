#!/usr/bin/env python3
"""Dedicated GUI to evaluate pneumatic pumps using dual pressure/vacuum protocol."""

from __future__ import annotations

import csv
import json
import os
import shutil
import sys
from collections import deque
from typing import Any

try:
    from PySide6 import QtCore, QtWidgets
except Exception:
    from PyQt5 import QtCore, QtWidgets

try:
    import pyqtgraph as pg
except Exception as exc:
    raise SystemExit(
        "Falta pyqtgraph. Instala con: python3 -m pip install pyqtgraph PySide6"
    ) from exc

BASE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
CORE_SCRIPT = os.path.join(BASE_DIR, "software", "tools", "pump_eval_core.py")
EVENT_PREFIX = "[pump_eval_event]"


class PumpEvalGUI(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        self.process: QtCore.QProcess | None = None
        self.process_buffer = ""

        self.t_values: deque[float] = deque(maxlen=12000)
        self.raw_values: deque[float] = deque(maxlen=12000)
        self.filtered_values: deque[float] = deque(maxlen=12000)
        self.markers: list[Any] = []
        self.history_curves: list[Any] = []
        self.history_rows: list[dict[str, str]] = []

        self.last_summary_csv: str | None = None
        self.last_raw_csv: str | None = None
        self.safety_break_triggered = False

        self.setWindowTitle("Pump Evaluator GUI (selección de cámaras)")
        self.resize(1400, 860)

        self._build_ui()
        self._set_plot_live_mode()
        self._update_idle_state()
        self.refresh_history()

    def _build_ui(self) -> None:
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)

        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(10)

        top_split = QtWidgets.QHBoxLayout()
        top_split.setSpacing(12)
        top_split.addWidget(self._build_config_panel(), 2)
        top_split.addWidget(self._build_live_panel(), 3)
        root.addLayout(top_split, 4)

        root.addWidget(self._build_plot_panel(), 3)

        bottom_split = QtWidgets.QHBoxLayout()
        bottom_split.setSpacing(12)
        bottom_split.addWidget(self._build_history_panel(), 2)
        bottom_split.addWidget(self._build_log_panel(), 3)
        root.addLayout(bottom_split, 2)

    def _build_config_panel(self) -> QtWidgets.QWidget:
        box = QtWidgets.QGroupBox("Configuración evaluación")
        form = QtWidgets.QFormLayout(box)
        form.setLabelAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)
        form.setFormAlignment(QtCore.Qt.AlignTop)
        form.setFieldGrowthPolicy(QtWidgets.QFormLayout.AllNonFixedFieldsGrow)
        form.setHorizontalSpacing(14)
        form.setVerticalSpacing(8)

        self.input_pump_label = QtWidgets.QLineEdit("actuales")

        self.cb_chamber_a = QtWidgets.QCheckBox("A")
        self.cb_chamber_b = QtWidgets.QCheckBox("B")
        self.cb_chamber_c = QtWidgets.QCheckBox("C")
        self.cb_chamber_a.setChecked(True)
        self.cb_chamber_b.setChecked(True)
        self.cb_chamber_c.setChecked(True)
        chamber_widget = QtWidgets.QWidget()
        chamber_layout = QtWidgets.QHBoxLayout(chamber_widget)
        chamber_layout.setContentsMargins(0, 0, 0, 0)
        chamber_layout.setSpacing(12)
        chamber_layout.addWidget(self.cb_chamber_a)
        chamber_layout.addWidget(self.cb_chamber_b)
        chamber_layout.addWidget(self.cb_chamber_c)
        chamber_layout.addStretch(1)

        self.spin_kp_pos = QtWidgets.QDoubleSpinBox()
        self.spin_kp_pos.setRange(-2000.0, 2000.0)
        self.spin_kp_pos.setDecimals(3)
        self.spin_kp_pos.setValue(24.0)

        self.spin_ki_pos = QtWidgets.QDoubleSpinBox()
        self.spin_ki_pos.setRange(-10000.0, 10000.0)
        self.spin_ki_pos.setDecimals(3)
        self.spin_ki_pos.setValue(1500.0)

        self.spin_kp_neg = QtWidgets.QDoubleSpinBox()
        self.spin_kp_neg.setRange(-2000.0, 2000.0)
        self.spin_kp_neg.setDecimals(3)
        self.spin_kp_neg.setValue(-75.0)

        self.spin_ki_neg = QtWidgets.QDoubleSpinBox()
        self.spin_ki_neg.setRange(-10000.0, 10000.0)
        self.spin_ki_neg.setDecimals(3)
        self.spin_ki_neg.setValue(-750.0)

        self.spin_target_pressure = QtWidgets.QDoubleSpinBox()
        self.spin_target_pressure.setRange(1.0, 80.0)
        self.spin_target_pressure.setDecimals(2)
        self.spin_target_pressure.setValue(25.0)
        self.spin_target_pressure.setSuffix(" kPa")

        self.spin_target_vacuum = QtWidgets.QDoubleSpinBox()
        self.spin_target_vacuum.setRange(-80.0, -0.5)
        self.spin_target_vacuum.setDecimals(2)
        self.spin_target_vacuum.setValue(-15.0)
        self.spin_target_vacuum.setSuffix(" kPa")

        self.spin_safety_max = QtWidgets.QDoubleSpinBox()
        self.spin_safety_max.setRange(1.0, 120.0)
        self.spin_safety_max.setDecimals(2)
        self.spin_safety_max.setValue(45.0)
        self.spin_safety_max.setSuffix(" kPa")

        self.spin_safety_min = QtWidgets.QDoubleSpinBox()
        self.spin_safety_min.setRange(-120.0, -1.0)
        self.spin_safety_min.setDecimals(2)
        self.spin_safety_min.setValue(-45.0)
        self.spin_safety_min.setSuffix(" kPa")

        self.spin_pwm_capacity = QtWidgets.QSpinBox()
        self.spin_pwm_capacity.setRange(0, 255)
        self.spin_pwm_capacity.setValue(220)

        self.spin_timeout_phase = QtWidgets.QDoubleSpinBox()
        self.spin_timeout_phase.setRange(0.2, 15.0)
        self.spin_timeout_phase.setDecimals(2)
        self.spin_timeout_phase.setValue(3.0)
        self.spin_timeout_phase.setSuffix(" s")

        self.spin_runs = QtWidgets.QSpinBox()
        self.spin_runs.setRange(1, 50)
        self.spin_runs.setValue(3)

        self.spin_sample_ms = QtWidgets.QSpinBox()
        self.spin_sample_ms.setRange(5, 500)
        self.spin_sample_ms.setValue(20)
        self.spin_sample_ms.setSuffix(" ms")

        self.spin_vent_s = QtWidgets.QDoubleSpinBox()
        self.spin_vent_s.setRange(0.0, 10.0)
        self.spin_vent_s.setDecimals(2)
        self.spin_vent_s.setValue(0.6)
        self.spin_vent_s.setSuffix(" s")

        self.spin_rest_s = QtWidgets.QDoubleSpinBox()
        self.spin_rest_s.setRange(0.0, 10.0)
        self.spin_rest_s.setDecimals(2)
        self.spin_rest_s.setValue(0.4)
        self.spin_rest_s.setSuffix(" s")

        self.spin_filter_window = QtWidgets.QSpinBox()
        self.spin_filter_window.setRange(1, 21)
        self.spin_filter_window.setSingleStep(2)
        self.spin_filter_window.setValue(5)
        self.spin_filter_window.setToolTip("Ventana del filtro mediana (se recomienda impar)")

        self.input_registry_csv = QtWidgets.QLineEdit("experiments/pump_eval_registry.csv")

        self.cb_demo = QtWidgets.QCheckBox("Modo demo (sin hardware)")
        self.cb_demo.setChecked(False)

        self.btn_start = QtWidgets.QPushButton("Iniciar")
        self.btn_cancel = QtWidgets.QPushButton("Cancelar")
        self.btn_export = QtWidgets.QPushButton("Exportar")
        self.btn_refresh_history = QtWidgets.QPushButton("Refrescar histórico")
        self.btn_cancel.setEnabled(False)
        self.btn_export.setEnabled(False)

        self.btn_start.clicked.connect(self.on_start)
        self.btn_cancel.clicked.connect(self.on_cancel)
        self.btn_export.clicked.connect(self.on_export)
        self.btn_refresh_history.clicked.connect(self.refresh_history)

        self.label_run_status = QtWidgets.QLabel("Estado: listo")
        self.label_run_status.setWordWrap(True)

        buttons = QtWidgets.QHBoxLayout()
        buttons.setSpacing(10)
        buttons.addWidget(self.btn_start)
        buttons.addWidget(self.btn_cancel)
        buttons.addWidget(self.btn_export)

        controls = [
            self.input_pump_label,
            self.spin_kp_pos,
            self.spin_ki_pos,
            self.spin_kp_neg,
            self.spin_ki_neg,
            self.spin_target_pressure,
            self.spin_target_vacuum,
            self.spin_safety_max,
            self.spin_safety_min,
            self.spin_pwm_capacity,
            self.spin_timeout_phase,
            self.spin_runs,
            self.spin_sample_ms,
            self.spin_vent_s,
            self.spin_rest_s,
            self.spin_filter_window,
            self.input_registry_csv,
            self.btn_start,
            self.btn_cancel,
            self.btn_export,
            self.btn_refresh_history,
        ]
        for control in controls:
            control.setMinimumHeight(30)
        for cb in (self.cb_chamber_a, self.cb_chamber_b, self.cb_chamber_c, self.cb_demo):
            cb.setMinimumHeight(26)

        form.addRow("Pump label", self.input_pump_label)
        form.addRow("Cámara(s) [A/B/C]", chamber_widget)
        form.addRow("Kp+", self.spin_kp_pos)
        form.addRow("Ki+", self.spin_ki_pos)
        form.addRow("Kp-", self.spin_kp_neg)
        form.addRow("Ki-", self.spin_ki_neg)
        form.addRow("Target presión", self.spin_target_pressure)
        form.addRow("Target vacío", self.spin_target_vacuum)
        form.addRow("Safety +max", self.spin_safety_max)
        form.addRow("Safety -min", self.spin_safety_min)
        form.addRow("PWM capacidad", self.spin_pwm_capacity)
        form.addRow("Timeout por fase", self.spin_timeout_phase)
        form.addRow("Runs", self.spin_runs)
        form.addRow("Sample", self.spin_sample_ms)
        form.addRow("Vent", self.spin_vent_s)
        form.addRow("Rest", self.spin_rest_s)
        form.addRow("Filtro mediana", self.spin_filter_window)
        form.addRow("Registry CSV", self.input_registry_csv)
        form.addRow(self.cb_demo)
        form.addRow(buttons)
        form.addRow(self.btn_refresh_history)
        form.addRow(self.label_run_status)

        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        scroll.setWidget(box)

        container = QtWidgets.QWidget()
        container_layout = QtWidgets.QVBoxLayout(container)
        container_layout.setContentsMargins(0, 0, 0, 0)
        container_layout.addWidget(scroll)
        return container

    def _selected_chamber_mask(self) -> int:
        mask = 0
        if self.cb_chamber_a.isChecked():
            mask |= 1
        if self.cb_chamber_b.isChecked():
            mask |= 2
        if self.cb_chamber_c.isChecked():
            mask |= 4
        return mask

    def _build_live_panel(self) -> QtWidgets.QGroupBox:
        box = QtWidgets.QGroupBox("Métricas en vivo")
        grid = QtWidgets.QGridLayout(box)

        self.card_top_pressure = QtWidgets.QLabel("-")
        self.card_time_top_pressure = QtWidgets.QLabel("-")
        self.card_top_vacuum = QtWidgets.QLabel("-")
        self.card_time_top_vacuum = QtWidgets.QLabel("-")
        self.card_ttarget_pressure = QtWidgets.QLabel("-")
        self.card_ttarget_vacuum = QtWidgets.QLabel("-")
        self.card_apta = QtWidgets.QLabel("-")
        self.card_score = QtWidgets.QLabel("-")
        self.card_phase = QtWidgets.QLabel("-")
        self.card_progress = QtWidgets.QLabel("-")

        labels = [
            self.card_top_pressure,
            self.card_time_top_pressure,
            self.card_top_vacuum,
            self.card_time_top_vacuum,
            self.card_ttarget_pressure,
            self.card_ttarget_vacuum,
            self.card_apta,
            self.card_score,
            self.card_phase,
            self.card_progress,
        ]
        for lbl in labels:
            lbl.setStyleSheet("font-family: monospace; font-weight: 600;")

        grid.addWidget(QtWidgets.QLabel("Tope presión"), 0, 0)
        grid.addWidget(self.card_top_pressure, 0, 1)
        grid.addWidget(QtWidgets.QLabel("Tiempo tope presión"), 1, 0)
        grid.addWidget(self.card_time_top_pressure, 1, 1)
        grid.addWidget(QtWidgets.QLabel("Tope vacío"), 2, 0)
        grid.addWidget(self.card_top_vacuum, 2, 1)
        grid.addWidget(QtWidgets.QLabel("Tiempo tope vacío"), 3, 0)
        grid.addWidget(self.card_time_top_vacuum, 3, 1)
        grid.addWidget(QtWidgets.QLabel("Tiempo target presión"), 4, 0)
        grid.addWidget(self.card_ttarget_pressure, 4, 1)
        grid.addWidget(QtWidgets.QLabel("Tiempo target vacío"), 5, 0)
        grid.addWidget(self.card_ttarget_vacuum, 5, 1)
        grid.addWidget(QtWidgets.QLabel("Estado"), 6, 0)
        grid.addWidget(self.card_apta, 6, 1)
        grid.addWidget(QtWidgets.QLabel("Score final"), 7, 0)
        grid.addWidget(self.card_score, 7, 1)
        grid.addWidget(QtWidgets.QLabel("Fase"), 8, 0)
        grid.addWidget(self.card_phase, 8, 1)
        grid.addWidget(QtWidgets.QLabel("Progreso"), 9, 0)
        grid.addWidget(self.card_progress, 9, 1)

        return box

    def _build_plot_panel(self) -> QtWidgets.QGroupBox:
        box = QtWidgets.QGroupBox("Curvas kPa (en vivo / histórico)")
        layout = QtWidgets.QVBoxLayout(box)

        pg.setConfigOptions(antialias=True)
        self.plot = pg.PlotWidget(title="Pump Evaluation")
        self.plot.setLabel("left", "kPa")
        self.plot.setLabel("bottom", "Tiempo sesión", "s")
        self.plot.addLegend(offset=(10, 10))

        self.curve_raw = self.plot.plot(
            pen=pg.mkPen("#00A6FB", width=1),
            name="Presión cruda",
        )
        self.curve_filtered = self.plot.plot(
            pen=pg.mkPen("#FFB703", width=2),
            name="Presión filtrada",
        )

        self.label_plot_context = QtWidgets.QLabel(
            "Modo gráfica: en vivo. Para comparar histórico, selecciona una o más filas abajo."
        )
        self.label_plot_context.setWordWrap(True)

        self.label_marker_legend = QtWidgets.QLabel(
            "Líneas verticales: azul=inicio fase, gris=fin fase, "
            "verde=cruce target, magenta=tope fase, rojo=safety break."
        )
        self.label_marker_legend.setWordWrap(True)

        layout.addWidget(self.plot)
        layout.addWidget(self.label_plot_context)
        layout.addWidget(self.label_marker_legend)
        return box

    def _build_history_panel(self) -> QtWidgets.QGroupBox:
        box = QtWidgets.QGroupBox("Histórico (solo APTA, score desc)")
        layout = QtWidgets.QVBoxLayout(box)

        self.history_table = QtWidgets.QTableWidget(0, 9)
        self.history_table.setHorizontalHeaderLabels(
            [
                "timestamp",
                "pump_label",
                "score_final",
                "topP_mean",
                "topV_mean",
                "tTargetP_mean",
                "tTargetV_mean",
                "runs",
                "summary_csv",
            ]
        )
        self.history_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.history_table.setSelectionMode(QtWidgets.QAbstractItemView.ExtendedSelection)
        self.history_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.history_table.horizontalHeader().setStretchLastSection(True)
        self.history_table.itemSelectionChanged.connect(self.on_history_selection_changed)

        self.label_history = QtWidgets.QLabel("Registros: 0")
        self.label_history_hint = QtWidgets.QLabel(
            "Tip: selecciona una o varias filas para cargar comparativo en la gráfica."
        )
        self.label_history_hint.setWordWrap(True)

        layout.addWidget(self.history_table)
        layout.addWidget(self.label_history)
        layout.addWidget(self.label_history_hint)
        return box

    def _build_log_panel(self) -> QtWidgets.QGroupBox:
        box = QtWidgets.QGroupBox("Salida proceso")
        layout = QtWidgets.QVBoxLayout(box)

        self.text_output = QtWidgets.QPlainTextEdit()
        self.text_output.setReadOnly(True)

        layout.addWidget(self.text_output)
        return box

    def _append_output(self, text: str) -> None:
        for line in text.replace("\r", "\n").splitlines():
            self.text_output.appendPlainText(line)
        self.text_output.ensureCursorVisible()

    def _set_running(self, running: bool) -> None:
        self.btn_start.setEnabled(not running)
        self.btn_cancel.setEnabled(running)

    def _is_process_running(self) -> bool:
        return bool(self.process and self.process.state() != QtCore.QProcess.NotRunning)

    def _set_plot_live_mode(self) -> None:
        self.curve_raw.show()
        self.curve_filtered.show()
        self.plot.setTitle("Pump Evaluation (en vivo)")
        self.label_plot_context.setText(
            "Modo gráfica: en vivo. Para comparar histórico, selecciona una o más filas abajo."
        )

    def _set_plot_history_mode(self, count: int) -> None:
        self.curve_raw.hide()
        self.curve_filtered.hide()
        self.plot.setTitle("Pump Evaluation (comparativo histórico)")
        self.label_plot_context.setText(
            f"Modo gráfica: histórico comparativo ({count} curva(s) filtradas). "
            "Selecciona varias filas para comparar."
        )

    def _clear_history_curves(self) -> None:
        for curve in self.history_curves:
            self.plot.removeItem(curve)
        self.history_curves = []

    def _update_idle_state(self) -> None:
        self.card_top_pressure.setText("-")
        self.card_time_top_pressure.setText("-")
        self.card_top_vacuum.setText("-")
        self.card_time_top_vacuum.setText("-")
        self.card_ttarget_pressure.setText("-")
        self.card_ttarget_vacuum.setText("-")
        self.card_apta.setText("-")
        self.card_score.setText("-")
        self.card_phase.setText("-")
        self.card_progress.setText("-")

    def _clear_plot(self) -> None:
        self.t_values.clear()
        self.raw_values.clear()
        self.filtered_values.clear()
        self.curve_raw.setData([], [])
        self.curve_filtered.setData([], [])
        self._clear_history_curves()
        for marker in self.markers:
            self.plot.removeItem(marker)
        self.markers = []

    def _add_marker(
        self,
        x_value: float,
        color: str,
        style: Any = QtCore.Qt.DashLine,
        width: float = 1.2,
    ) -> None:
        line = pg.InfiniteLine(
            pos=float(x_value),
            angle=90,
            pen=pg.mkPen(color, width=width, style=style),
            movable=False,
        )
        self.plot.addItem(line)
        self.markers.append(line)

    def _build_command(self) -> list[str]:
        python_bin = sys.executable
        chamber_mask = self._selected_chamber_mask()
        command = [
            python_bin,
            CORE_SCRIPT,
            "--pump-label",
            self.input_pump_label.text().strip(),
            "--chamber",
            str(chamber_mask),
            "--kp-pos",
            f"{self.spin_kp_pos.value():.4f}",
            "--ki-pos",
            f"{self.spin_ki_pos.value():.4f}",
            "--kp-neg",
            f"{self.spin_kp_neg.value():.4f}",
            "--ki-neg",
            f"{self.spin_ki_neg.value():.4f}",
            "--target-pressure-kpa",
            f"{self.spin_target_pressure.value():.4f}",
            "--target-vacuum-kpa",
            f"{self.spin_target_vacuum.value():.4f}",
            "--safety-max-kpa",
            f"{self.spin_safety_max.value():.4f}",
            "--safety-min-kpa",
            f"{self.spin_safety_min.value():.4f}",
            "--pwm-capacity",
            str(self.spin_pwm_capacity.value()),
            "--timeout-phase-s",
            f"{self.spin_timeout_phase.value():.4f}",
            "--runs",
            str(self.spin_runs.value()),
            "--sample-ms",
            str(self.spin_sample_ms.value()),
            "--vent-s",
            f"{self.spin_vent_s.value():.4f}",
            "--rest-s",
            f"{self.spin_rest_s.value():.4f}",
            "--filter-window",
            str(self.spin_filter_window.value()),
            "--registry-csv",
            self.input_registry_csv.text().strip(),
        ]

        if self.cb_demo.isChecked():
            command.append("--demo")

        return command

    def on_start(self) -> None:
        if self.process and self.process.state() != QtCore.QProcess.NotRunning:
            QtWidgets.QMessageBox.information(
                self,
                "Proceso activo",
                "Ya hay una evaluación en curso.",
            )
            return

        label = self.input_pump_label.text().strip()
        if not label:
            QtWidgets.QMessageBox.warning(
                self,
                "Label requerido",
                "Define pump_label antes de iniciar.",
            )
            return

        chamber_mask = self._selected_chamber_mask()
        if chamber_mask == 0:
            QtWidgets.QMessageBox.warning(
                self,
                "Cámara inválida",
                "Selecciona al menos una cámara (A, B o C).",
            )
            return
        if self.spin_safety_min.value() >= self.spin_safety_max.value():
            QtWidgets.QMessageBox.warning(
                self,
                "Safety inválido",
                "Safety -min debe ser menor que Safety +max.",
            )
            return

        if not os.path.exists(CORE_SCRIPT):
            QtWidgets.QMessageBox.critical(self, "Core missing", f"No existe: {CORE_SCRIPT}")
            return

        self._set_running(True)
        self.btn_export.setEnabled(False)
        self.label_run_status.setText("Estado: ejecutando...")
        self._set_plot_live_mode()
        self._clear_plot()
        self._update_idle_state()
        self.text_output.clear()
        self.process_buffer = ""
        self.last_summary_csv = None
        self.last_raw_csv = None
        self.safety_break_triggered = False

        command = self._build_command()
        self._append_output(" ".join(command))

        self.process = QtCore.QProcess(self)
        self.process.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        self.process.readyReadStandardOutput.connect(self._on_process_output)
        self.process.finished.connect(self._on_process_finished)
        self.process.errorOccurred.connect(self._on_process_error)
        self.process.start(command[0], command[1:])

        if not self.process.waitForStarted(1500):
            self._set_running(False)
            QtWidgets.QMessageBox.critical(
                self,
                "No se pudo iniciar",
                self.process.errorString(),
            )
            self.process = None

    def on_cancel(self) -> None:
        if not self.process:
            return
        if self.process.state() == QtCore.QProcess.NotRunning:
            return

        self._append_output("Cancelando evaluación...")
        self.process.terminate()
        if not self.process.waitForFinished(1500):
            self.process.kill()
            self.process.waitForFinished(1000)

    def on_export(self) -> None:
        if not self.last_summary_csv and not self.last_raw_csv:
            QtWidgets.QMessageBox.information(
                self,
                "Sin datos",
                "No hay archivos para exportar todavía.",
            )
            return

        target_dir = QtWidgets.QFileDialog.getExistingDirectory(
            self,
            "Selecciona carpeta de exportación",
            os.path.join(BASE_DIR, "experiments"),
        )
        if not target_dir:
            return

        copied: list[str] = []
        for path in [self.last_summary_csv, self.last_raw_csv]:
            if path and os.path.exists(path):
                dst = os.path.join(target_dir, os.path.basename(path))
                shutil.copy2(path, dst)
                copied.append(dst)

        if copied:
            self._append_output("Exportados:")
            for entry in copied:
                self._append_output(f"- {entry}")
            QtWidgets.QMessageBox.information(
                self,
                "Exportación",
                "Archivos exportados correctamente.",
            )

    def _summary_to_raw_path(self, summary_csv: str | None) -> str | None:
        summary_abs = self._to_abs_path(summary_csv)
        if not summary_abs:
            return None
        filename = os.path.basename(summary_abs)
        if not filename.startswith("pump_eval_summary_"):
            return None
        raw_name = filename.replace("pump_eval_summary_", "pump_eval_raw_", 1)
        raw_abs = os.path.join(os.path.dirname(summary_abs), raw_name)
        if os.path.exists(raw_abs):
            return raw_abs
        return None

    def _load_raw_series(self, raw_csv_path: str) -> dict[str, Any] | None:
        if not raw_csv_path or not os.path.exists(raw_csv_path):
            return None

        t_values: list[float] = []
        raw_values: list[float] = []
        filtered_values: list[float] = []
        phase_starts: list[tuple[str, float]] = []
        phase_ends: list[tuple[str, float]] = []

        prev_phase = ""
        prev_t = 0.0
        has_prev = False

        with open(raw_csv_path, newline="", encoding="utf-8") as handle:
            reader = csv.DictReader(handle)
            for row in reader:
                try:
                    t_val = float((row.get("t_session_s", "") or "").strip())
                    raw_val = float((row.get("pressure_raw_kpa", "") or "").strip())
                    filtered_val = float((row.get("pressure_filtered_kpa", "") or "").strip())
                except (TypeError, ValueError):
                    continue

                phase = str((row.get("phase", "") or "").strip())
                if not has_prev:
                    phase_starts.append((phase, t_val))
                    has_prev = True
                elif phase != prev_phase:
                    phase_ends.append((prev_phase, prev_t))
                    phase_starts.append((phase, t_val))

                t_values.append(t_val)
                raw_values.append(raw_val)
                filtered_values.append(filtered_val)
                prev_phase = phase
                prev_t = t_val

        if has_prev:
            phase_ends.append((prev_phase, prev_t))

        if not t_values:
            return None

        return {
            "t": t_values,
            "raw": raw_values,
            "filtered": filtered_values,
            "phase_starts": phase_starts,
            "phase_ends": phase_ends,
        }

    def on_history_selection_changed(self) -> None:
        if self._is_process_running():
            return

        selection_model = self.history_table.selectionModel()
        if not selection_model:
            return

        selected_rows = sorted(index.row() for index in selection_model.selectedRows())
        if not selected_rows:
            self._clear_plot()
            self._set_plot_live_mode()
            return

        max_rows = 6
        if len(selected_rows) > max_rows:
            selected_rows = selected_rows[:max_rows]
            self._append_output(
                f"Comparativo histórico limitado a {max_rows} filas para mantener claridad."
            )

        self._clear_plot()
        self._set_plot_history_mode(len(selected_rows))

        palette = [
            "#FFB703",
            "#00A6FB",
            "#FB5607",
            "#2A9D8F",
            "#8338EC",
            "#EF476F",
        ]

        loaded_count = 0
        first_loaded: dict[str, Any] | None = None
        for idx, row_idx in enumerate(selected_rows):
            if row_idx < 0 or row_idx >= len(self.history_rows):
                continue
            row = self.history_rows[row_idx]
            summary_csv = row.get("summary_csv", "")
            raw_csv = self._summary_to_raw_path(summary_csv)
            if not raw_csv:
                continue

            series = self._load_raw_series(raw_csv)
            if not series:
                continue

            color = palette[idx % len(palette)]
            label = row.get("pump_label", "") or f"row_{row_idx + 1}"
            stamp = row.get("timestamp", "") or "sin_fecha"
            curve = self.plot.plot(
                series["t"],
                series["filtered"],
                pen=pg.mkPen(color, width=2),
                name=f"{label} | {stamp}",
            )
            self.history_curves.append(curve)
            loaded_count += 1
            if first_loaded is None:
                first_loaded = series

        if loaded_count == 0:
            self.label_plot_context.setText(
                "Modo gráfica: histórico comparativo. No se encontraron CSV raw para la selección."
            )
            return

        # Add phase boundaries from first selected row to avoid clutter.
        if first_loaded is not None:
            for _, t_val in first_loaded["phase_starts"]:
                self._add_marker(
                    t_val,
                    "#3A86FF",
                    style=QtCore.Qt.DashLine,
                    width=1.1,
                )
            for _, t_val in first_loaded["phase_ends"]:
                self._add_marker(
                    t_val,
                    "#6C757D",
                    style=QtCore.Qt.DotLine,
                    width=1.0,
                )

        self.label_plot_context.setText(
            f"Modo gráfica: histórico comparativo ({loaded_count} curva(s) cargadas). "
            "Selecciona varias filas para comparar visualmente."
        )

    def _on_process_output(self) -> None:
        if not self.process:
            return
        chunk = bytes(self.process.readAllStandardOutput()).decode("utf-8", errors="replace")
        if not chunk:
            return

        self.process_buffer += chunk
        while "\n" in self.process_buffer:
            line, self.process_buffer = self.process_buffer.split("\n", 1)
            line = line.strip()
            if not line:
                continue
            self._append_output(line)
            self._consume_line(line)

    def _consume_line(self, line: str) -> None:
        if not line.startswith(EVENT_PREFIX):
            return
        payload_text = line[len(EVENT_PREFIX) :].strip()
        try:
            payload = json.loads(payload_text)
        except json.JSONDecodeError:
            return
        self._handle_event(payload)

    def _to_abs_path(self, maybe_rel: str | None) -> str | None:
        if not maybe_rel:
            return None
        if os.path.isabs(maybe_rel):
            return maybe_rel
        return os.path.join(BASE_DIR, maybe_rel)

    @staticmethod
    def _fmt_num(value: Any, suffix: str = "") -> str:
        if value is None:
            return "N/A"
        try:
            return f"{float(value):.3f}{suffix}"
        except (TypeError, ValueError):
            return "N/A"

    def _handle_event(self, payload: dict[str, Any]) -> None:
        event = payload.get("event", "")

        if event == "session_start":
            self.card_progress.setText(f"run 0/{payload.get('runs', '?')}")
            self.card_phase.setText("init")
            return

        if event == "run_start":
            self.card_progress.setText(
                f"run {payload.get('run_idx', '?')}/{payload.get('runs', '?')}"
            )
            return

        if event == "phase_start":
            phase = str(payload.get("phase", "?"))
            self.card_phase.setText(phase)
            t = payload.get("t_session_s")
            if t is not None:
                self._add_marker(
                    float(t),
                    "#3A86FF",
                    style=QtCore.Qt.DashLine,
                    width=1.2,
                )
            return

        if event == "sample":
            t = payload.get("t_session_s")
            raw = payload.get("pressure_raw_kpa")
            filtered = payload.get("pressure_filtered_kpa")
            if t is None or raw is None or filtered is None:
                return
            self.t_values.append(float(t))
            self.raw_values.append(float(raw))
            self.filtered_values.append(float(filtered))
            self.curve_raw.setData(list(self.t_values), list(self.raw_values))
            self.curve_filtered.setData(list(self.t_values), list(self.filtered_values))
            return

        if event == "target_cross":
            t = payload.get("t_session_s")
            if t is not None:
                self._add_marker(
                    float(t),
                    "#2A9D8F",
                    style=QtCore.Qt.DashDotLine,
                    width=1.3,
                )
            return

        if event == "safety_break":
            self.safety_break_triggered = True
            t = payload.get("t_session_s")
            if t is not None:
                self._add_marker(
                    float(t),
                    "#E63946",
                    style=QtCore.Qt.SolidLine,
                    width=1.8,
                )
            self.card_phase.setText("SAFETY_BREAK")
            self.card_apta.setText("NO_APTA")
            pressure_txt = self._fmt_num(payload.get("pressure_filtered_kpa"), " kPa")
            self.label_run_status.setText(f"Estado: SAFETY_BREAK ({pressure_txt})")
            return

        if event == "phase_end":
            end_t = payload.get("end_session_s")
            if end_t is not None:
                self._add_marker(
                    float(end_t),
                    "#6C757D",
                    style=QtCore.Qt.DotLine,
                    width=1.1,
                )
            peak_t = payload.get("time_to_top_session_s")
            if peak_t is not None:
                self._add_marker(
                    float(peak_t),
                    "#EF476F",
                    style=QtCore.Qt.DashDotDotLine,
                    width=1.2,
                )
            return

        if event == "run_end":
            self.card_top_pressure.setText(self._fmt_num(payload.get("top_pressure_kpa"), " kPa"))
            self.card_time_top_pressure.setText(
                self._fmt_num(payload.get("time_to_top_pressure_s"), " s")
            )
            self.card_top_vacuum.setText(self._fmt_num(payload.get("top_vacuum_kpa"), " kPa"))
            self.card_time_top_vacuum.setText(
                self._fmt_num(payload.get("time_to_top_vacuum_s"), " s")
            )
            self.card_ttarget_pressure.setText(
                self._fmt_num(payload.get("time_to_target_pressure_s"), " s")
            )
            self.card_ttarget_vacuum.setText(
                self._fmt_num(payload.get("time_to_target_vacuum_s"), " s")
            )
            valid = bool(payload.get("valid", False))
            self.card_apta.setText("APTA" if valid else "NO_APTA")
            return

        if event == "session_end":
            self.card_apta.setText("APTA" if bool(payload.get("apta", False)) else "NO_APTA")
            self.card_score.setText(self._fmt_num(payload.get("score_final"), ""))
            self.card_top_pressure.setText(
                self._fmt_num(payload.get("top_pressure_mean_kpa"), " kPa")
            )
            self.card_top_vacuum.setText(self._fmt_num(payload.get("top_vacuum_mean_kpa"), " kPa"))
            self.card_time_top_pressure.setText(
                self._fmt_num(payload.get("time_to_top_pressure_mean_s"), " s")
            )
            self.card_time_top_vacuum.setText(
                self._fmt_num(payload.get("time_to_top_vacuum_mean_s"), " s")
            )
            self.card_ttarget_pressure.setText(
                self._fmt_num(payload.get("time_to_target_pressure_mean_s"), " s")
            )
            self.card_ttarget_vacuum.setText(
                self._fmt_num(payload.get("time_to_target_vacuum_mean_s"), " s")
            )
            self.last_summary_csv = self._to_abs_path(payload.get("summary_csv"))
            self.last_raw_csv = self._to_abs_path(payload.get("raw_csv"))
            self.btn_export.setEnabled(bool(self.last_summary_csv or self.last_raw_csv))
            return

        if event == "error":
            message = payload.get("message", "unknown")
            if payload.get("kind") == "safety_break":
                self.safety_break_triggered = True
                self.card_phase.setText("SAFETY_BREAK")
                self.card_apta.setText("NO_APTA")
            self.label_run_status.setText(f"Estado: error ({message})")
            return

    def _on_process_finished(self, exit_code: int, _exit_status: Any) -> None:
        self._on_process_output()
        self._set_running(False)
        if self.safety_break_triggered:
            status = "SAFETY_BREAK"
        else:
            status = "OK" if exit_code == 0 else f"ERROR ({exit_code})"
        self.label_run_status.setText(f"Estado: finalizado {status}")
        self.process = None
        self.refresh_history()

    def _on_process_error(self, _error: Any) -> None:
        if not self.process:
            return
        self._append_output(f"[qprocess] {self.process.errorString()}")

    def refresh_history(self) -> None:
        path_text = self.input_registry_csv.text().strip() or "experiments/pump_eval_registry.csv"
        path = path_text if os.path.isabs(path_text) else os.path.join(BASE_DIR, path_text)

        if not os.path.exists(path):
            self.history_rows = []
            self.history_table.setRowCount(0)
            self.label_history.setText("Registros: 0 (sin archivo)")
            return

        rows: list[dict[str, str]] = []
        with open(path, newline="", encoding="utf-8") as handle:
            reader = csv.DictReader(handle)
            for row in reader:
                if (row.get("apta", "") or "0").strip() not in {"1", "true", "True"}:
                    continue
                rows.append(row)

        rows.sort(
            key=lambda item: (
                self._row_float(item, "score_final") is None,
                -(self._row_float(item, "score_final") or 0.0),
            )
        )

        self.history_rows = rows
        self.history_table.setRowCount(len(rows))
        current_label = self.input_pump_label.text().strip()
        for idx, row in enumerate(rows):
            values = [
                row.get("timestamp", ""),
                row.get("pump_label", ""),
                row.get("score_final", ""),
                row.get("top_pressure_mean_kpa", ""),
                row.get("top_vacuum_mean_kpa", ""),
                row.get("time_to_target_pressure_mean_s", ""),
                row.get("time_to_target_vacuum_mean_s", ""),
                row.get("runs", ""),
                row.get("summary_csv", ""),
            ]
            for col, value in enumerate(values):
                item = QtWidgets.QTableWidgetItem(str(value))
                if current_label and row.get("pump_label", "") == current_label:
                    item.setBackground(QtCore.Qt.yellow)
                self.history_table.setItem(idx, col, item)

        self.label_history.setText(f"Registros: {len(rows)}")

    @staticmethod
    def _row_float(row: dict[str, str], key: str) -> float | None:
        value = (row.get(key, "") or "").strip()
        if not value:
            return None
        try:
            return float(value)
        except ValueError:
            return None

    def closeEvent(self, event: Any) -> None:
        try:
            self.on_cancel()
        except Exception:
            pass
        event.accept()


def main() -> int:
    app = QtWidgets.QApplication(sys.argv)
    window = PumpEvalGUI()
    window.show()
    return int(app.exec())


if __name__ == "__main__":
    raise SystemExit(main())

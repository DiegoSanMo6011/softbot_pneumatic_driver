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

        self.last_summary_csv: str | None = None
        self.last_raw_csv: str | None = None

        self.setWindowTitle("Pump Evaluator GUI (ABC fijo)")
        self.resize(1400, 860)

        self._build_ui()
        self._update_idle_state()
        self.refresh_history()

    def _build_ui(self) -> None:
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)

        root = QtWidgets.QVBoxLayout(central)

        top_split = QtWidgets.QHBoxLayout()
        top_split.addWidget(self._build_config_panel(), 2)
        top_split.addWidget(self._build_live_panel(), 3)
        root.addLayout(top_split, 2)

        root.addWidget(self._build_plot_panel(), 3)

        bottom_split = QtWidgets.QHBoxLayout()
        bottom_split.addWidget(self._build_history_panel(), 2)
        bottom_split.addWidget(self._build_log_panel(), 3)
        root.addLayout(bottom_split, 2)

    def _build_config_panel(self) -> QtWidgets.QGroupBox:
        box = QtWidgets.QGroupBox("Configuración evaluación")
        form = QtWidgets.QFormLayout(box)

        self.input_pump_label = QtWidgets.QLineEdit("actuales")
        self.label_chamber = QtWidgets.QLabel("ABC (mask=7) fijo")

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
        buttons.addWidget(self.btn_start)
        buttons.addWidget(self.btn_cancel)
        buttons.addWidget(self.btn_export)

        form.addRow("Pump label", self.input_pump_label)
        form.addRow("Cámara", self.label_chamber)
        form.addRow("Target presión", self.spin_target_pressure)
        form.addRow("Target vacío", self.spin_target_vacuum)
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

        return box

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
        box = QtWidgets.QGroupBox("Curva kPa (cruda y filtrada)")
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

        layout.addWidget(self.plot)
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
        self.history_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.history_table.horizontalHeader().setStretchLastSection(True)

        self.label_history = QtWidgets.QLabel("Registros: 0")

        layout.addWidget(self.history_table)
        layout.addWidget(self.label_history)
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
        for marker in self.markers:
            self.plot.removeItem(marker)
        self.markers = []

    def _add_marker(self, x_value: float, color: str) -> None:
        line = pg.InfiniteLine(
            pos=float(x_value),
            angle=90,
            pen=pg.mkPen(color, width=1, style=QtCore.Qt.DashLine),
            movable=False,
        )
        self.plot.addItem(line)
        self.markers.append(line)

    def _build_command(self) -> list[str]:
        python_bin = sys.executable
        command = [
            python_bin,
            CORE_SCRIPT,
            "--pump-label",
            self.input_pump_label.text().strip(),
            "--chamber",
            "7",
            "--target-pressure-kpa",
            f"{self.spin_target_pressure.value():.4f}",
            "--target-vacuum-kpa",
            f"{self.spin_target_vacuum.value():.4f}",
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

        if not os.path.exists(CORE_SCRIPT):
            QtWidgets.QMessageBox.critical(self, "Core missing", f"No existe: {CORE_SCRIPT}")
            return

        self._set_running(True)
        self.btn_export.setEnabled(False)
        self.label_run_status.setText("Estado: ejecutando...")
        self._clear_plot()
        self._update_idle_state()
        self.text_output.clear()
        self.process_buffer = ""
        self.last_summary_csv = None
        self.last_raw_csv = None

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
                self._add_marker(float(t), "#3A86FF")
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
                self._add_marker(float(t), "#2A9D8F")
            return

        if event == "phase_end":
            end_t = payload.get("end_session_s")
            if end_t is not None:
                self._add_marker(float(end_t), "#6C757D")
            peak_t = payload.get("time_to_top_session_s")
            if peak_t is not None:
                self._add_marker(float(peak_t), "#EF476F")
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
            self.label_run_status.setText(f"Estado: error ({message})")
            return

    def _on_process_finished(self, exit_code: int, _exit_status: Any) -> None:
        self._on_process_output()
        self._set_running(False)
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

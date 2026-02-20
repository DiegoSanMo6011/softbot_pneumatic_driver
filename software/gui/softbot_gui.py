#!/usr/bin/env python3
"""
GUI de escritorio para SoftBot (telemetría en tiempo real + control básico).
- Gráficas: presión, setpoint, PWM
- Controles: modo, cámara, setpoint, E-STOP
- Logging: CSV en experiments/logs/
"""

import csv
import os
import sys
import time
from collections import deque

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

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, Int8, Int16, Int16MultiArray

BASE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
SOFTWARE_DIR = os.path.join(BASE_DIR, "software")
if SOFTWARE_DIR not in sys.path:
    sys.path.append(SOFTWARE_DIR)

from sdk.protocol import (  # noqa: E402
    CHAMBER_A,
    CHAMBER_B,
    CHAMBER_BLOCKED,
    CHAMBER_C,
    HW_MUX_CHAMBER_A,
    HW_MUX_CHAMBER_B,
    HW_PUMP_INFLATE_AUX,
    HW_PUMP_INFLATE_MAIN,
    HW_PUMP_SUCTION_AUX,
    HW_PUMP_SUCTION_MAIN,
    HW_VALVE_CHAMBER_C,
    HW_VALVE_INFLATE,
    HW_VALVE_SUCTION,
    MODE_HARDWARE_DIAGNOSTIC,
    MODE_LABELS,
    MODE_PID_INFLATE,
    MODE_PID_SUCTION,
    MODE_PWM_INFLATE,
    MODE_PWM_SUCTION,
    MODE_STOP,
    MODE_VENT,
)

LOG_DIR = os.path.join(BASE_DIR, "experiments", "logs")
PUMP_BENCH_SCRIPT = os.path.join(BASE_DIR, "software", "tools", "pump_swap_validation.py")

MODE_OPTIONS = [
    ("0 - Stop", MODE_STOP),
    ("1 - PID Inflado", MODE_PID_INFLATE),
    ("-1 - PID Succión", MODE_PID_SUCTION),
    ("2 - PWM Inflado", MODE_PWM_INFLATE),
    ("-2 - PWM Succión", MODE_PWM_SUCTION),
    ("9 - Hardware Diag", MODE_HARDWARE_DIAGNOSTIC),
]


class SoftBotNode(Node):
    def __init__(self):
        super().__init__("softbot_gui")

        # --- Topicos ---
        self.topic_chamber = "/active_chamber"
        self.topic_mode = "/pressure_mode"
        self.topic_setpoint = "/pressure_setpoint"
        self.topic_tuning = "/tuning_params"
        self.topic_hwtest = "/hardware_test"
        self.topic_feedback = "/pressure_feedback"
        self.topic_debug = "/system_debug"

        # --- Publicadores ---
        self.pub_chamber = self.create_publisher(Int8, self.topic_chamber, 10)
        self.pub_mode = self.create_publisher(Int8, self.topic_mode, 10)
        self.pub_setpoint = self.create_publisher(Float32, self.topic_setpoint, 10)
        self.pub_tuning = self.create_publisher(
            Float32MultiArray,
            self.topic_tuning,
            10,
        )
        self.pub_hwtest = self.create_publisher(Int16, self.topic_hwtest, 10)

        # --- Suscriptores ---
        self.sub_feedback = self.create_subscription(
            Float32,
            self.topic_feedback,
            self._cb_feedback,
            10,
        )
        self.sub_debug = self.create_subscription(
            Int16MultiArray,
            self.topic_debug,
            self._cb_debug,
            10,
        )
        self.sub_setpoint = self.create_subscription(
            Float32,
            self.topic_setpoint,
            self._cb_setpoint,
            10,
        )
        self.sub_mode = self.create_subscription(
            Int8,
            self.topic_mode,
            self._cb_mode,
            10,
        )
        self.sub_chamber = self.create_subscription(
            Int8,
            self.topic_chamber,
            self._cb_chamber,
            10,
        )

        # --- Estado ---
        self.pressure_kpa = 0.0
        self.setpoint = 0.0
        self.mode = 0
        self.chamber = 0
        self.pwm_main = 0
        self.pwm_aux = 0
        self.error_raw = 0

    def _cb_feedback(self, msg: Float32):
        self.pressure_kpa = float(msg.data)

    def _cb_setpoint(self, msg: Float32):
        self.setpoint = float(msg.data)

    def _cb_mode(self, msg: Int8):
        self.mode = int(msg.data)

    def _cb_chamber(self, msg: Int8):
        self.chamber = int(msg.data)

    def _cb_debug(self, msg: Int16MultiArray):
        if len(msg.data) >= 4:
            self.pwm_main = int(msg.data[0])
            self.pwm_aux = int(msg.data[1])
            self.error_raw = int(msg.data[2])
            self.mode = int(msg.data[3])

    @staticmethod
    def _valid_chamber(chamber: int) -> bool:
        return chamber in range(0, 8)

    def send_command(self, chamber: int, mode: int, setpoint: float):
        if not self._valid_chamber(int(chamber)):
            raise ValueError(f"Camara invalida: {chamber}. Usa una mascara valida entre 0 y 7.")
        self.pub_chamber.publish(Int8(data=int(chamber)))
        self.pub_mode.publish(Int8(data=int(mode)))
        self.pub_setpoint.publish(Float32(data=float(setpoint)))

    def send_command_reliable(
        self,
        chamber: int,
        mode: int,
        setpoint: float,
        repeats: int = 3,
        interval_ms: int = 30,
    ):
        self.send_command(chamber, mode, setpoint)
        for i in range(1, repeats):
            QtCore.QTimer.singleShot(
                interval_ms * i,
                lambda c=chamber, m=mode, s=setpoint: self.send_command(c, m, s),
            )

    def stop(self):
        self.pub_mode.publish(Int8(data=MODE_STOP))
        self.pub_setpoint.publish(Float32(data=0.0))

    def e_stop(self):
        self.pub_chamber.publish(Int8(data=CHAMBER_BLOCKED))
        self.pub_mode.publish(Int8(data=MODE_STOP))
        self.pub_setpoint.publish(Float32(data=0.0))

    def update_tuning(self, kp_pos, ki_pos, kp_neg, ki_neg, max_safe, min_safe):
        msg = Float32MultiArray()
        msg.data = [
            float(kp_pos),
            float(ki_pos),
            float(kp_neg),
            float(ki_neg),
            float(max_safe),
            float(min_safe),
        ]
        self.pub_tuning.publish(msg)

    def vent(self, chamber_id: int, duration_ms: int):
        self.send_command_reliable(chamber_id, MODE_VENT, 0.0)
        if duration_ms > 0:
            QtCore.QTimer.singleShot(duration_ms, self.stop)

    def hardware_test(self, bitmask: int, pwm: int, repeats: int = 3, interval_ms: int = 30):
        mode_msg = Int8(data=MODE_HARDWARE_DIAGNOSTIC)
        setpoint_msg = Float32(data=float(max(0, min(255, int(pwm)))))
        mask_msg = Int16(data=int(bitmask))
        self.pub_mode.publish(mode_msg)
        self.pub_setpoint.publish(setpoint_msg)
        self.pub_hwtest.publish(mask_msg)
        for i in range(1, max(1, int(repeats))):
            QtCore.QTimer.singleShot(interval_ms * i, lambda m=mode_msg: self.pub_mode.publish(m))
            QtCore.QTimer.singleShot(
                interval_ms * i,
                lambda s=setpoint_msg: self.pub_setpoint.publish(s),
            )
            QtCore.QTimer.singleShot(
                interval_ms * i,
                lambda h=mask_msg: self.pub_hwtest.publish(h),
            )


class SoftBotGUI(QtWidgets.QMainWindow):
    def __init__(self, node: SoftBotNode):
        super().__init__()
        self.node = node

        self.setWindowTitle("SoftBot GUI — Telemetría en Tiempo Real")
        self.resize(1200, 720)

        self._init_state()
        self._build_ui()
        self._init_timers()

    def _init_state(self):
        self.start_time = time.time()
        self.max_points = 1500
        self.t = deque(maxlen=self.max_points)
        self.pressure = deque(maxlen=self.max_points)
        self.setpoint = deque(maxlen=self.max_points)
        self.pwm_main = deque(maxlen=self.max_points)
        self.pwm_aux = deque(maxlen=self.max_points)
        self.error = deque(maxlen=self.max_points)

        self.log_file = None
        self.log_writer = None
        self.bench_process = None

    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)

        main_layout = QtWidgets.QHBoxLayout(central)

        # --- Panel izquierdo: controles ---
        control_panel = QtWidgets.QVBoxLayout()

        chamber_box = QtWidgets.QGroupBox("Cámaras activas (/active_chamber bitmask)")
        chamber_layout = QtWidgets.QHBoxLayout(chamber_box)
        self.cb_chamber_a = QtWidgets.QCheckBox("A")
        self.cb_chamber_b = QtWidgets.QCheckBox("B")
        self.cb_chamber_c = QtWidgets.QCheckBox("C")
        self.cb_chamber_a.setChecked(True)
        self.cb_chamber_b.setChecked(True)
        self.cb_chamber_c.setChecked(True)
        chamber_layout.addWidget(self.cb_chamber_a)
        chamber_layout.addWidget(self.cb_chamber_b)
        chamber_layout.addWidget(self.cb_chamber_c)
        chamber_layout.addStretch(1)

        self.combo_mode = QtWidgets.QComboBox()
        self.combo_mode.addItems([label for label, _ in MODE_OPTIONS])
        self.combo_mode.currentIndexChanged.connect(self.on_mode_changed)

        self.spin_setpoint = QtWidgets.QDoubleSpinBox()
        self.spin_setpoint.setRange(-80.0, 80.0)
        self.spin_setpoint.setDecimals(2)
        self.spin_setpoint.setValue(0.0)

        btn_send = QtWidgets.QPushButton("Enviar comando")
        btn_stop = QtWidgets.QPushButton("Stop")
        btn_estop = QtWidgets.QPushButton("E-STOP")
        btn_reset = QtWidgets.QPushButton("Reset gráficas")
        self.btn_log = QtWidgets.QPushButton("Iniciar log")

        btn_send.clicked.connect(self.on_send)
        btn_stop.clicked.connect(self.on_stop)
        btn_estop.clicked.connect(self.on_estop)
        btn_reset.clicked.connect(self.on_reset)
        self.btn_log.clicked.connect(self.on_toggle_log)

        # --- Venteo ---
        vent_box = QtWidgets.QGroupBox("Venteo (liberar presión)")
        vent_layout = QtWidgets.QFormLayout(vent_box)
        self.spin_vent_ms = QtWidgets.QSpinBox()
        self.spin_vent_ms.setRange(0, 5000)
        self.spin_vent_ms.setValue(500)
        self.spin_vent_ms.setSuffix(" ms")
        btn_vent = QtWidgets.QPushButton("Ventear")
        btn_vent.clicked.connect(self.on_vent)
        vent_layout.addRow("Duración", self.spin_vent_ms)
        vent_layout.addRow(btn_vent)

        # --- Hardware diag ---
        hw_box = QtWidgets.QGroupBox("Hardware test (componentes)")
        hw_layout = QtWidgets.QGridLayout(hw_box)

        self.cb_hw_inflate_main = QtWidgets.QCheckBox("Pump Inflate Main")
        self.cb_hw_inflate_aux = QtWidgets.QCheckBox("Pump Inflate Aux")
        self.cb_hw_suction_main = QtWidgets.QCheckBox("Pump Suction Main")
        self.cb_hw_suction_aux = QtWidgets.QCheckBox("Pump Suction Aux")
        self.cb_hw_valve_inflate = QtWidgets.QCheckBox("Valve Inflate")
        self.cb_hw_valve_suction = QtWidgets.QCheckBox("Valve Suction")
        self.cb_hw_valve_chamber_c = QtWidgets.QCheckBox("Valve Chamber C (legacy BOOST pin)")
        self.cb_hw_mux_a = QtWidgets.QCheckBox("Mux A")
        self.cb_hw_mux_b = QtWidgets.QCheckBox("Mux B")

        self.spin_hw_pwm = QtWidgets.QSpinBox()
        self.spin_hw_pwm.setRange(0, 255)
        self.spin_hw_pwm.setValue(120)

        btn_hw_apply = QtWidgets.QPushButton("Aplicar HW Test")
        btn_hw_off = QtWidgets.QPushButton("HW OFF")
        btn_hw_apply.clicked.connect(self.on_hw_apply)
        btn_hw_off.clicked.connect(self.on_hw_off)

        hw_layout.addWidget(self.cb_hw_inflate_main, 0, 0)
        hw_layout.addWidget(self.cb_hw_inflate_aux, 0, 1)
        hw_layout.addWidget(self.cb_hw_suction_main, 1, 0)
        hw_layout.addWidget(self.cb_hw_suction_aux, 1, 1)
        hw_layout.addWidget(self.cb_hw_valve_inflate, 2, 0)
        hw_layout.addWidget(self.cb_hw_valve_suction, 2, 1)
        hw_layout.addWidget(self.cb_hw_valve_chamber_c, 3, 0)
        hw_layout.addWidget(self.cb_hw_mux_a, 3, 1)
        hw_layout.addWidget(self.cb_hw_mux_b, 4, 0)
        hw_layout.addWidget(QtWidgets.QLabel("PWM"), 4, 1)
        hw_layout.addWidget(self.spin_hw_pwm, 4, 2)
        hw_layout.addWidget(btn_hw_apply, 5, 0, 1, 2)
        hw_layout.addWidget(btn_hw_off, 5, 2, 1, 1)

        # --- Tuning ---
        tuning_box = QtWidgets.QGroupBox("Tuning rápido")
        tuning_layout = QtWidgets.QFormLayout(tuning_box)
        self.kp_pos = QtWidgets.QDoubleSpinBox()
        self.kp_pos.setRange(-2000, 2000)
        self.kp_pos.setValue(24.0)
        self.ki_pos = QtWidgets.QDoubleSpinBox()
        self.ki_pos.setRange(-5000, 5000)
        self.ki_pos.setValue(1500.0)
        self.kp_neg = QtWidgets.QDoubleSpinBox()
        self.kp_neg.setRange(-2000, 2000)
        self.kp_neg.setValue(-75.0)
        self.ki_neg = QtWidgets.QDoubleSpinBox()
        self.ki_neg.setRange(-5000, 5000)
        self.ki_neg.setValue(-750.0)
        self.max_safe = QtWidgets.QDoubleSpinBox()
        self.max_safe.setRange(0, 200)
        self.max_safe.setValue(55.0)
        self.min_safe = QtWidgets.QDoubleSpinBox()
        self.min_safe.setRange(-200, 0)
        self.min_safe.setValue(-60.0)

        tuning_widgets = [
            self.kp_pos,
            self.ki_pos,
            self.kp_neg,
            self.ki_neg,
            self.max_safe,
            self.min_safe,
        ]
        for widget in tuning_widgets:
            widget.setDecimals(2)

        tuning_layout.addRow("Kp+", self.kp_pos)
        tuning_layout.addRow("Ki+", self.ki_pos)
        tuning_layout.addRow("Kp-", self.kp_neg)
        tuning_layout.addRow("Ki-", self.ki_neg)
        tuning_layout.addRow("Max safe", self.max_safe)
        tuning_layout.addRow("Min safe", self.min_safe)

        btn_tuning = QtWidgets.QPushButton("Enviar tuning")
        btn_tuning.clicked.connect(self.on_tuning)
        tuning_layout.addRow(btn_tuning)

        # --- Benchmark bombas ---
        bench_box = QtWidgets.QGroupBox("Benchmark bombas (competencia)")
        bench_layout = QtWidgets.QGridLayout(bench_box)

        self.input_bench_label = QtWidgets.QLineEdit("actuales")
        self.combo_bench_mode = QtWidgets.QComboBox()
        self.combo_bench_mode.addItem("PID inflado (mode 1)", "pid")

        self.spin_bench_target = QtWidgets.QDoubleSpinBox()
        self.spin_bench_target.setRange(1.0, 80.0)
        self.spin_bench_target.setDecimals(2)
        self.spin_bench_target.setValue(35.0)

        self.spin_bench_runs = QtWidgets.QSpinBox()
        self.spin_bench_runs.setRange(1, 50)
        self.spin_bench_runs.setValue(5)

        self.spin_bench_timeout_s = QtWidgets.QDoubleSpinBox()
        self.spin_bench_timeout_s.setRange(0.5, 10.0)
        self.spin_bench_timeout_s.setDecimals(2)
        self.spin_bench_timeout_s.setValue(3.0)
        self.spin_bench_timeout_s.setSuffix(" s")

        self.spin_bench_sample_ms = QtWidgets.QSpinBox()
        self.spin_bench_sample_ms.setRange(5, 500)
        self.spin_bench_sample_ms.setValue(20)
        self.spin_bench_sample_ms.setSuffix(" ms")

        self.spin_bench_vent_s = QtWidgets.QDoubleSpinBox()
        self.spin_bench_vent_s.setRange(0.0, 5.0)
        self.spin_bench_vent_s.setDecimals(2)
        self.spin_bench_vent_s.setValue(0.6)
        self.spin_bench_vent_s.setSuffix(" s")

        self.spin_bench_rest_s = QtWidgets.QDoubleSpinBox()
        self.spin_bench_rest_s.setRange(0.0, 5.0)
        self.spin_bench_rest_s.setDecimals(2)
        self.spin_bench_rest_s.setValue(0.4)
        self.spin_bench_rest_s.setSuffix(" s")

        self.btn_bench_start = QtWidgets.QPushButton("Ejecutar benchmark")
        self.btn_bench_stop = QtWidgets.QPushButton("Cancelar benchmark")
        self.btn_bench_stop.setEnabled(False)
        self.btn_bench_start.clicked.connect(self.on_benchmark_start)
        self.btn_bench_stop.clicked.connect(self.on_benchmark_stop)

        self.text_bench_output = QtWidgets.QPlainTextEdit()
        self.text_bench_output.setReadOnly(True)
        self.text_bench_output.setMinimumHeight(120)
        self.text_bench_output.setPlaceholderText(
            "Salida del benchmark de bombas. Se guarda CSV en experiments/."
        )

        bench_layout.addWidget(QtWidgets.QLabel("Etiqueta bombas"), 0, 0)
        bench_layout.addWidget(self.input_bench_label, 0, 1)
        bench_layout.addWidget(QtWidgets.QLabel("Modo"), 1, 0)
        bench_layout.addWidget(self.combo_bench_mode, 1, 1)
        bench_layout.addWidget(QtWidgets.QLabel("Target"), 2, 0)
        bench_layout.addWidget(self.spin_bench_target, 2, 1)
        bench_layout.addWidget(QtWidgets.QLabel("Runs"), 3, 0)
        bench_layout.addWidget(self.spin_bench_runs, 3, 1)
        bench_layout.addWidget(QtWidgets.QLabel("Timeout"), 4, 0)
        bench_layout.addWidget(self.spin_bench_timeout_s, 4, 1)
        bench_layout.addWidget(QtWidgets.QLabel("Sample"), 5, 0)
        bench_layout.addWidget(self.spin_bench_sample_ms, 5, 1)
        bench_layout.addWidget(QtWidgets.QLabel("Vent"), 6, 0)
        bench_layout.addWidget(self.spin_bench_vent_s, 6, 1)
        bench_layout.addWidget(QtWidgets.QLabel("Rest"), 7, 0)
        bench_layout.addWidget(self.spin_bench_rest_s, 7, 1)
        bench_layout.addWidget(self.btn_bench_start, 8, 0)
        bench_layout.addWidget(self.btn_bench_stop, 8, 1)
        bench_layout.addWidget(self.text_bench_output, 9, 0, 1, 2)

        # --- Estado ---
        self.label_status = QtWidgets.QLabel("Estado: ---")
        self.label_status.setWordWrap(True)

        control_panel.addWidget(chamber_box)
        control_panel.addWidget(QtWidgets.QLabel("Modo"))
        control_panel.addWidget(self.combo_mode)
        control_panel.addWidget(QtWidgets.QLabel("Setpoint (kPa o PWM)"))
        control_panel.addWidget(self.spin_setpoint)
        control_panel.addWidget(btn_send)
        control_panel.addWidget(btn_stop)
        control_panel.addWidget(btn_estop)
        control_panel.addWidget(self.btn_log)
        control_panel.addWidget(btn_reset)
        control_panel.addWidget(tuning_box)
        control_panel.addWidget(vent_box)
        control_panel.addWidget(hw_box)
        control_panel.addWidget(bench_box)
        control_panel.addWidget(self.label_status)
        control_panel.addStretch(1)

        # --- Panel derecho: gráficas ---
        plot_panel = QtWidgets.QVBoxLayout()
        pg.setConfigOptions(antialias=True)

        self.plot_pressure = pg.PlotWidget(title="Presión / Setpoint")
        self.plot_pressure.setLabel("left", "kPa / PWM")
        self.plot_pressure.setLabel("bottom", "Tiempo", "s")
        self.curve_pressure = self.plot_pressure.plot(
            pen=pg.mkPen("#00A6FB", width=2),
            name="Presión",
        )
        self.curve_setpoint = self.plot_pressure.plot(
            pen=pg.mkPen("#FFB703", width=2, style=QtCore.Qt.DashLine),
            name="Setpoint",
        )

        self.plot_pwm = pg.PlotWidget(title="PWM (Main/Aux)")
        self.plot_pwm.setLabel("left", "PWM")
        self.plot_pwm.setLabel("bottom", "Tiempo", "s")
        self.curve_pwm_main = self.plot_pwm.plot(
            pen=pg.mkPen("#8AC926", width=2),
            name="PWM Main",
        )
        self.curve_pwm_aux = self.plot_pwm.plot(
            pen=pg.mkPen("#FF595E", width=2),
            name="PWM Aux",
        )

        plot_panel.addWidget(self.plot_pressure)
        plot_panel.addWidget(self.plot_pwm)

        main_layout.addLayout(control_panel, 1)
        main_layout.addLayout(plot_panel, 3)

    def _init_timers(self):
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(20)  # ~50 Hz

    def update_ui(self):
        rclpy.spin_once(self.node, timeout_sec=0)

        now = time.time() - self.start_time
        self.t.append(now)
        self.pressure.append(self.node.pressure_kpa)
        self.setpoint.append(self.node.setpoint)
        self.pwm_main.append(self.node.pwm_main)
        self.pwm_aux.append(self.node.pwm_aux)
        self.error.append(self.node.error_raw / 10.0)

        self.curve_pressure.setData(self.t, self.pressure)
        self.curve_setpoint.setData(self.t, self.setpoint)
        self.curve_pwm_main.setData(self.t, self.pwm_main)
        self.curve_pwm_aux.setData(self.t, self.pwm_aux)

        mode_txt = MODE_LABELS.get(self.node.mode, str(self.node.mode))
        chamber_mask = int(self.node.chamber) & 0x07
        chamber_txt = self._chamber_mask_label(chamber_mask)

        self.label_status.setText(
            f"P={self.node.pressure_kpa:.2f} kPa | "
            f"SP={self.node.setpoint:.2f} | "
            f"PWM=({self.node.pwm_main},{self.node.pwm_aux}) | "
            f"Mode={self.node.mode} ({mode_txt}) | "
            f"Err={self.node.error_raw / 10.0:.2f} | "
            f"ChMask={chamber_mask} ({chamber_txt})"
        )

        if self.log_writer:
            self.log_writer.writerow(
                [
                    f"{now:.4f}",
                    f"{self.node.setpoint:.4f}",
                    f"{self.node.pressure_kpa:.4f}",
                    self.node.pwm_main,
                    self.node.pwm_aux,
                    self.node.mode,
                    f"{self.node.error_raw / 10.0:.4f}",
                ]
            )

    @staticmethod
    def _chamber_mask_label(mask: int) -> str:
        if mask <= 0:
            return "BLOCKED"
        labels = []
        if mask & CHAMBER_A:
            labels.append("A")
        if mask & CHAMBER_B:
            labels.append("B")
        if mask & CHAMBER_C:
            labels.append("C")
        return "+".join(labels) if labels else "BLOCKED"

    def _selected_chamber_mask(self) -> int:
        mask = 0
        if self.cb_chamber_a.isChecked():
            mask |= CHAMBER_A
        if self.cb_chamber_b.isChecked():
            mask |= CHAMBER_B
        if self.cb_chamber_c.isChecked():
            mask |= CHAMBER_C
        return mask

    def on_send(self):
        chamber = self._selected_chamber_mask()
        mode = MODE_OPTIONS[self.combo_mode.currentIndex()][1]
        setpoint = self.spin_setpoint.value()
        self.node.send_command_reliable(chamber, mode, setpoint)

    def on_mode_changed(self, index: int):
        mode = MODE_OPTIONS[index][1]
        if mode in (MODE_PWM_INFLATE, MODE_PWM_SUCTION, MODE_HARDWARE_DIAGNOSTIC):
            # PWM
            self.spin_setpoint.setRange(0.0, 255.0)
            self.spin_setpoint.setDecimals(0)
        else:
            # PID (kPa)
            self.spin_setpoint.setRange(-80.0, 80.0)
            self.spin_setpoint.setDecimals(2)

    def on_stop(self):
        self.node.stop()

    def on_estop(self):
        self.node.e_stop()

    def on_reset(self):
        self.start_time = time.time()
        self.t.clear()
        self.pressure.clear()
        self.setpoint.clear()
        self.pwm_main.clear()
        self.pwm_aux.clear()
        self.error.clear()

    def on_vent(self):
        chamber = self._selected_chamber_mask()
        duration_ms = int(self.spin_vent_ms.value())
        self.node.vent(chamber, duration_ms)

    def _hardware_mask_from_ui(self) -> int:
        mask = 0
        if self.cb_hw_inflate_main.isChecked():
            mask |= HW_PUMP_INFLATE_MAIN
        if self.cb_hw_inflate_aux.isChecked():
            mask |= HW_PUMP_INFLATE_AUX
        if self.cb_hw_suction_main.isChecked():
            mask |= HW_PUMP_SUCTION_MAIN
        if self.cb_hw_suction_aux.isChecked():
            mask |= HW_PUMP_SUCTION_AUX
        if self.cb_hw_valve_inflate.isChecked():
            mask |= HW_VALVE_INFLATE
        if self.cb_hw_valve_suction.isChecked():
            mask |= HW_VALVE_SUCTION
        if self.cb_hw_valve_chamber_c.isChecked():
            mask |= HW_VALVE_CHAMBER_C
        if self.cb_hw_mux_a.isChecked():
            mask |= HW_MUX_CHAMBER_A
        if self.cb_hw_mux_b.isChecked():
            mask |= HW_MUX_CHAMBER_B
        return mask

    def on_hw_apply(self):
        mask = self._hardware_mask_from_ui()
        pwm = int(self.spin_hw_pwm.value())
        self.node.hardware_test(mask, pwm)

    def on_hw_off(self):
        self.node.hardware_test(0, 0, repeats=2, interval_ms=20)
        self.node.stop()

    def on_tuning(self):
        self.node.update_tuning(
            kp_pos=self.kp_pos.value(),
            ki_pos=self.ki_pos.value(),
            kp_neg=self.kp_neg.value(),
            ki_neg=self.ki_neg.value(),
            max_safe=self.max_safe.value(),
            min_safe=self.min_safe.value(),
        )

    def _set_benchmark_running(self, running: bool):
        self.btn_bench_start.setEnabled(not running)
        self.btn_bench_stop.setEnabled(running)

    def _append_benchmark_output(self, text: str):
        cleaned = text.replace("\r", "\n")
        for line in cleaned.splitlines():
            self.text_bench_output.appendPlainText(line)
        self.text_bench_output.ensureCursorVisible()

    def on_benchmark_start(self):
        if self.bench_process and self.bench_process.state() != QtCore.QProcess.NotRunning:
            QtWidgets.QMessageBox.information(
                self,
                "Benchmark en curso",
                "Ya hay un benchmark ejecutandose. Espera a que termine o cancelalo.",
            )
            return

        chamber = self._selected_chamber_mask()
        if chamber == CHAMBER_BLOCKED:
            QtWidgets.QMessageBox.warning(
                self,
                "Camara invalida",
                "Selecciona al menos una camara (A, B o C) para correr benchmark de bombas.",
            )
            return

        pump_label = self.input_bench_label.text().strip()
        if not pump_label:
            QtWidgets.QMessageBox.warning(
                self,
                "Etiqueta requerida",
                "Define una etiqueta para identificar la configuracion de bombas.",
            )
            return

        if not os.path.exists(PUMP_BENCH_SCRIPT):
            QtWidgets.QMessageBox.critical(
                self,
                "Script no encontrado",
                f"No existe: {PUMP_BENCH_SCRIPT}",
            )
            return

        mode = self.combo_bench_mode.currentData() or "pid"
        command = [
            sys.executable,
            PUMP_BENCH_SCRIPT,
            "--pump-label",
            pump_label,
            "--mode",
            str(mode),
            "--target-kpa",
            f"{self.spin_bench_target.value():.2f}",
            "--chamber",
            str(chamber),
            "--runs",
            str(self.spin_bench_runs.value()),
            "--timeout-s",
            f"{self.spin_bench_timeout_s.value():.2f}",
            "--sample-ms",
            str(self.spin_bench_sample_ms.value()),
            "--vent-s",
            f"{self.spin_bench_vent_s.value():.2f}",
            "--rest-s",
            f"{self.spin_bench_rest_s.value():.2f}",
        ]

        self.text_bench_output.clear()
        self._append_benchmark_output("=== Benchmark bombas ===")
        self._append_benchmark_output(" ".join(command))

        self.bench_process = QtCore.QProcess(self)
        self.bench_process.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        self.bench_process.readyReadStandardOutput.connect(self._on_benchmark_stdout)
        self.bench_process.finished.connect(self._on_benchmark_finished)
        self.bench_process.errorOccurred.connect(self._on_benchmark_error)
        self._set_benchmark_running(True)
        self.bench_process.start(command[0], command[1:])

        if not self.bench_process.waitForStarted(1500):
            self._set_benchmark_running(False)
            QtWidgets.QMessageBox.critical(
                self,
                "No se pudo iniciar benchmark",
                self.bench_process.errorString(),
            )
            self.bench_process = None

    def _on_benchmark_stdout(self):
        if not self.bench_process:
            return
        chunk = bytes(self.bench_process.readAllStandardOutput()).decode("utf-8", errors="replace")
        if chunk:
            self._append_benchmark_output(chunk)

    def _on_benchmark_finished(self, exit_code: int, _exit_status):
        self._on_benchmark_stdout()
        self._set_benchmark_running(False)
        status = "OK" if exit_code == 0 else f"ERROR ({exit_code})"
        self._append_benchmark_output(f"=== Benchmark finalizado: {status} ===")
        self.bench_process = None

    def _on_benchmark_error(self, _process_error):
        if not self.bench_process:
            return
        self._append_benchmark_output(f"[qprocess] {self.bench_process.errorString()}")

    def on_benchmark_stop(self):
        if not self.bench_process:
            return
        if self.bench_process.state() == QtCore.QProcess.NotRunning:
            return
        self._append_benchmark_output("Cancelando benchmark...")
        self.bench_process.terminate()
        if not self.bench_process.waitForFinished(1500):
            self.bench_process.kill()
            self.bench_process.waitForFinished(1000)

    def on_toggle_log(self):
        if self.log_writer is None:
            os.makedirs(LOG_DIR, exist_ok=True)
            filename = time.strftime("gui_log_%Y%m%d_%H%M%S.csv")
            filepath = os.path.join(LOG_DIR, filename)
            self.log_file = open(filepath, mode="w", newline="")
            self.log_writer = csv.writer(self.log_file)
            self.log_writer.writerow(
                [
                    "Timestamp_s",
                    "Setpoint",
                    "Feedback_kPa",
                    "PWM_Main",
                    "PWM_Aux",
                    "Mode",
                    "Error_kPa",
                ]
            )
            self.btn_log.setText("Detener log")
        else:
            self.log_file.close()
            self.log_file = None
            self.log_writer = None
            self.btn_log.setText("Iniciar log")

    def closeEvent(self, event):
        try:
            if self.log_file:
                self.log_file.close()
        except Exception:
            pass
        try:
            self.on_benchmark_stop()
        except Exception:
            pass
        self.node.destroy_node()
        rclpy.shutdown()
        event.accept()


def main():
    rclpy.init()
    node = SoftBotNode()

    app = QtWidgets.QApplication(sys.argv)
    gui = SoftBotGUI(node)
    gui.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()

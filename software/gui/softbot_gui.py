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
from std_msgs.msg import Float32, Float32MultiArray, Int8, Int16MultiArray

BASE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
SOFTWARE_DIR = os.path.join(BASE_DIR, "software")
if SOFTWARE_DIR not in sys.path:
    sys.path.append(SOFTWARE_DIR)

from sdk.protocol import (  # noqa: E402
    CHAMBER_BLOCKED,
    MODE_LABELS,
    MODE_PID_INFLATE,
    MODE_PID_INFLATE_TURBO,
    MODE_PID_SUCTION,
    MODE_PWM_INFLATE,
    MODE_PWM_SUCTION,
    MODE_STOP,
    MODE_TANK_FILL,
    MODE_VENT,
)

LOG_DIR = os.path.join(BASE_DIR, "experiments", "logs")

MODE_OPTIONS = [
    ("0 - Stop", MODE_STOP),
    ("1 - PID Inflado", MODE_PID_INFLATE),
    ("-1 - PID Succión", MODE_PID_SUCTION),
    ("2 - PWM Inflado", MODE_PWM_INFLATE),
    ("-2 - PWM Succión", MODE_PWM_SUCTION),
]


class SoftBotNode(Node):
    def __init__(self):
        super().__init__("softbot_gui")

        # --- Topicos ---
        self.topic_chamber = "/active_chamber"
        self.topic_mode = "/pressure_mode"
        self.topic_setpoint = "/pressure_setpoint"
        self.topic_tuning = "/tuning_params"
        self.topic_boost = "/boost_valve"
        self.topic_tank_state = "/tank_state"
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
        self.pub_boost = self.create_publisher(Int8, self.topic_boost, 10)

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
        self.sub_tank_state = self.create_subscription(
            Int8,
            self.topic_tank_state,
            self._cb_tank_state,
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
        self.tank_state = 0

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

    def _cb_tank_state(self, msg: Int8):
        self.tank_state = int(msg.data)

    @staticmethod
    def _valid_chamber(chamber: int) -> bool:
        return chamber in (0, 1, 2, 3)

    def send_command(self, chamber: int, mode: int, setpoint: float):
        if not self._valid_chamber(int(chamber)):
            raise ValueError(f"Camara invalida: {chamber}. Usa 0, 1, 2 o 3.")
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

    def set_boost(self, enabled: bool, repeats: int = 3, interval_ms: int = 30):
        msg = Int8(data=1 if enabled else 0)
        self.pub_boost.publish(msg)
        for i in range(1, max(1, int(repeats))):
            QtCore.QTimer.singleShot(
                interval_ms * i,
                lambda m=msg: self.pub_boost.publish(m),
            )

    def fill_tank(self, target_kpa: float):
        self.send_command_reliable(CHAMBER_BLOCKED, MODE_TANK_FILL, float(target_kpa))

    def inflate_turbo(self, chamber_id: int, target_kpa: float):
        self.send_command_reliable(
            chamber_id,
            MODE_PID_INFLATE_TURBO,
            float(target_kpa),
        )

    def vent(self, chamber_id: int, duration_ms: int):
        self.send_command_reliable(chamber_id, MODE_VENT, 0.0)
        if duration_ms > 0:
            QtCore.QTimer.singleShot(duration_ms, self.stop)


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
        self.boost_enabled = False

    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)

        main_layout = QtWidgets.QHBoxLayout(central)

        # --- Panel izquierdo: controles ---
        control_panel = QtWidgets.QVBoxLayout()

        self.combo_chamber = QtWidgets.QComboBox()
        self.combo_chamber.addItems(["0 - Bloqueo", "1 - Cámara A", "2 - Cámara B", "3 - A+B"])

        self.combo_mode = QtWidgets.QComboBox()
        self.combo_mode.addItems([label for label, _ in MODE_OPTIONS])
        self.combo_mode.currentIndexChanged.connect(self.on_mode_changed)

        self.spin_setpoint = QtWidgets.QDoubleSpinBox()
        self.spin_setpoint.setRange(-80.0, 80.0)
        self.spin_setpoint.setDecimals(2)
        self.spin_setpoint.setValue(0.0)

        btn_send = QtWidgets.QPushButton("Enviar comando")
        btn_turbo_pid = QtWidgets.QPushButton("Inflar Turbo+PID")
        btn_turbo_pid.setToolTip("Flujo recomendado: pulso turbo automatico y entrada a PID.")
        btn_stop = QtWidgets.QPushButton("Stop")
        btn_estop = QtWidgets.QPushButton("E-STOP")
        btn_reset = QtWidgets.QPushButton("Reset gráficas")
        self.btn_log = QtWidgets.QPushButton("Iniciar log")

        btn_send.clicked.connect(self.on_send)
        btn_turbo_pid.clicked.connect(self.on_turbo_pid)
        btn_stop.clicked.connect(self.on_stop)
        btn_estop.clicked.connect(self.on_estop)
        btn_reset.clicked.connect(self.on_reset)
        self.btn_log.clicked.connect(self.on_toggle_log)

        # --- Tanque (Fill) ---
        tank_box = QtWidgets.QGroupBox("Tanque (Fill)")
        tank_layout = QtWidgets.QFormLayout(tank_box)
        self.spin_tank_kpa = QtWidgets.QDoubleSpinBox()
        self.spin_tank_kpa.setRange(0.0, 80.0)
        self.spin_tank_kpa.setDecimals(2)
        self.spin_tank_kpa.setValue(35.0)
        btn_tank_fill = QtWidgets.QPushButton("Llenar tanque")
        btn_tank_stop = QtWidgets.QPushButton("Detener llenado")
        btn_tank_fill.clicked.connect(self.on_tank_fill)
        btn_tank_stop.clicked.connect(self.on_tank_stop)
        tank_layout.addRow("Objetivo kPa", self.spin_tank_kpa)
        tank_layout.addRow(btn_tank_fill)
        tank_layout.addRow(btn_tank_stop)

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

        # --- Boost ---
        boost_box = QtWidgets.QGroupBox("BOOST manual (diagnóstico)")
        boost_layout = QtWidgets.QFormLayout(boost_box)
        self.btn_boost = QtWidgets.QPushButton("BOOST: OFF")
        self.btn_boost.setCheckable(True)
        self.spin_boost_ms = QtWidgets.QSpinBox()
        self.spin_boost_ms.setRange(10, 2000)
        self.spin_boost_ms.setValue(150)
        self.spin_boost_ms.setSuffix(" ms")
        btn_boost_pulse = QtWidgets.QPushButton("Pulso BOOST")
        self.btn_boost.clicked.connect(self.on_boost_toggle)
        btn_boost_pulse.clicked.connect(self.on_boost_pulse)
        boost_layout.addRow(self.btn_boost)
        boost_layout.addRow("Duración", self.spin_boost_ms)
        boost_layout.addRow(btn_boost_pulse)

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

        # --- Estado ---
        self.label_status = QtWidgets.QLabel("Estado: ---")
        self.label_status.setWordWrap(True)

        control_panel.addWidget(QtWidgets.QLabel("Cámara activa"))
        control_panel.addWidget(self.combo_chamber)
        control_panel.addWidget(QtWidgets.QLabel("Modo"))
        control_panel.addWidget(self.combo_mode)
        control_panel.addWidget(QtWidgets.QLabel("Setpoint (kPa o PWM)"))
        control_panel.addWidget(self.spin_setpoint)
        control_panel.addWidget(btn_send)
        control_panel.addWidget(btn_turbo_pid)
        control_panel.addWidget(btn_stop)
        control_panel.addWidget(btn_estop)
        control_panel.addWidget(self.btn_log)
        control_panel.addWidget(btn_reset)
        control_panel.addWidget(tuning_box)
        control_panel.addWidget(tank_box)
        control_panel.addWidget(vent_box)
        control_panel.addWidget(boost_box)
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

        tank_map = {0: "IDLE", 1: "LLENANDO", 2: "LLENO", 3: "TIMEOUT"}
        tank_txt = tank_map.get(self.node.tank_state, "N/A")
        mode_txt = MODE_LABELS.get(self.node.mode, str(self.node.mode))
        boost_txt = (
            "ON(auto)"
            if self.node.mode == MODE_PID_INFLATE_TURBO
            else ("ON" if self.boost_enabled else "OFF")
        )

        self.label_status.setText(
            f"P={self.node.pressure_kpa:.2f} kPa | "
            f"SP={self.node.setpoint:.2f} | "
            f"PWM=({self.node.pwm_main},{self.node.pwm_aux}) | "
            f"Mode={self.node.mode} ({mode_txt}) | "
            f"Err={self.node.error_raw / 10.0:.2f} | "
            f"Boost={boost_txt} | "
            f"Tanque={tank_txt}"
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

    def on_send(self):
        chamber = self.combo_chamber.currentIndex()
        mode = MODE_OPTIONS[self.combo_mode.currentIndex()][1]
        setpoint = self.spin_setpoint.value()
        self.node.send_command_reliable(chamber, mode, setpoint)

    def on_turbo_pid(self):
        chamber = self.combo_chamber.currentIndex()
        setpoint = float(self.spin_setpoint.value())
        if chamber == CHAMBER_BLOCKED:
            QtWidgets.QMessageBox.warning(
                self,
                "Turbo+PID inválido",
                "Selecciona una camara (A, B o A+B) para usar Turbo+PID.",
            )
            return
        if setpoint <= 0.0:
            QtWidgets.QMessageBox.warning(
                self,
                "Turbo+PID inválido",
                "El setpoint debe ser mayor a 0 kPa para inflado turbo.",
            )
            return
        if setpoint > 80.0:
            QtWidgets.QMessageBox.warning(
                self,
                "Turbo+PID inválido",
                "Turbo+PID usa setpoint en kPa. Usa un valor en el rango 0-80 kPa.",
            )
            return
        self.node.inflate_turbo(chamber, setpoint)

    def on_mode_changed(self, index: int):
        mode = MODE_OPTIONS[index][1]
        if mode in (MODE_PWM_INFLATE, MODE_PWM_SUCTION):
            # PWM
            self.spin_setpoint.setRange(0.0, 255.0)
            self.spin_setpoint.setDecimals(0)
        else:
            # PID (kPa)
            self.spin_setpoint.setRange(-80.0, 80.0)
            self.spin_setpoint.setDecimals(2)

    def on_stop(self):
        self.node.stop()
        self._boost_pulse_off()

    def on_estop(self):
        self.node.e_stop()
        self._boost_pulse_off()

    def on_reset(self):
        self.start_time = time.time()
        self.t.clear()
        self.pressure.clear()
        self.setpoint.clear()
        self.pwm_main.clear()
        self.pwm_aux.clear()
        self.error.clear()

    def on_tank_fill(self):
        target = float(self.spin_tank_kpa.value())
        self.node.set_boost(False)
        self.boost_enabled = False
        self.btn_boost.setChecked(False)
        self.btn_boost.setText("BOOST: OFF")
        self.node.fill_tank(target)

    def on_tank_stop(self):
        self.node.stop()

    def on_vent(self):
        chamber = self.combo_chamber.currentIndex()
        duration_ms = int(self.spin_vent_ms.value())
        self.node.vent(chamber, duration_ms)

    def on_boost_toggle(self):
        self.boost_enabled = bool(self.btn_boost.isChecked())
        self.node.set_boost(self.boost_enabled)
        self.btn_boost.setText("BOOST: ON" if self.boost_enabled else "BOOST: OFF")

    def on_boost_pulse(self):
        duration_ms = int(self.spin_boost_ms.value())
        self.node.set_boost(True)
        self.boost_enabled = True
        self.btn_boost.setChecked(True)
        self.btn_boost.setText("BOOST: ON")
        QtCore.QTimer.singleShot(duration_ms, self._boost_pulse_off)

    def _boost_pulse_off(self):
        self.node.set_boost(False)
        self.boost_enabled = False
        self.btn_boost.setChecked(False)
        self.btn_boost.setText("BOOST: OFF")

    def on_tuning(self):
        self.node.update_tuning(
            kp_pos=self.kp_pos.value(),
            ki_pos=self.ki_pos.value(),
            kp_neg=self.kp_neg.value(),
            ki_neg=self.ki_neg.value(),
            max_safe=self.max_safe.value(),
            min_safe=self.min_safe.value(),
        )

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
            self._boost_pulse_off()
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

import sys
import csv
import os
from datetime import datetime
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QGroupBox, QPushButton, QComboBox, QLineEdit
)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont
from pymavlink import mavutil


FRIENDLY_NAMES = {
    "SYS_STATUS.voltage_battery":       "Batt. Voltage",
    "SYS_STATUS.current_battery":       "Batt. Current",
    "DISTANCE_SENSOR.current_distance": "Lidar Distance",
    "VFR_HUD.alt":                      "Relative Altitude",
    "GPS_RAW_INT.satellites_visible":   "GPS Satellites",
}

THRESHOLDS = {
    "SYS_STATUS.voltage_battery":       {"orange": 13200, "green": 14800},
    "SYS_STATUS.current_battery":       {"orange": 3000,  "green": 5000, "invert": True},
    "GPS_RAW_INT.satellites_visible":   {"orange": 5,     "green": 8},
    "DISTANCE_SENSOR.current_distance": {"orange": 50,    "green": 200},
}

LOG_FIELDS = [
    ("timestamp",         None),
    ("voltage_mV",        "SYS_STATUS.voltage_battery"),
    ("current_cA",        "SYS_STATUS.current_battery"),
    ("altitude_m",        "VFR_HUD.alt"),
    ("groundspeed_ms",    "VFR_HUD.groundspeed"),
    ("gps_satellites",    "GPS_RAW_INT.satellites_visible"),
    ("lidar_cm",          "DISTANCE_SENSOR.current_distance"),
    ("roll_rad",          "ATTITUDE.roll"),
    ("pitch_rad",         "ATTITUDE.pitch"),
    ("yaw_rad",           "ATTITUDE.yaw"),
    ("flight_mode",       "HEARTBEAT.custom_mode"),
]

DEFAULT_DYNAMIC_KEYS = [
    "VFR_HUD.alt",
    "DISTANCE_SENSOR.current_distance",
    "GPS_RAW_INT.satellites_visible",
    "ATTITUDE.yaw",
]

STYLE = """
    QMainWindow, QWidget { background-color: #0a0a0a; color: #ecf0f1; font-family: Consolas, Arial; }
    QGroupBox { border: 2px solid #2c3e50; border-radius: 6px; margin-top: 10px; font-weight: bold; padding: 6px; }
    QGroupBox::title { padding: 0 4px; }
    QComboBox { background-color: #2c3e50; color: white; padding: 6px; font-size: 12px; border: none; border-radius: 3px; }
    QComboBox::drop-down { border: none; }
    QPushButton { padding: 10px 20px; font-weight: bold; font-size: 13px; border-radius: 5px; border: none; }
    QLineEdit { background-color: #2c3e50; color: white; padding: 8px 10px; font-size: 13px; border: 1px solid #3d5166; border-radius: 4px; }
"""


def status_color(raw_key, raw_value):
    t = THRESHOLDS.get(raw_key)
    if t is None or raw_value is None:
        return "#3498db"
    invert = t.get("invert", False)
    if not invert:
        if raw_value >= t["green"]:  return "#2ecc71"
        if raw_value >= t["orange"]: return "#e67e22"
        return "#e74c3c"
    else:
        if raw_value <= t["green"]:  return "#2ecc71"
        if raw_value <= t["orange"]: return "#e67e22"
        return "#e74c3c"


def format_value(raw_key, value):
    if value is None:
        return "--"
    if "voltage"  in raw_key: return f"{value / 1000.0:.2f} V"
    if "current"  in raw_key: return f"{value / 100.0:.1f} A"
    if "distance" in raw_key: return f"{value / 100.0:.2f} m"
    if "yaw" in raw_key or "pitch" in raw_key or "roll" in raw_key:
        import math
        return f"{math.degrees(value):.1f}°"
    if isinstance(value, float): return f"{value:.2f}"
    return str(value)


class FixedCard(QGroupBox):
    def __init__(self, title, default_text, color):
        super().__init__(title)
        layout = QVBoxLayout()
        self.label = QLabel(default_text)
        self.label.setFont(QFont("Arial", 22, QFont.Bold))
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setStyleSheet(f"color: {color}; padding: 10px;")
        layout.addWidget(self.label)
        self.setLayout(layout)

    def set_text(self, text, color=None):
        self.label.setText(text)
        if color:
            self.label.setStyleSheet(f"color: {color}; padding: 10px;")


class DynamicCard(QGroupBox):
    def __init__(self):
        super().__init__("—")
        layout = QVBoxLayout()
        self.combo = QComboBox()
        self.combo.currentTextChanged.connect(self._on_combo_changed)
        layout.addWidget(self.combo)
        self.label = QLabel("--")
        self.label.setFont(QFont("Arial", 26, QFont.Bold))
        self.label.setStyleSheet("color: #3498db; margin: 16px;")
        self.label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.label)
        self.setLayout(layout)
        self.display_to_raw = {}

    def _on_combo_changed(self, text):
        raw_key = self.display_to_raw.get(text)
        self.setTitle(FRIENDLY_NAMES.get(raw_key, raw_key) if raw_key else "—")

    def update_combo(self, display_to_raw):
        self.display_to_raw = display_to_raw
        current = self.combo.currentText()
        self.combo.blockSignals(True)
        self.combo.clear()
        self.combo.addItems(list(display_to_raw.keys()))
        if current in display_to_raw:
            self.combo.setCurrentText(current)
        self.combo.blockSignals(False)
        self._on_combo_changed(self.combo.currentText())

    def set_default(self, raw_key, display_to_raw):
        display = next((d for d, r in display_to_raw.items() if r == raw_key), None)
        if display:
            self.combo.setCurrentText(display)

    def current_raw_key(self):
        return self.display_to_raw.get(self.combo.currentText())

    def set_value(self, text, color):
        self.label.setText(text)
        self.label.setStyleSheet(f"color: {color}; margin: 16px;")


class Dashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PX4 Telemetry Dashboard")
        self.resize(1200, 520)
        self.setStyleSheet(STYLE)

        self.master = None
        self.is_connected = False
        self.is_connecting = False
        self.raw_telemetry = {}
        self.display_to_raw = {}
        self.last_combo_keys = []
        self.defaults_applied = False

        self.is_armed = False
        self.log_file = None
        self.log_writer = None

        self.data_timer = QTimer()
        self.data_timer.timeout.connect(self.update_data)

        self.heartbeat_timer = QTimer()
        self.heartbeat_timer.timeout.connect(self.try_heartbeat)

        self.log_timer = QTimer()
        self.log_timer.timeout.connect(self.write_log_row)

        self.setup_ui()

    def setup_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)
        layout.setSpacing(10)

        # --- top bar ---
        top = QHBoxLayout()
        top.setSpacing(10)

        lbl_port = QLabel("Listen port:")
        lbl_port.setStyleSheet("color: #95a5a6; font-size: 13px;")
        self.input_port = QLineEdit("14551")
        self.input_port.setFixedWidth(90)

        self.btn_connect = QPushButton("Connect")
        self.btn_connect.setFixedWidth(130)
        self.btn_connect.setStyleSheet("background-color: #27ae60; color: white;")
        self.btn_connect.clicked.connect(self.toggle_connection)

        self.lbl_status = QLabel("Waiting...")
        self.lbl_status.setStyleSheet("color: #7f8c8d; font-size: 13px;")

        top.addWidget(lbl_port)
        top.addWidget(self.input_port)
        top.addWidget(self.btn_connect)
        top.addSpacing(16)
        top.addWidget(self.lbl_status)
        top.addStretch()
        layout.addLayout(top)

        # --- fixed cards row ---
        fixed_row = QHBoxLayout()
        self.card_voltage = FixedCard("Voltage",     "--",      "#f1c40f")
        self.card_current = FixedCard("Current",     "--",      "#f39c12")
        self.card_arm     = FixedCard("Arm Status",  "UNKNOWN", "#95a5a6")
        self.card_mode    = FixedCard("Flight Mode", "--",      "#3498db")
        for c in [self.card_voltage, self.card_current, self.card_arm, self.card_mode]:
            fixed_row.addWidget(c)
        layout.addLayout(fixed_row)

        # --- dynamic cards row ---
        dynamic_row = QHBoxLayout()
        self.dynamic_cards = [DynamicCard() for _ in range(4)]
        for card in self.dynamic_cards:
            dynamic_row.addWidget(card)
        layout.addLayout(dynamic_row)

    def toggle_connection(self):
        if self.is_connected or self.is_connecting:
            self.disconnect()
            return
        try:
            port = self.input_port.text().strip()
            self.master = mavutil.mavlink_connection(f"udpin:0.0.0.0:{port}")
            self.is_connecting = True
            self.defaults_applied = False
            self.input_port.setEnabled(False)
            self.btn_connect.setText("Connecting...")
            self.btn_connect.setStyleSheet("background-color: #e67e22; color: white;")
            self.lbl_status.setText(f"Listening on UDP port {port}...")
            self.heartbeat_timer.start(200)
        except Exception as e:
            self.lbl_status.setText(f"Error: {e}")

    def start_arm_log(self):
        os.makedirs("logs", exist_ok=True)
        filename = datetime.now().strftime("logs/armed_%Y-%m-%d_%H-%M-%S.csv")
        self.log_file = open(filename, "w", newline="")
        self.log_writer = csv.DictWriter(self.log_file, fieldnames=[f for f, _ in LOG_FIELDS])
        self.log_writer.writeheader()
        self.log_timer.start(1000)

    def stop_arm_log(self):
        self.log_timer.stop()
        if self.log_file:
            self.log_file.close()
            self.log_file = None
            self.log_writer = None

    def write_log_row(self):
        if not self.log_writer:
            return
        row = {"timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]}
        for col, raw_key in LOG_FIELDS[1:]:
            row[col] = self.raw_telemetry.get(raw_key, "")
        self.log_writer.writerow(row)
        self.log_file.flush()

    def disconnect(self):
        self.is_connected = False
        self.is_connecting = False
        self.heartbeat_timer.stop()
        self.data_timer.stop()
        self.stop_arm_log()
        self.raw_telemetry.clear()
        self.last_combo_keys = []
        self.input_port.setEnabled(True)
        self.btn_connect.setText("Connect")
        self.btn_connect.setStyleSheet("background-color: #27ae60; color: white;")
        self.lbl_status.setText("Disconnected.")

    def try_heartbeat(self):
        try:
            msg = self.master.recv_match(type="HEARTBEAT", blocking=False)
            if msg:
                self.is_connecting = False
                self.is_connected = True
                self.heartbeat_timer.stop()
                self.btn_connect.setText("Disconnect")
                self.btn_connect.setStyleSheet("background-color: #c0392b; color: white;")
                self.lbl_status.setText("Connected — receiving telemetry")
                self.data_timer.start(50)
        except Exception as e:
            self.lbl_status.setText(f"Error: {e}")
            self.disconnect()

    def refresh_combos(self):
        raw_keys = sorted(self.raw_telemetry.keys())
        display_names = [f"{FRIENDLY_NAMES.get(k, k)} [{k}]" for k in raw_keys]
        if display_names == self.last_combo_keys:
            return
        self.last_combo_keys = display_names
        self.display_to_raw = {d: r for d, r in zip(display_names, raw_keys)}
        for card in self.dynamic_cards:
            card.update_combo(self.display_to_raw)
        if not self.defaults_applied and self.display_to_raw:
            for card, key in zip(self.dynamic_cards, DEFAULT_DYNAMIC_KEYS):
                card.set_default(key, self.display_to_raw)
            self.defaults_applied = True

    def update_data(self):
        if not self.is_connected:
            return
        try:
            for _ in range(50):
                msg = self.master.recv_match(blocking=False)
                if not msg:
                    break
                for key, val in msg.to_dict().items():
                    if key != "mavpackettype":
                        self.raw_telemetry[f"{msg.get_type()}.{key}"] = val

            self.refresh_combos()

            raw_v = self.raw_telemetry.get("SYS_STATUS.voltage_battery")
            self.card_voltage.set_text(format_value("voltage", raw_v),
                                       status_color("SYS_STATUS.voltage_battery", raw_v))

            raw_a = self.raw_telemetry.get("SYS_STATUS.current_battery")
            self.card_current.set_text(format_value("current", raw_a),
                                       status_color("SYS_STATUS.current_battery", raw_a))

            base_mode = self.raw_telemetry.get("HEARTBEAT.base_mode", 0)
            armed = bool(base_mode & 128)
            if armed and not self.is_armed:
                self.start_arm_log()
            elif not armed and self.is_armed:
                self.stop_arm_log()
            self.is_armed = armed
            self.card_arm.set_text("ARMED" if armed else "DISARMED",
                                   "#e74c3c" if armed else "#2ecc71")

            mode = self.raw_telemetry.get("HEARTBEAT.custom_mode")
            self.card_mode.set_text(str(mode) if mode is not None else "--", "#3498db")

            for card in self.dynamic_cards:
                rk = card.current_raw_key()
                if rk:
                    raw_val = self.raw_telemetry.get(rk)
                    card.set_value(format_value(rk, raw_val), status_color(rk, raw_val))

        except Exception as e:
            self.lbl_status.setText(f"Error: {e}")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = Dashboard()
    window.show()
    sys.exit(app.exec_())
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

class CompactDynamicDashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PX4 telemetry dashboard (real-time)")
        self.resize(1200, 550)
        self.setStyleSheet("""
            QMainWindow, QWidget { background-color: #0a0a0a; color: #ecf0f1; font-family: Consolas, Arial; }
            QGroupBox { border: 2px solid #34495e; border-radius: 6px; margin-top: 10px; font-weight: bold; }
            QComboBox { background-color: #34495e; color: white; padding: 6px; font-size: 12px; }
            QPushButton { padding: 12px; font-weight: bold; font-size: 14px; border-radius: 5px; }
        """)

        self.master = None
        self.is_connected = False
        self.is_connecting = False
        self.log_file = None
        self.log_writer = None
        self.log_columns = None
        self.raw_telemetry = {}
        self.display_to_raw = {}
        self.last_combo_keys = []
        self.friendly_names = {
            "SYS_STATUS.voltage_battery": "Batt. Voltage",
            "SYS_STATUS.current_battery": "Batt. Current",
            "DISTANCE_SENSOR.current_distance": "Lidar distance",
            "VFR_HUD.alt": "Relative altitude",
            "GPS_RAW_INT.satellites_visible": "GPS Sat. visible"
        }
        # Threshold-ok - status szinek.
        #   Format: {"orange": value, "green": value, "invert": bool}
        #   invert=False (default): when higher is better -> red < orange <= green
        #   invert=True:            when lower  is better -> green < orange <= red
        self.thresholds = {
            "SYS_STATUS.voltage_battery":      {"orange": 13200, "green": 14800},          # mV, higher=better
            "SYS_STATUS.current_battery":      {"orange": 3000,  "green": 5000, "invert": True},  # cA, lower=better
            "GPS_RAW_INT.satellites_visible":  {"orange": 5,     "green": 8},              # count, higher=better
            "DISTANCE_SENSOR.current_distance":{"orange": 50,    "green": 200},            # cm, higher=better
        }
        self.dynamic_panels = []
        self.setup_ui()
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_data)
        self.connect_timer = QTimer()
        self.connect_timer.timeout.connect(self.try_heartbeat)
        self.log_timer = QTimer()
        self.log_timer.timeout.connect(self.write_log_row)

    def setup_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        top_bar = QHBoxLayout()
        top_bar.setSpacing(16)

        self.input_ip = QLineEdit("172.18.190.31")
        self.input_ip.setStyleSheet("background-color: #2c3e50; color: white; padding: 8px 10px; font-size: 14px; border: 1px solid #4a6278; border-radius: 4px;")
        self.input_ip.setFixedWidth(160)
        self.input_port = QLineEdit("14550")
        self.input_port.setStyleSheet("background-color: #2c3e50; color: white; padding: 8px 10px; font-size: 14px; border: 1px solid #4a6278; border-radius: 4px;")
        self.input_port.setFixedWidth(90)
        self.btn_connect = QPushButton("Connect (UDP)")
        self.btn_connect.setStyleSheet("background-color: #27ae60; color: white;")
        self.btn_connect.clicked.connect(self.toggle_connection)
        top_bar.addWidget(self.input_ip)
        top_bar.addWidget(self.input_port)
        top_bar.addWidget(self.btn_connect)
        top_bar.addStretch()
        main_layout.addLayout(top_bar)

        status_row = QHBoxLayout()
        self.card_voltage = self.create_fixed_card("Voltage", "0.00 V", "#f1c40f")
        self.card_current = self.create_fixed_card("Current", "0.0 A", "#f39c12")
        self.card_arm = self.create_fixed_card("Status", "DISARMED", "#2ecc71")
        self.card_mode = self.create_fixed_card("Flight Mode", "--", "#3498db")
        status_row.addWidget(self.card_voltage);
        status_row.addWidget(self.card_current)
        status_row.addWidget(self.card_arm);
        status_row.addWidget(self.card_mode)
        main_layout.addLayout(status_row)

        dynamic_row = QHBoxLayout()
        default_raw_keys = ["VFR_HUD.alt", "DISTANCE_SENSOR.current_distance", "GPS_RAW_INT.satellites_visible",
                            "ATTITUDE.yaw"]
        
        for i in range(4):
            panel = self.create_dynamic_card()
            self.dynamic_panels.append(panel)
            dynamic_row.addWidget(panel["group"])
            panel["default_raw"] = default_raw_keys[i]
        main_layout.addLayout(dynamic_row)

    def create_fixed_card(self, title, default_text, color):
        group = QGroupBox(title)
        layout = QVBoxLayout()
        lbl = QLabel(default_text);
        lbl.setFont(QFont("Arial", 24, QFont.Bold))
        lbl.setAlignment(Qt.AlignCenter);
        lbl.setStyleSheet(f"color: {color}; padding: 12px;")
        layout.addWidget(lbl);
        group.setLayout(layout);
        group.value_label = lbl
        return group

    def create_dynamic_card(self):
        group = QGroupBox("Empty field");
        layout = QVBoxLayout()
        combo = QComboBox();
        layout.addWidget(combo)
        lbl = QLabel("--");
        lbl.setFont(QFont("Arial", 28, QFont.Bold))
        lbl.setStyleSheet("color: #3498db; margin: 20px;");
        lbl.setAlignment(Qt.AlignCenter)
        layout.addWidget(lbl);
        group.setLayout(layout)
        return {"group": group, "combo": combo, "label": lbl, "default_raw": None}

    def toggle_connection(self):
        if not self.is_connected and not self.is_connecting:
            try:
                ip = self.input_ip.text().strip()
                port = self.input_port.text().strip()
                self.master = mavutil.mavlink_connection(f"udpout:{ip}:{port}")
                self.is_connecting = True
                self.input_ip.setEnabled(False)
                self.input_port.setEnabled(False)
                self.btn_connect.setText("Connecting...")
                self.btn_connect.setStyleSheet("background-color: #e67e22; color: white;")
                self.connect_timer.start(200)
            except Exception as e:
                print(e)
        else:
            self.is_connected = False
            self.is_connecting = False
            self.connect_timer.stop()
            self.timer.stop()
            self.stop_logging()
            self.input_ip.setEnabled(True)
            self.input_port.setEnabled(True)
            self.btn_connect.setText("Connect (UDP)")
            self.btn_connect.setStyleSheet("background-color: #27ae60; color: white;")

    def try_heartbeat(self):
        try:
            msg = self.master.recv_match(type='HEARTBEAT', blocking=False)
            if msg:
                self.is_connecting = False
                self.is_connected = True
                self.connect_timer.stop()
                self.btn_connect.setText("Disconnect")
                self.btn_connect.setStyleSheet("background-color: #c0392b; color: white;")
                self.timer.start(50)
                self.start_logging()
        except Exception as e:
            if getattr(e, "winerror", None) == 10022:
                return  # Windows: no data yet on udpout socket, keep polling
            print(f"Heartbeat error: {e}")

    def start_logging(self):
        os.makedirs("logs", exist_ok=True)
        filename = datetime.now().strftime("logs/flight_%Y-%m-%d_%H-%M-%S.csv")
        self.log_file = open(filename, "w", newline="")
        self.log_columns = None
        self.log_writer = None
        self.log_timer.start(1000)

    def stop_logging(self):
        self.log_timer.stop()
        if self.log_file:
            self.log_file.close()
            self.log_file = None
            self.log_writer = None
            self.log_columns = None

    def write_log_row(self):
        if not self.raw_telemetry or not self.log_file:
            return
        if self.log_columns is None:
            self.log_columns = ["timestamp"] + sorted(self.raw_telemetry.keys())
            self.log_writer = csv.DictWriter(self.log_file, fieldnames=self.log_columns, extrasaction="ignore")
            self.log_writer.writeheader()
        row = {"timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]}
        row.update(self.raw_telemetry)
        self.log_writer.writerow(row)
        self.log_file.flush()

    def refresh_combo_sources(self):
        raw_keys = sorted(self.raw_telemetry.keys())
        display_names = [f"{self.friendly_names.get(k, k)} [{k}]" for k in raw_keys]
        if display_names == self.last_combo_keys: return
        self.last_combo_keys = display_names
        self.display_to_raw = {f"{self.friendly_names.get(k, k)} [{k}]": k for k in raw_keys}
        for panel in self.dynamic_panels:
            combo = panel["combo"];
            current = combo.currentText()
            combo.blockSignals(True);
            combo.clear();
            combo.addItems(display_names)
            if current in display_names: combo.setCurrentText(current)
            combo.blockSignals(False)

    def get_status_color(self, raw_key, raw_value):
        """Return a hex color string based on the value's threshold status."""
        t = self.thresholds.get(raw_key)
        if t is None or raw_value is None:
            return "#3498db"  # default blue — no threshold defined
        invert = t.get("invert", False)
        if not invert:
            if raw_value >= t["green"]:   return "#2ecc71"  # green
            if raw_value >= t["orange"]:  return "#e67e22"  # orange
            return "#e74c3c"                                 # red
        else:
            if raw_value <= t["green"]:   return "#2ecc71"  # green
            if raw_value <= t["orange"]:  return "#e67e22"  # orange
            return "#e74c3c"                                 # red

    def format_value(self, raw_key, value):
        if value is None: return "--"
        if "voltage" in raw_key: return f"{value / 1000.0:.2f}V"
        if "current" in raw_key: return f"{value / 100.0:.1f}A"
        if "distance" in raw_key: return f"{value / 100.0:.2f}m"
        return str(value)

    def update_data(self):
        if not self.is_connected: return
        try:
            for _ in range(50):
                msg = self.master.recv_match(blocking=False)
                if not msg: break
                msg_dict = msg.to_dict()
                for key, val in msg_dict.items():
                    if key != "mavpackettype": self.raw_telemetry[f"{msg.get_type()}.{key}"] = val

            self.refresh_combo_sources()

            raw_v = self.raw_telemetry.get("SYS_STATUS.voltage_battery")
            raw_a = self.raw_telemetry.get("SYS_STATUS.current_battery")
            v = self.format_value("voltage", raw_v)
            a = self.format_value("current", raw_a)

            self.card_voltage.value_label.setText(v)
            self.card_voltage.value_label.setStyleSheet(
                f"color: {self.get_status_color('SYS_STATUS.voltage_battery', raw_v)}; padding: 12px;")

            self.card_current.value_label.setText(a)
            self.card_current.value_label.setStyleSheet(
                f"color: {self.get_status_color('SYS_STATUS.current_battery', raw_a)}; padding: 12px;")

            arm_text = self.card_arm.value_label.text()
            if arm_text == "ARMED":
                self.card_arm.value_label.setStyleSheet("color: #e74c3c; padding: 12px;")
            elif arm_text == "DISARMED":
                self.card_arm.value_label.setStyleSheet("color: #2ecc71; padding: 12px;")

            for panel in self.dynamic_panels:
                rk = self.display_to_raw.get(panel["combo"].currentText())
                if rk:
                    raw_val = self.raw_telemetry.get(rk)
                    panel["label"].setText(self.format_value(rk, raw_val))
                    color = self.get_status_color(rk, raw_val)
                    panel["label"].setStyleSheet(f"color: {color}; margin: 20px;")

        except Exception as e:
            print(f"Error: {e}")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = CompactDynamicDashboard()
    window.show()
    sys.exit(app.exec_())
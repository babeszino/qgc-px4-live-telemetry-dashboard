import sys
from flask import Flask
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QGroupBox, QPushButton, QComboBox, QLineEdit
)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont
from pymavlink import mavutil


# --- EREDETI DASHBOARD KIBŐVÍTVE ---
class CompactDynamicDashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        # ... (Az összes UI beállításod változatlan marad az eredeti kódból) ...
        self.setWindowTitle("PX4 live info bridge")
        self.resize(1200, 550)
        self.setStyleSheet("""
            QMainWindow, QWidget { background-color: #1a1a1a; color: #ecf0f1; font-family: Consolas, Arial; }
            QGroupBox { border: 2px solid #34495e; border-radius: 6px; margin-top: 10px; font-weight: bold; }
            QComboBox { background-color: #34495e; color: white; padding: 6px; font-size: 12px; }
            QPushButton { padding: 12px; font-weight: bold; font-size: 14px; border-radius: 5px; }
        """)

        self.master = None
        self.is_connected = False
        self.is_connecting = False
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
        self.dynamic_panels = []
        self.setup_ui()
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_data)
        self.connect_timer = QTimer()
        self.connect_timer.timeout.connect(self.try_heartbeat)

    def setup_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        top_bar = QHBoxLayout()
        top_bar.setSpacing(16)

        self.input_ip = QLineEdit("127.0.0.1")
        self.input_ip.setStyleSheet("background-color: #2c3e50; color: white; padding: 8px 10px; font-size: 14px; border: 1px solid #4a6278; border-radius: 4px;")
        self.input_ip.setFixedWidth(160)
        self.input_port = QLineEdit("14551")
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
                self.master = mavutil.mavlink_connection(f"udp:{ip}:{port}")
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
        except Exception as e:
            print(f"Heartbeat error: {e}")

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

            # Adatok előkészítése a telefonra
            v = self.format_value("voltage", self.raw_telemetry.get("SYS_STATUS.voltage_battery"))
            a = self.format_value("current", self.raw_telemetry.get("SYS_STATUS.current_battery"))
            l = self.format_value("distance", self.raw_telemetry.get("DISTANCE_SENSOR.current_distance"))
            alt = self.raw_telemetry.get("VFR_HUD.alt", "--")
            gps = self.raw_telemetry.get("GPS_RAW_INT.satellites_visible", "--")

            # UI frissítése laptopon
            self.card_voltage.value_label.setText(v)
            self.card_current.value_label.setText(a)

            for panel in self.dynamic_panels:
                rk = self.display_to_raw.get(panel["combo"].currentText())
                if rk: panel["label"].setText(self.format_value(rk, self.raw_telemetry.get(rk)))

        except Exception as e:
            print(f"Error: {e}")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = CompactDynamicDashboard()
    window.show()
    sys.exit(app.exec_())
import sys
import threading
from flask import Flask
from flask_socketio import SocketIO
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QGroupBox, QPushButton, QComboBox
)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont
from pymavlink import mavutil

# --- WEBSZERVER KONFIGURÁCIÓ ---
flask_app = Flask(__name__)
socketio = SocketIO(flask_app, cors_allowed_origins="*", async_mode='eventlet')
# Ez a HTML kód fog megjelenni a telefonodon
HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <title>Drone Mobile Link</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { background-color: #111; color: #2ecc71; font-family: sans-serif; text-align: center; padding: 10px; }
        .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }
        .card { background: #222; border: 1px solid #333; padding: 15px; border-radius: 10px; }
        .label { font-size: 0.8em; color: #888; text-transform: uppercase; }
        .value { font-size: 1.8em; font-weight: bold; display: block; margin-top: 5px; }
        h2 { color: #f1c40f; margin-bottom: 20px; border-bottom: 1px solid #333; padding-bottom: 10px; }
    </style>
</head>
<body>
    <h2>MOBILE TELEMETRY</h2>
    <div class="grid" id="data-grid">
        <div class="card"> <span class="label">Feszültség</span> <span class="value" id="volt">--</span> </div>
        <div class="card"> <span class="label">Áram</span> <span class="value" id="curr">--</span> </div>
        <div class="card"> <span class="label">Állapot</span> <span class="value" id="arm">--</span> </div>
        <div class="card"> <span class="label">Lidar</span> <span class="value" id="lidar">--</span> </div>
        <div class="card"> <span class="label">Magasság</span> <span class="value" id="alt">--</span> </div>
        <div class="card"> <span class="label">Műhold</span> <span class="value" id="gps">--</span> </div>
    </div>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/socketio/4.0.1/socket.io.js"></script>
    <script>
        var socket = io();
        socket.on('update', function(data) {
            if(data.volt) document.getElementById('volt').innerText = data.volt;
            if(data.curr) document.getElementById('curr').innerText = data.curr;
            if(data.arm) document.getElementById('arm').innerText = data.arm;
            if(data.lidar) document.getElementById('lidar').innerText = data.lidar;
            if(data.alt) document.getElementById('alt').innerText = data.alt;
            if(data.gps) document.getElementById('gps').innerText = data.gps;
        });
    </script>
</body>
</html>
"""


@flask_app.route('/')
def index():
    return HTML_TEMPLATE


def run_flask():
    # Az allow_unsafe_werkzeug=True kiküszöböli a hibát, ha nincs eventlet,
    # de az eventlettel lesz igazán stabil.
    socketio.run(flask_app, host='0.0.0.0', port=5000, log_output=False, allow_unsafe_werkzeug=True)


# --- EREDETI DASHBOARD KIBŐVÍTVE ---
class CompactDynamicDashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        # ... (Az összes UI beállításod változatlan marad az eredeti kódból) ...
        self.setWindowTitle("PX4 Server & Mobile Bridge")
        self.resize(1200, 550)
        self.setStyleSheet("""
            QMainWindow, QWidget { background-color: #1a1a1a; color: #ecf0f1; font-family: Consolas, Arial; }
            QGroupBox { border: 2px solid #34495e; border-radius: 6px; margin-top: 10px; font-weight: bold; }
            QComboBox { background-color: #34495e; color: white; padding: 6px; font-size: 12px; }
            QPushButton { padding: 12px; font-weight: bold; font-size: 14px; border-radius: 5px; }
        """)

        self.master = None
        self.is_connected = False
        self.raw_telemetry = {}
        self.display_to_raw = {}
        self.last_combo_keys = []
        self.friendly_names = {
            "SYS_STATUS.voltage_battery": "Akkufeszültség",
            "SYS_STATUS.current_battery": "Áramfelvétel",
            "DISTANCE_SENSOR.current_distance": "Lidar távolság",
            "VFR_HUD.alt": "Relatív magasság",
            "GPS_RAW_INT.satellites_visible": "GPS műholdak"
        }
        self.dynamic_panels = []
        self.setup_ui()
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_data)

    def setup_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        top_bar = QHBoxLayout()
        self.btn_connect = QPushButton("KAPCSOLÓDÁS (UDP:14551)")
        self.btn_connect.setStyleSheet("background-color: #27ae60; color: white;")
        self.btn_connect.clicked.connect(self.toggle_connection)
        top_bar.addWidget(self.btn_connect)
        self.lbl_status = QLabel("SERVER ACTIVE - PORT 5000")
        top_bar.addWidget(self.lbl_status)
        main_layout.addLayout(top_bar)

        status_row = QHBoxLayout()
        self.card_voltage = self.create_fixed_card("Feszültség", "0.00 V", "#f1c40f")
        self.card_current = self.create_fixed_card("Áram", "0.0 A", "#f39c12")
        self.card_arm = self.create_fixed_card("Állapot", "DISARMED", "#2ecc71")
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
        group = QGroupBox("Szabad mező");
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
        if not self.is_connected:
            try:
                self.master = mavutil.mavlink_connection("udp:127.0.0.1:14551")
                self.is_connected = True
                self.btn_connect.setText("KAPCSOLAT BONTÁSA")
                self.timer.start(50)
            except Exception as e:
                print(e)
        else:
            self.is_connected = False
            self.timer.stop()

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

            # ADATKÜLDÉS A TELEFONRA
            socketio.emit('update', {
                'volt': v, 'curr': a, 'lidar': l,
                'alt': f"{alt}m", 'gps': f"{gps}db",
                'arm': "ARMED" if (self.raw_telemetry.get("HEARTBEAT.base_mode", 0) & 128) else "DISARMED"
            })

            for panel in self.dynamic_panels:
                rk = self.display_to_raw.get(panel["combo"].currentText())
                if rk: panel["label"].setText(self.format_value(rk, self.raw_telemetry.get(rk)))

        except Exception as e:
            print(f"Hiba: {e}")


if __name__ == "__main__":
    # Indítjuk a webes hidat egy külön szálon
    threading.Thread(target=run_flask, daemon=True).start()

    app = QApplication(sys.argv)
    window = CompactDynamicDashboard()
    window.show()
    sys.exit(app.exec_())
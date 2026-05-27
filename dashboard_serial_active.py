import sys
import csv
import os
import math
import collections
from datetime import datetime
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QGroupBox, QPushButton, QComboBox, QLineEdit, QProgressBar,
    QGridLayout, QTextEdit, QTabWidget, QTableWidget, QTableWidgetItem, QHeaderView
)
from PyQt5.QtCore import QTimer, Qt, pyqtSignal
from PyQt5.QtGui import QFont
from pymavlink import mavutil


BETTER_NAMES = {
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
    ("arm_status",        "HEARTBEAT.base_mode"),
    ("lat",               "GPS_RAW_INT.lat"),
    ("lon",               "GPS_RAW_INT.lon"),
    ("vib_x",             "VIBRATION.vibration_x"),
    ("vib_y",             "VIBRATION.vibration_y"),
    ("vib_z",             "VIBRATION.vibration_z"),
    ("wp_current",        "MISSION_CURRENT.seq"),
    ("wp_distance_m",     "NAV_CONTROLLER_OUTPUT.wp_dist"),
]

DEFAULT_DYNAMIC_KEYS = [ 
    "VFR_HUD.alt",
    "DISTANCE_SENSOR.current_distance",
    "GPS_RAW_INT.satellites_visible",
    "ATTITUDE.yaw",
]

STYLE = """
    QMainWindow, QWidget {
        background-color: #EEEEEE;
        color: #6FCF94;
        font-family: Consolas, Arial;
    }
    QGroupBox {
        border: 2px solid #2FA084;
        border-radius: 6px;
        margin-top: 10px;
        font-weight: bold;
        padding: 6px;
        color: #EEEEEE;
        background-color: #6FCF94;
    }
    QGroupBox::title { padding: 0 4px; }
    QLabel { background-color: transparent; }
    QComboBox {
        background-color: #2FA084;
        color: #EEEEEE;
        padding: 6px;
        font-size: 15px;
        border: none;
        border-radius: 3px;
    }
    QComboBox::drop-down { border: none; }
    QPushButton {
        padding: 10px 20px;
        font-weight: bold;
        font-size: 14px;
        border-radius: 5px;
        border: none;
    }
    QLineEdit {
        background-color: #6FCF94;
        color: #EEEEEE;
        padding: 8px 10px;
        font-size: 14px;
        border: 1px solid #2FA084;
        border-radius: 4px;
    }
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


def indicator_color(raw_key, raw_value):
    if raw_key not in THRESHOLDS or raw_value is None:
        return "#7f8c8d"
    return status_color(raw_key, raw_value)


def format_value(raw_key, value):
    if value is None:
        return "--"
    
    if "voltage"  in raw_key: return f"{value / 1000.0:.2f} V"
    if "current"  in raw_key: return f"{value / 100.0:.1f} A"
    if "distance" in raw_key: return f"{value / 100.0:.2f} m"
    if "yaw" in raw_key or "pitch" in raw_key or "roll" in raw_key:
        return f"{math.degrees(value):.1f}°"
    if isinstance(value, float): return f"{value:.2f}"
    return str(value)


def haversine_distance(lat1, lon1, lat2, lon2):
    R = 6371000  # earth radius in meters
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    return R * 2 * math.asin(math.sqrt(a))


PX4_MAIN_MODES = {
    1: "MANUAL",
    2: "ALTCTL",
    3: "POSCTL",
    4: "AUTO",
    5: "ACRO",
    6: "OFFBOARD",
    7: "STABILIZED",
}

PX4_SUB_MODES_AUTO = {
    1: "READY",
    2: "TAKEOFF",
    3: "LOITER",
    4: "MISSION",
    5: "RTL",
    6: "LAND",
    8: "FOLLOW TARGET",
    9: "PRECLAND",
}

def decode_px4_mode(custom_mode):
    main = (custom_mode >> 16) & 0xFF
    sub  = (custom_mode >> 24) & 0xFF
    main_name = PX4_MAIN_MODES.get(main, f"MODE({main})")
    if main == 4 and sub in PX4_SUB_MODES_AUTO:
        return f"AUTO:{PX4_SUB_MODES_AUTO[sub]}"
    return main_name


SENSOR_BITS = {
    0x00000001: "3D Gyro",
    0x00000002: "3D Accel",
    0x00000004: "3D Mag",
    0x00000008: "Barometer",
    0x00000020: "GPS",
    0x00000040: "Optical Flow",
    0x00010000: "RC Receiver",
    0x00020000: "Gyro 2",
    0x00040000: "Accel 2",
    0x00080000: "Mag 2",
    0x00200000: "AHRS",
    0x02000000: "Battery",
    0x04000000: "Proximity",
    0x10000000: "Pre-arm",
}


class VibrationMonitor(QGroupBox):
    def __init__(self):
        super().__init__("Vibration Monitor")
        layout = QGridLayout()
        layout.setSpacing(8)
        self.bars  = {}
        self.labels = {}
        axes = ["X", "Y", "Z"]
        for i, axis in enumerate(axes):
            lbl_name = QLabel(f"Vib {axis}")
            lbl_name.setStyleSheet("color: #1F6F5F; font-size: 15px;")
            lbl_name.setFixedWidth(40)

            bar = QProgressBar()
            bar.setMinimum(0)
            bar.setMaximum(60)
            bar.setValue(0)
            bar.setTextVisible(False)
            bar.setFixedHeight(22)
            bar.setStyleSheet("""
                QProgressBar {
                    background-color: #6FCF94;
                    border: 1px solid #2FA084;
                    border-radius: 3px;
                }
                QProgressBar::chunk {
                    background-color: #2ecc71;
                    border-radius: 2px;
                }
            """)

            lbl_val = QLabel("--")
            lbl_val.setStyleSheet("color: #EEEEEE; font-size: 15px;")
            lbl_val.setFixedWidth(60)

            layout.addWidget(lbl_name, i, 0)
            layout.addWidget(bar,      i, 1)
            layout.addWidget(lbl_val,  i, 2)
            self.bars[axis]   = bar
            self.labels[axis] = lbl_val

        clipping_row = len(axes)
        self.lbl_clipping = QLabel("Clipping:  --")
        self.lbl_clipping.setStyleSheet("color: #1F6F5F; font-size: 15px;")
        layout.addWidget(self.lbl_clipping, clipping_row, 0, 1, 3)

        self.setLayout(layout)

    def update(self, vib_x, vib_y, vib_z, clipping):
        for axis, val in zip(["X", "Y", "Z"], [vib_x, vib_y, vib_z]):
            bar   = self.bars[axis]
            label = self.labels[axis]
            if val is None:
                label.setText("--")
                bar.setValue(0)
                continue
            if val < 15:
                color = "#2ecc71"
            elif val < 30:
                color = "#e67e22"
            else:
                color = "#e74c3c"
            bar.setStyleSheet(f"""
                QProgressBar {{
                    background-color: #6FCF94;
                    border: 1px solid #2FA084;
                    border-radius: 3px;
                }}
                QProgressBar::chunk {{
                    background-color: {color};
                    border-radius: 2px;
                }}
            """)
            bar.setValue(min(60, int(val)))
            label.setText(f"{val:.1f}")

        if clipping is not None:
            clip_color = "#e74c3c" if clipping > 0 else "#2ecc71"
            self.lbl_clipping.setStyleSheet(f"color: {clip_color}; font-size: 15px;")
            self.lbl_clipping.setText(f"Clipping:  {int(clipping)}")
        else:
            self.lbl_clipping.setText("Clipping:  --")
            self.lbl_clipping.setStyleSheet("color: #1F6F5F; font-size: 15px;")


class MaxValuesPanel(QGroupBox):
    def __init__(self):
        super().__init__("Session Peaks")
        layout = QGridLayout()
        layout.setSpacing(6)
        self.values = {
            "Max Altitude":  ("--", "#3498db"),
            "Max Speed":     ("--", "#9b59b6"),
            "Max Current":   ("--", "#e67e22"),
            "Max Distance":  ("--", "#1abc9c"),
        }
        self.labels = {}
        for i, (name, (default, color)) in enumerate(self.values.items()):
            lbl_name = QLabel(name)
            lbl_name.setStyleSheet("color: #1F6F5F; font-size: 14px;")
            lbl_val = QLabel(default)
            lbl_val.setFont(QFont("Arial", 15, QFont.Bold))
            lbl_val.setStyleSheet(f"color: {color};")
            lbl_val.setAlignment(Qt.AlignRight)
            layout.addWidget(lbl_name, i, 0)
            layout.addWidget(lbl_val,  i, 1)
            self.labels[name] = lbl_val
        self.setLayout(layout)

    def update(self, name, value_str):
        if name in self.labels:
            self.labels[name].setText(value_str)

    def reset(self):
        for lbl in self.labels.values():
            lbl.setText("--")


class MotorOutputPanel(QGroupBox):
    def __init__(self):
        super().__init__("Motor Outputs")
        layout = QGridLayout()
        layout.setSpacing(8)
        self.bars = []
        self.labels = []
        for i in range(4):
            lbl_name = QLabel(f"M{i+1}")
            lbl_name.setStyleSheet("color: #1F6F5F; font-size: 15px;")
            lbl_name.setFixedWidth(24)

            bar = QProgressBar()
            bar.setMinimum(1000)
            bar.setMaximum(2000)
            bar.setValue(1000)
            bar.setTextVisible(False)
            bar.setFixedHeight(22)
            bar.setStyleSheet("""
                QProgressBar {
                    background-color: #6FCF94;
                    border: 1px solid #2FA084;
                    border-radius: 3px;
                }
                QProgressBar::chunk {
                    background-color: #2980b9;
                    border-radius: 2px;
                }
            """)

            lbl_val = QLabel("--")
            lbl_val.setStyleSheet("color: #EEEEEE; font-size: 15px;")
            lbl_val.setFixedWidth(50)

            layout.addWidget(lbl_name, i, 0)
            layout.addWidget(bar,      i, 1)
            layout.addWidget(lbl_val,  i, 2)
            self.bars.append(bar)
            self.labels.append(lbl_val)

        self.setLayout(layout)

    def update(self, outputs):
        for i, (bar, lbl) in enumerate(zip(self.bars, self.labels)):
            val = outputs[i] if i < len(outputs) else None
            if val is None or val < 900:
                bar.setValue(1000)
                bar.setStyleSheet(bar.styleSheet().replace("#2980b9", "#2FA084"))
                lbl.setText("--")
            else:
                clamped = max(1000, min(2000, int(val)))
                pct = (clamped - 1000) / 1000.0
                if pct < 0.3:
                    color = "#2ecc71"
                elif pct < 0.7:
                    color = "#e67e22"
                else:
                    color = "#e74c3c"
                bar.setStyleSheet(f"""
                    QProgressBar {{
                        background-color: #6FCF94;
                        border: 1px solid #2FA084;
                        border-radius: 3px;
                    }}
                    QProgressBar::chunk {{
                        background-color: {color};
                        border-radius: 2px;
                    }}
                """)
                bar.setValue(clamped)
                lbl.setText(f"{clamped}")


class StatusTextPanel(QGroupBox):
    def __init__(self):
        super().__init__("PX4 Messages")
        layout = QVBoxLayout()

        btn_clear = QPushButton("Clear")
        btn_clear.setFixedWidth(70)
        btn_clear.setStyleSheet("background-color: #2FA084; color: #EEEEEE; padding: 4px; font-size: 14px;")
        btn_clear.clicked.connect(self.clear)

        top = QHBoxLayout()
        top.addWidget(QLabel("Recent PX4 messages:"))
        top.addStretch()
        top.addWidget(btn_clear)
        layout.addLayout(top)

        self.log = QTextEdit()
        self.log.setReadOnly(True)
        self.log.setFixedHeight(160)
        self.log.setStyleSheet("""
            QTextEdit {
                background-color: #6FCF94;
                color: #EEEEEE;
                font-family: Consolas, monospace;
                font-size: 15px;
                border: 1px solid #2FA084;
                border-radius: 4px;
            }
        """)
        layout.addWidget(self.log)
        self.setLayout(layout)

    def add(self, severity, text):
        if severity <= 2:
            color = "#e74c3c"
        elif severity <= 4:
            color = "#e67e22"
        else:
            color = "#ecf0f1"

        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log.append(f'<span style="color:#7f8c8d;">[{timestamp}]</span> <span style="color:{color};">{text}</span>')
        self.log.verticalScrollBar().setValue(self.log.verticalScrollBar().maximum())

    def clear(self):
        self.log.clear()


class SensorHealthPanel(QGroupBox):
    def __init__(self):
        super().__init__("Sensor Health")
        self.grid = QGridLayout()
        self.grid.setSpacing(4)
        self.setLayout(self.grid)
        self.indicators = {}

    def update(self, present, enabled, health):
        for i in reversed(range(self.grid.count())):
            self.grid.itemAt(i).widget().deleteLater()
        self.indicators.clear()

        if present is None:
            lbl = QLabel("No data")
            lbl.setStyleSheet("color: #1F6F5F;")
            self.grid.addWidget(lbl, 0, 0)
            return

        active_sensors = {
            bit: name for bit, name in SENSOR_BITS.items()
            if (present & bit) and (enabled & bit)
        }

        col, row = 0, 0
        for bit, name in active_sensors.items():
            healthy = bool(health & bit)
            color = "#2ecc71" if healthy else "#e74c3c"
            lbl = QLabel(f"● {name}")
            lbl.setStyleSheet(f"color: {color}; font-size: 15px;")
            lbl.setFixedHeight(20)
            self.grid.addWidget(lbl, row, col)
            col += 1
            if col > 2:
                col = 0
                row += 1


class FixedCard(QGroupBox):
    def __init__(self, title, default_text, color):
        super().__init__(title)
        layout = QVBoxLayout()
        self.label = QLabel(default_text)
        self.label.setFont(QFont("Arial", 17, QFont.Bold))
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setStyleSheet(f"color: {color}; padding: 10px;")
        layout.addWidget(self.label)
        dot_row = QHBoxLayout()
        dot_row.addStretch()
        self.indicator = QLabel()
        self.indicator.setFixedSize(12, 12)
        self.indicator.setStyleSheet("background-color: #2FA084; border-radius: 6px;")
        dot_row.addWidget(self.indicator)
        layout.addLayout(dot_row)
        self.setLayout(layout)

    def set_text(self, text, color=None):
        self.label.setText(text)
        if color:
            self.label.setStyleSheet(f"color: {color}; padding: 10px;")

    def set_indicator(self, color):
        self.indicator.setStyleSheet(f"background-color: {color}; border-radius: 6px;")


class SearchableComboBox(QWidget):
    currentTextChanged = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)

        self.search_box = QLineEdit()
        self.search_box.setPlaceholderText("Search...")
        self.search_box.setStyleSheet("""
            background-color: #6FCF94;
            color: #EEEEEE;
            padding: 3px 6px;
            font-size: 14px;
            border: 1px solid #2FA084;
            border-radius: 3px;
        """)
        self.search_box.textChanged.connect(self._on_search)

        self.combo = QComboBox()
        self.combo.currentTextChanged.connect(self.currentTextChanged)

        layout.addWidget(self.search_box)
        layout.addWidget(self.combo)
        self._all_items = []

    def _on_search(self, text):
        current = self.combo.currentText()
        self.combo.blockSignals(True)
        self.combo.clear()
        filtered = [i for i in self._all_items if text.lower() in i.lower()]
        self.combo.addItems(filtered)
        if current in filtered:
            self.combo.setCurrentText(current)
        self.combo.blockSignals(False)
        self.currentTextChanged.emit(self.combo.currentText())

    def update_items(self, items, preserve=None):
        self._all_items = list(items)
        self.combo.blockSignals(True)
        self.combo.clear()
        text = self.search_box.text()
        filtered = [i for i in self._all_items if text.lower() in i.lower()] if text else self._all_items
        self.combo.addItems(filtered)
        if preserve and preserve in filtered:
            self.combo.setCurrentText(preserve)
        self.combo.blockSignals(False)
        self.currentTextChanged.emit(self.combo.currentText())

    def currentText(self):
        return self.combo.currentText()

    def setCurrentText(self, text):
        self.combo.setCurrentText(text)


class DynamicCard(QGroupBox):
    def __init__(self):
        super().__init__("—")
        layout = QVBoxLayout()
        self.combo = SearchableComboBox()
        self.combo.currentTextChanged.connect(self._on_combo_changed)
        layout.addWidget(self.combo)
        self.label = QLabel("--")
        self.label.setFont(QFont("Arial", 19, QFont.Bold))
        self.label.setStyleSheet("color: #1F6F5F; margin: 8px;")
        self.label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.label)
        dot_row = QHBoxLayout()
        dot_row.addStretch()
        self.indicator = QLabel()
        self.indicator.setFixedSize(12, 12)
        self.indicator.setStyleSheet("background-color: #2FA084; border-radius: 6px;")
        dot_row.addWidget(self.indicator)
        layout.addLayout(dot_row)
        self.setLayout(layout)
        self.display_to_raw = {}

    def _on_combo_changed(self, text):
        raw_key = self.display_to_raw.get(text)
        self.setTitle(BETTER_NAMES.get(raw_key, raw_key) if raw_key else "—")

    def update_combo(self, display_to_raw):
        self.display_to_raw = display_to_raw
        current = self.combo.currentText()
        self.combo.update_items(list(display_to_raw.keys()), preserve=current)
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

    def set_indicator(self, color):
        self.indicator.setStyleSheet(f"background-color: {color}; border-radius: 6px;")


class MissionCard(QGroupBox):
    def __init__(self):
        super().__init__("Mission Status")
        layout = QVBoxLayout()
        layout.setSpacing(8)

        self.lbl_waypoint = QLabel("Waypoint: --")
        self.lbl_waypoint.setFont(QFont("Arial", 17, QFont.Bold))
        self.lbl_waypoint.setStyleSheet("color: #3498db;")
        self.lbl_waypoint.setAlignment(Qt.AlignCenter)

        self.lbl_distance = QLabel("Distance to WP: --")
        self.lbl_distance.setFont(QFont("Arial", 15))
        self.lbl_distance.setStyleSheet("color: #EEEEEE;")
        self.lbl_distance.setAlignment(Qt.AlignCenter)

        self.lbl_reached = QLabel("")
        self.lbl_reached.setFont(QFont("Arial", 13))
        self.lbl_reached.setStyleSheet("color: #2ecc71;")
        self.lbl_reached.setAlignment(Qt.AlignCenter)

        self.progress = QProgressBar()
        self.progress.setTextVisible(True)
        self.progress.setStyleSheet("""
            QProgressBar {
                border: 1px solid #2FA084;
                border-radius: 4px;
                background-color: #6FCF94;
                color: white;
                text-align: center;
                height: 20px;
            }
            QProgressBar::chunk {
                background-color: #1F6F5F;
                border-radius: 3px;
            }
        """)
        self.progress.setValue(0)
        self.progress.setFormat("No mission")

        layout.addWidget(self.lbl_waypoint)
        layout.addWidget(self.lbl_distance)
        layout.addWidget(self.lbl_reached)
        layout.addWidget(self.progress)
        self.setLayout(layout)
        self.last_reached = -1
        self.total_wps = 0

    def update(self, current_seq, wp_dist, last_reached, total_wps):
        self.total_wps = total_wps

        if current_seq is not None:
            self.lbl_waypoint.setText(f"Waypoint: {current_seq}")
        else:
            self.lbl_waypoint.setText("Waypoint: --")

        if wp_dist is not None:
            self.lbl_distance.setText(f"Distance to WP: {wp_dist:.1f} m")
        else:
            self.lbl_distance.setText("Distance to WP: --")

        if last_reached is not None and last_reached != self.last_reached:
            self.last_reached = last_reached
            self.lbl_reached.setText(f"✔ Reached WP {last_reached}")
        elif last_reached is None:
            self.lbl_reached.setText("")

        if total_wps > 0 and current_seq is not None:
            pct = int((current_seq / total_wps) * 100)
            self.progress.setMaximum(total_wps)
            self.progress.setValue(current_seq)
            self.progress.setFormat(f"WP {current_seq} / {total_wps}  ({pct}%)")
        else:
            self.progress.setValue(0)
            self.progress.setFormat("No mission loaded")


class Dashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("px4-qgc-dashboard")
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

        self.flight_seconds = 0
        self.flight_timer = QTimer()
        self.flight_timer.timeout.connect(self._tick_flight_timer)
        self.was_armed = False

        self.home_lat = None
        self.home_lon = None

        self.msg_count = 0
        self.msg_per_sec = 0
        self.health_timer = QTimer()
        self.health_timer.timeout.connect(self._tick_health)

        self.current_buffer = collections.deque(maxlen=60)
        self.mah_used = 0.0
        self.BATTERY_CAPACITY_MAH = 4200
        self.last_current_time = None

        self.max_altitude = None
        self.max_speed    = None
        self.max_current  = None
        self.max_distance = None

        self.mavlink_table_timer = QTimer()
        self.mavlink_table_timer.timeout.connect(self.update_mavlink_table)

        self.setup_ui()

    def setup_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        main_layout = QVBoxLayout(root)
        main_layout.setSpacing(10)

        top = QHBoxLayout()
        top.setSpacing(10)

        lbl_port = QLabel("Listen port:")
        lbl_port.setStyleSheet("color: #1F6F5F; font-size: 14px;")
        self.input_port = QLineEdit("14540")
        self.input_port.setFixedWidth(90)

        self.btn_connect = QPushButton("Connect")
        self.btn_connect.setFixedWidth(130)
        self.btn_connect.setStyleSheet("background-color: #27ae60; color: white;")
        self.btn_connect.clicked.connect(self.toggle_connection)

        self.lbl_status = QLabel("Waiting...")
        self.lbl_status.setStyleSheet("color: #1F6F5F; font-size: 14px;")

        top.addWidget(lbl_port)
        top.addWidget(self.input_port)
        top.addWidget(self.btn_connect)
        top.addSpacing(16)
        top.addWidget(self.lbl_status)
        top.addStretch()
        main_layout.addLayout(top)

        self.main_tabs = QTabWidget()
        self.main_tabs.setStyleSheet("""
            QTabWidget::pane { border: 2px solid #2FA084; border-radius: 4px; }
            QTabBar::tab { background: #2FA084; color: #EEEEEE; padding: 8px 24px; font-size: 14px; font-weight: bold; min-width: 140px; }
            QTabBar::tab:selected { background: #6FCF94; color: #EEEEEE; border-top: 2px solid #1F6F5F; }
        """)
        self.main_tabs.tabBar().setExpanding(True)
        self.main_tabs.currentChanged.connect(self._on_main_tab_changed)
        main_layout.addWidget(self.main_tabs)

        dashboard_widget = QWidget()
        layout = QVBoxLayout(dashboard_widget)
        layout.setSpacing(10)

        self.card_voltage = FixedCard("Voltage",     "--",      "#f1c40f")
        self.card_current = FixedCard("Current",     "--",      "#f39c12")
        self.card_arm     = FixedCard("Arm Status",  "UNKNOWN", "#95a5a6")
        self.card_mode    = FixedCard("Flight Mode", "--",      "#3498db")
        fixed_row1 = QHBoxLayout()
        for c in [self.card_voltage, self.card_current, self.card_arm, self.card_mode]:
            fixed_row1.addWidget(c)
        layout.addLayout(fixed_row1)

        self.card_timer        = FixedCard("Flight Timer",  "00:00:00", "#9b59b6")
        self.card_home_dist    = FixedCard("Home Distance", "--",       "#1abc9c")
        self.card_link         = FixedCard("Link Health",   "--",       "#95a5a6")
        self.card_battery_time = FixedCard("Time Left",     "--",       "#9b59b6")
        fixed_row2 = QHBoxLayout()
        for c in [self.card_timer, self.card_home_dist, self.card_link, self.card_battery_time]:
            fixed_row2.addWidget(c)
        layout.addLayout(fixed_row2)

        dynamic_row = QHBoxLayout()
        self.dynamic_cards = [DynamicCard() for _ in range(4)]
        for card in self.dynamic_cards:
            dynamic_row.addWidget(card)
        layout.addLayout(dynamic_row)

        self.inner_tabs = QTabWidget()
        self.inner_tabs.tabBar().setExpanding(True)
        self.inner_tabs.setStyleSheet("""
            QTabWidget::pane { border: 2px solid #2FA084; border-radius: 4px; }
            QTabBar::tab { background: #2FA084; color: #EEEEEE; padding: 6px 12px; font-size: 15px; }
            QTabBar::tab:selected { background: #6FCF94; color: #EEEEEE; border-top: 2px solid #1F6F5F; }
        """)

        health_widget = QWidget()
        health_layout = QHBoxLayout(health_widget)
        health_layout.setSpacing(10)
        self.sensor_panel = SensorHealthPanel()
        self.motor_panel  = MotorOutputPanel()
        self.vib_monitor  = VibrationMonitor()
        health_layout.addWidget(self.sensor_panel, stretch=2)
        health_layout.addWidget(self.motor_panel,  stretch=1)
        health_layout.addWidget(self.vib_monitor,  stretch=1)

        log_widget = QWidget()
        log_layout = QVBoxLayout(log_widget)
        self.statustext_panel = StatusTextPanel()
        log_layout.addWidget(self.statustext_panel)

        mission_widget = QWidget()
        mission_layout = QHBoxLayout(mission_widget)
        mission_layout.setSpacing(10)
        self.mission_card = MissionCard()
        self.max_panel    = MaxValuesPanel()
        mission_layout.addWidget(self.mission_card, stretch=2)
        mission_layout.addWidget(self.max_panel,    stretch=1)

        self.inner_tabs.addTab(health_widget,   "⬤  Health")
        self.inner_tabs.addTab(log_widget,      "⬤  Log")
        self.inner_tabs.addTab(mission_widget,  "⬤  Mission / Session")
        layout.addWidget(self.inner_tabs)

        self.main_tabs.addTab(dashboard_widget, "Dashboard")

        inspector_widget = QWidget()
        inspector_layout = QVBoxLayout(inspector_widget)
        inspector_layout.setSpacing(8)

        search_row = QHBoxLayout()
        lbl_search = QLabel("Filter:")
        lbl_search.setStyleSheet("color: #1F6F5F; font-size: 14px;")
        self.mavlink_search = QLineEdit()
        self.mavlink_search.setPlaceholderText("Type to filter by message or field name...")
        self.mavlink_search.setStyleSheet("""
            background-color: #6FCF94; color: #EEEEEE;
            padding: 6px 10px; font-size: 15px;
            border: 1px solid #2FA084; border-radius: 4px;
        """)
        self.mavlink_search.textChanged.connect(self.update_mavlink_table)
        search_row.addWidget(lbl_search)
        search_row.addWidget(self.mavlink_search)
        inspector_layout.addLayout(search_row)

        self.mavlink_table = QTableWidget()
        self.mavlink_table.setColumnCount(3)
        self.mavlink_table.setHorizontalHeaderLabels(["Message", "Field", "Value"])
        self.mavlink_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.Interactive)
        self.mavlink_table.horizontalHeader().setSectionResizeMode(1, QHeaderView.Stretch)
        self.mavlink_table.horizontalHeader().setSectionResizeMode(2, QHeaderView.Interactive)
        self.mavlink_table.setColumnWidth(0, 200)
        self.mavlink_table.setColumnWidth(2, 130)
        self.mavlink_table.verticalHeader().setVisible(False)
        self.mavlink_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.mavlink_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.mavlink_table.setSortingEnabled(False)
        self.mavlink_table.setStyleSheet("""
            QTableWidget {
                background-color: #6FCF94; color: #111844;
                font-family: Consolas, monospace; font-size: 15px;
                border: 1px solid #2FA084; gridline-color: #2FA084;
                font-weight: bold;
            }
            QTableWidget::item:selected { background-color: #2FA084; color: #EEEEEE; }
            QHeaderView::section {
                background-color: #2FA084; color: #EEEEEE;
                padding: 6px; border: none; font-weight: bold; font-size: 15px;
            }
        """)
        inspector_layout.addWidget(self.mavlink_table)

        self.main_tabs.addTab(inspector_widget, "MAVLink Inspector")

    def update_mavlink_table(self):
        if not self.raw_telemetry:
            return

        try:
            snapshot = dict(self.raw_telemetry)
        except Exception:
            return

        search = self.mavlink_search.text().lower()

        rows = []
        for key, value in sorted(snapshot.items()):
            parts = key.split(".", 1)
            msg_type = parts[0]
            field    = parts[1] if len(parts) > 1 else ""
            try:
                formatted = format_value(key, value)
            except Exception:
                formatted = str(value)
            if search and search not in key.lower() and search not in formatted.lower():
                continue
            rows.append((key, msg_type, field, formatted))

        current_keys = [r[0] for r in rows]

        if current_keys != getattr(self, "_mavlink_last_keys", None):
            self._mavlink_last_keys = current_keys
            self.mavlink_table.setUpdatesEnabled(False)
            self.mavlink_table.setRowCount(len(rows))
            for i, (key, msg_type, field, formatted) in enumerate(rows):
                self.mavlink_table.setItem(i, 0, QTableWidgetItem(msg_type))
                self.mavlink_table.setItem(i, 1, QTableWidgetItem(field))
                self.mavlink_table.setItem(i, 2, QTableWidgetItem(formatted))
            self.mavlink_table.setUpdatesEnabled(True)
        else:
            self.mavlink_table.setUpdatesEnabled(False)
            for i, (key, msg_type, field, formatted) in enumerate(rows):
                item = self.mavlink_table.item(i, 2)
                if item is None:
                    self.mavlink_table.setItem(i, 2, QTableWidgetItem(formatted))
                elif item.text() != formatted:
                    item.setText(formatted)
            self.mavlink_table.setUpdatesEnabled(True)

    def _on_main_tab_changed(self, index):
        if index == 1:
            self.mavlink_table_timer.start(500)
            QTimer.singleShot(0, self.update_mavlink_table)
        else:
            self.mavlink_table_timer.stop()

    def _update_battery_estimate(self, raw_a):
        if raw_a is None:
            return

        now = datetime.now()
        current_amps = raw_a / 100.0

        if self.last_current_time is not None:
            dt_hours = (now - self.last_current_time).total_seconds() / 3600.0
            self.mah_used += current_amps * 1000.0 * dt_hours
        self.last_current_time = now

        self.current_buffer.append(current_amps)

        mah_remaining = max(0, self.BATTERY_CAPACITY_MAH - self.mah_used)

        if len(self.current_buffer) >= 5 and sum(self.current_buffer) > 0:
            avg_current = sum(self.current_buffer) / len(self.current_buffer)
            hours_left = (mah_remaining / 1000.0) / avg_current
            minutes_left = hours_left * 60.0
            if minutes_left > 99:
                time_str = ">99 min"
                color = "#2ecc71"
            elif minutes_left > 10:
                time_str = f"{minutes_left:.1f} min"
                color = "#2ecc71"
            elif minutes_left > 5:
                time_str = f"{minutes_left:.1f} min"
                color = "#e67e22"
            else:
                time_str = f"{minutes_left:.1f} min"
                color = "#e74c3c"
            pct = int((mah_remaining / self.BATTERY_CAPACITY_MAH) * 100)
            self.card_battery_time.set_text(f"{time_str}\n{pct}% left", color)
        else:
            self.card_battery_time.set_text("Calculating...", "#95a5a6")

    def _tick_health(self):
        self.msg_per_sec = self.msg_count
        self.msg_count = 0

        if self.msg_per_sec >= 50:
            color = "#2ecc71"
        elif self.msg_per_sec >= 20:
            color = "#e67e22"
        else:
            color = "#e74c3c"
        self.card_link.set_text(f"{self.msg_per_sec} msg/s", color)

    def _tick_flight_timer(self):
        self.flight_seconds += 1
        h = self.flight_seconds // 3600
        m = (self.flight_seconds % 3600) // 60
        s = self.flight_seconds % 60
        self.card_timer.set_text(f"{h:02d}:{m:02d}:{s:02d}", "#9b59b6")

    def _set_inputs_enabled(self, enabled):
        self.input_port.setEnabled(enabled)

    def toggle_connection(self):
        if self.is_connected or self.is_connecting:
            self.disconnect()
            return
        try:
            port = self.input_port.text().strip()
            conn_str = f"udpin:0.0.0.0:{port}"
            status_msg = f"Listening on UDP port {port}..."

            self.master = mavutil.mavlink_connection(conn_str)
            self.is_connecting = True
            self.defaults_applied = False
            self._set_inputs_enabled(False)
            self.btn_connect.setText("Connecting…")
            self.btn_connect.setFixedWidth(155)
            self.btn_connect.setStyleSheet("background-color: #e67e22; color: white;")
            self.lbl_status.setText(status_msg)
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
            if col == "flight_mode":
                raw_mode = self.raw_telemetry.get("HEARTBEAT.custom_mode")
                row[col] = decode_px4_mode(raw_mode) if raw_mode is not None else ""
            elif col == "arm_status":
                base_mode = self.raw_telemetry.get("HEARTBEAT.base_mode", 0)
                row[col] = "ARMED" if (base_mode & 128) else "DISARMED"
            else:
                row[col] = self.raw_telemetry.get(raw_key, "")
        self.log_writer.writerow(row)
        self.log_file.flush()

    def disconnect(self):
        self.is_connected = False
        self.is_connecting = False
        if self.master:
            try:
                self.master.close()
            except Exception:
                pass
            self.master = None
        self.heartbeat_timer.stop()
        self.data_timer.stop()
        self.health_timer.stop()
        self.mavlink_table_timer.stop()
        self.card_link.set_text("--", "#95a5a6")
        self.stop_arm_log()
        self.raw_telemetry.clear()
        self.last_combo_keys = []
        self.home_lat = None
        self.home_lon = None
        self.mah_used = 0.0
        self.current_buffer.clear()
        self.last_current_time = None
        self._set_inputs_enabled(True)
        self.btn_connect.setText("Connect")
        self.btn_connect.setFixedWidth(130)
        self.btn_connect.setStyleSheet("background-color: #27ae60; color: white;")
        self.lbl_status.setText("Disconnected.")

    def try_heartbeat(self):
        try:
            msg = self.master.recv_match(type="HEARTBEAT", blocking=False)

            if msg:
                self.is_connecting = False
                self.is_connected = True
                self.heartbeat_timer.stop()
                self.btn_connect.setFixedWidth(155)
                self.btn_connect.setText("Disconnect")
                self.btn_connect.setStyleSheet("background-color: #c0392b; color: white;")
                self.lbl_status.setText("Connected — receiving telemetry")
                self.data_timer.start(50)
                self.health_timer.start(1000)

        except Exception as e:
            self.lbl_status.setText(f"Error: {e}")
            self.disconnect() 

    def refresh_combos(self):
        raw_keys = sorted(self.raw_telemetry.keys())
        display_names = [f"{BETTER_NAMES.get(k, k)} [{k}]" for k in raw_keys]

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
                self.msg_count += 1
                if msg.get_type() == "STATUSTEXT":
                    self.statustext_panel.add(msg.severity, msg.text.strip())
                for key, val in msg.to_dict().items():
                    if key != "mavpackettype":
                        if msg.get_type() == "HEARTBEAT" and msg.get_srcComponent() != 1:
                            continue
                        self.raw_telemetry[f"{msg.get_type()}.{key}"] = val

            self.refresh_combos()

            raw_v = self.raw_telemetry.get("SYS_STATUS.voltage_battery")
            self.card_voltage.set_text(format_value("voltage", raw_v),
                                       status_color("SYS_STATUS.voltage_battery", raw_v))
            self.card_voltage.set_indicator(indicator_color("SYS_STATUS.voltage_battery", raw_v))

            raw_a = self.raw_telemetry.get("SYS_STATUS.current_battery")
            self.card_current.set_text(format_value("current", raw_a),
                                       status_color("SYS_STATUS.current_battery", raw_a))
            self.card_current.set_indicator(indicator_color("SYS_STATUS.current_battery", raw_a))
            self._update_battery_estimate(raw_a)

            base_mode = self.raw_telemetry.get("HEARTBEAT.base_mode", 0)
            armed = bool(base_mode & 128)

            if armed and not self.is_armed:
                self.start_arm_log()

            elif not armed and self.is_armed:
                self.stop_arm_log()

            self.is_armed = armed

            if armed and not self.was_armed:
                self.flight_seconds = 0
                self.flight_timer.start(1000)
                self.mah_used = 0.0
                self.current_buffer.clear()
                self.last_current_time = None
                self.max_altitude = None
                self.max_speed    = None
                self.max_current  = None
                self.max_distance = None
                self.max_panel.reset()

            elif not armed and self.was_armed:
                self.flight_timer.stop()

            self.was_armed = armed
            arm_color = "#e74c3c" if armed else "#2ecc71"
            self.card_arm.set_text("ARMED" if armed else "DISARMED", arm_color)
            self.card_arm.set_indicator(arm_color)

            mode = self.raw_telemetry.get("HEARTBEAT.custom_mode")
            mode_str = decode_px4_mode(mode) if mode is not None else "--"
            self.card_mode.set_text(mode_str, "#3498db")
            self.card_mode.set_indicator("#7f8c8d")

            for card in self.dynamic_cards:
                rk = card.current_raw_key()
                if rk:
                    raw_val = self.raw_telemetry.get(rk)
                    card.set_value(format_value(rk, raw_val), status_color(rk, raw_val))
                    card.set_indicator(indicator_color(rk, raw_val))

            home_lat_raw = self.raw_telemetry.get("HOME_POSITION.latitude")
            home_lon_raw = self.raw_telemetry.get("HOME_POSITION.longitude")
            if home_lat_raw is not None and self.home_lat is None:
                self.home_lat = home_lat_raw / 1e7
                self.home_lon = home_lon_raw / 1e7

            cur_lat_raw = self.raw_telemetry.get("GPS_RAW_INT.lat")
            cur_lon_raw = self.raw_telemetry.get("GPS_RAW_INT.lon")
            if self.home_lat is not None and cur_lat_raw is not None:
                cur_lat = cur_lat_raw / 1e7
                cur_lon = cur_lon_raw / 1e7
                dist = haversine_distance(self.home_lat, self.home_lon, cur_lat, cur_lon)

                if dist >= 1000:
                    dist_str = f"{dist / 1000:.2f} km"

                else:
                    dist_str = f"{dist:.1f} m"

                color = "#2ecc71" if dist < 50 else "#e67e22" if dist < 200 else "#e74c3c"
                self.card_home_dist.set_text(dist_str, color)

            else:
                self.card_home_dist.set_text("No GPS", "#95a5a6")

            current_seq  = self.raw_telemetry.get("MISSION_CURRENT.seq")
            wp_dist      = self.raw_telemetry.get("NAV_CONTROLLER_OUTPUT.wp_dist")
            last_reached = self.raw_telemetry.get("MISSION_ITEM_REACHED.seq")
            total_wps    = self.raw_telemetry.get("MISSION_COUNT.count", 0)
            self.mission_card.update(current_seq, wp_dist, last_reached, total_wps)

            present = self.raw_telemetry.get("SYS_STATUS.onboard_control_sensors_present")
            enabled = self.raw_telemetry.get("SYS_STATUS.onboard_control_sensors_enabled")
            health  = self.raw_telemetry.get("SYS_STATUS.onboard_control_sensors_health")
            self.sensor_panel.update(present, enabled, health)

            outputs = [
                self.raw_telemetry.get(f"SERVO_OUTPUT_RAW.servo{i+1}_raw")
                for i in range(4)
            ]
            self.motor_panel.update(outputs)

            alt = self.raw_telemetry.get("VFR_HUD.alt")
            if alt is not None:
                if self.max_altitude is None or alt > self.max_altitude:
                    self.max_altitude = alt
                    self.max_panel.update("Max Altitude", f"{alt:.1f} m")

            spd = self.raw_telemetry.get("VFR_HUD.groundspeed")
            if spd is not None:
                if self.max_speed is None or spd > self.max_speed:
                    self.max_speed = spd
                    self.max_panel.update("Max Speed", f"{spd:.1f} m/s")

            raw_a = self.raw_telemetry.get("SYS_STATUS.current_battery")
            if raw_a is not None:
                amps = raw_a / 100.0
                if self.max_current is None or amps > self.max_current:
                    self.max_current = amps
                    self.max_panel.update("Max Current", f"{amps:.1f} A")

            if self.home_lat is not None:
                cur_lat_raw = self.raw_telemetry.get("GPS_RAW_INT.lat")
                cur_lon_raw = self.raw_telemetry.get("GPS_RAW_INT.lon")
                if cur_lat_raw is not None:
                    dist = haversine_distance(self.home_lat, self.home_lon,
                                              cur_lat_raw / 1e7, cur_lon_raw / 1e7)
                    if self.max_distance is None or dist > self.max_distance:
                        self.max_distance = dist
                        self.max_panel.update("Max Distance", f"{dist:.1f} m")

            vib_x    = self.raw_telemetry.get("VIBRATION.vibration_x")
            vib_y    = self.raw_telemetry.get("VIBRATION.vibration_y")
            vib_z    = self.raw_telemetry.get("VIBRATION.vibration_z")
            clipping = self.raw_telemetry.get("VIBRATION.clipping_0")
            self.vib_monitor.update(vib_x, vib_y, vib_z, clipping)

        except Exception as e:
            self.lbl_status.setText(f"Error: {e}")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = Dashboard()
    window.show()
    sys.exit(app.exec_())
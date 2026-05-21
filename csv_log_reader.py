import sys
import csv
import math
from datetime import datetime

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QFileDialog, QTableWidget, QTableWidgetItem,
    QHeaderView, QGroupBox, QGridLayout, QFrame, QSizePolicy
)

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont, QColor

# PX4 mode decoding
PX4_MAIN_MODES = {
    1: "MANUAL", 2: "ALTCTL", 3: "POSCTL", 4: "AUTO",
    5: "ACRO", 6: "OFFBOARD", 7: "STABILIZED",
}

PX4_SUB_MODES_AUTO = {
    1: "READY", 2: "TAKEOFF", 3: "LOITER", 4: "MISSION",
    5: "RTL", 6: "LAND", 8: "FOLLOW", 9: "PRECLAND"
}

def decode_px4_mode(raw):
    try:
        v = int(float(raw))
        main = (v >> 16) & 0xff
        sub = (v >> 24) & 0xff
        name = PX4_MAIN_MODES.get(main, f"MODE({main})")
        if main == 4 and sub in PX4_SUB_MODES_AUTO:
            return f"AUTO:{PX4_SUB_MODES_AUTO[sub]}"
        
        return name
    
    except Exception:
        return str(raw)

# column formatting

COLUMN_HEADERS = {
    "timestamp":    "Timestamp",
    "voltage_mV":     "Voltage",
    "current_cA":     "Current",
    "altitude_m":     "Altitude",
    "groundspeed_ms": "Speed",
    "gps_satellites": "GPS Sats",
    "lidar_cm":       "Lidar",
    "roll_rad":       "Roll",
    "pitch_rad":      "Pitch",
    "yaw_rad":        "Yaw",
    "flight_mode":    "Flight Mode",
    "arm_status":     "Arm Status",
    "lat":            "Latitude",
    "lon":            "Longitude",
    "vib_x":          "Vib X",
    "vib_y":          "Vib Y",
    "vib_z":          "Vib Z",
    "wp_current":     "Waypoint",
    "wp_distance_m":  "WP Dist",
}

def format_cell(col, raw):
    if raw is None or str(raw).strip() == "":
        return "--"
    
    try:
        if col == "voltage_mV":
            return f"{float(raw) / 1000:.2f} V"
        if col == "current_cA":
            return f"{float(raw) / 100:.1f} A"
        if col in ("roll_rad", "pitch_rad", "yaw_rad"):
            return f"{math.degrees(float(raw)):.1f}°"
        if col == "altitude_m":
            return f"{float(raw):.1f} m"
        if col == "groundspeed_ms":
            return f"{float(raw):.1f} m/s"
        if col == "lidar_cm":
            return f"{float(raw) / 100:.2f} m"
        if col == "flight_mode":
            return decode_px4_mode(raw)
        if col == "arm_status":
            return "ARMED" if (int(float(raw)) & 128) else "DISARMED"
        if col in ("lat", "lon"):
            v = float(raw)
            return f"{v / 1e7:.6f}" if abs(v) > 1000 else f"{v:.6f}"
        if col == "wp_distance_m":
            return f"{float(raw):.1f} m"
        if col == "wp_current":
            return str(int(float(raw)))
        if col in ("vib_x", "vib_y", "vib_z"):
            return f"{float(raw):.3f}"
        if col == "gps_satellites":
            return str(int(float(raw)))
        
    except Exception:
        pass

    return str(raw)

class NumericTableWidgetItem(QTableWidgetItem):
    def __init__(self, display_text, sort_value):
        super().__init__(display_text)
        self._sort_value = sort_value

    def __lt__(self, other):
        if isinstance(other, NumericTableWidgetItem):
            try:
                return float(self._sort_value) < float(other._sort_value)
            except (TypeError, ValueError):
                pass
        return super().__lt__(other)


class SummaryCard(QGroupBox):
    def __init__(self, title, value, color="#3498db"):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 8, 12, 8)
 
        lbl_title = QLabel(title)
        lbl_title.setStyleSheet("color: #7f8c8d; font-size: 11px;")
        lbl_title.setAlignment(Qt.AlignCenter)
 
        self.lbl_value = QLabel(value)
        self.lbl_value.setFont(QFont("Consolas", 15, QFont.Bold))
        self.lbl_value.setStyleSheet(f"color: {color};")
        self.lbl_value.setAlignment(Qt.AlignCenter)
 
        layout.addWidget(lbl_title)
        layout.addWidget(self.lbl_value)
        self.setStyleSheet(f"""
            QGroupBox {{
                border: 2px solid #2c3e50;
                border-radius: 6px;
                background-color: #111;
            }}
        """)
 
    def set(self, value):
        self.lbl_value.setText(value)

# main window
class LogViewer(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PX4 Flight Log Viewer")
        self.setMinimumSize(1100, 650)
        self.resize(1300, 750)
        self.setStyleSheet("""
            QMainWindow, QWidget {
                background-color: #0a0a0a;
                color: #ecf0f1;
                font-family: Consolas, Arial;
            }
            QGroupBox {
                border: 2px solid #2c3e50;
                border-radius: 6px;
                margin-top: 10px;
                font-weight: bold;
                padding: 6px;
                color: #ecf0f1;
            }
            QGroupBox::title { padding: 0 4px; }
            QPushButton {
                padding: 9px 22px;
                font-weight: bold;
                font-size: 13px;
                border-radius: 5px;
                border: none;
            }
            QTableWidget {
                background-color: #0d0d0d;
                color: #ecf0f1;
                font-family: Consolas, monospace;
                font-size: 12px;
                border: 1px solid #2c3e50;
                gridline-color: #1a1a1a;
            }
            QTableWidget::item:selected { background-color: #2c3e50; }
            QHeaderView::section {
                background-color: #1a2535;
                color: #ecf0f1;
                padding: 6px;
                border: none;
                font-weight: bold;
                font-size: 12px;
            }
            QScrollBar:vertical {
                background: #1a1a1a; width: 10px; border-radius: 5px;
            }
            QScrollBar::handle:vertical {
                background: #2c3e50; border-radius: 5px;
            }
        """)
        self._rows = []
        self._columns = []
        self._setup_ui()
 
    def _setup_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)
        layout.setSpacing(10)
        layout.setContentsMargins(14, 14, 14, 14)
 
        # top bar
        top = QHBoxLayout()
 
        self.lbl_file = QLabel("No file loaded")
        self.lbl_file.setStyleSheet("color: #7f8c8d; font-size: 13px;")
 
        self.btn_load = QPushButton("LOAD LOG FILE")
        self.btn_load.setStyleSheet("background-color: #2980b9; color: white;")
        self.btn_load.clicked.connect(self.load_file)
        self.btn_load.setFixedWidth(180)
 
        top.addWidget(self.lbl_file)
        top.addStretch()
        top.addWidget(self.btn_load)
        layout.addLayout(top)
 
        # summary cards
        summary_box = QGroupBox("Flight Summary")
        summary_layout = QHBoxLayout(summary_box)
        summary_layout.setSpacing(10)
 
        self.cards = {
            "date":     SummaryCard("Date",           "--",    "#ecf0f1"),
            "duration": SummaryCard("Duration",       "--",    "#9b59b6"),
            "rows":     SummaryCard("Log Rows",        "--",    "#3498db"),
            "max_alt":  SummaryCard("Max Altitude",   "--",    "#1abc9c"),
            "max_spd":  SummaryCard("Max Speed",      "--",    "#e67e22"),
            "min_volt": SummaryCard("Min Voltage",    "--",    "#f1c40f"),
            "max_curr": SummaryCard("Max Current",    "--",    "#e74c3c"),
            "gps_avg":  SummaryCard("Avg GPS Sats",   "--",    "#2ecc71"),
        }
        for card in self.cards.values():
            summary_layout.addWidget(card)
 
        layout.addWidget(summary_box)
 
        # table
        self.table = QTableWidget()
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.setSelectionBehavior(QTableWidget.SelectRows)
        self.table.verticalHeader().setVisible(False)
        self.table.horizontalHeader().setStretchLastSection(False)
        self.table.setSortingEnabled(True)
        self.table.setAlternatingRowColors(True)
        self.table.setStyleSheet(self.table.styleSheet() + """
            QTableWidget { alternate-background-color: #0f0f0f; }
        """)
        layout.addWidget(self.table)
 
        # status bar
        self.lbl_status = QLabel("Load a log file to get started.")
        self.lbl_status.setStyleSheet("color: #7f8c8d; font-size: 12px;")
        layout.addWidget(self.lbl_status)
 
    # | loading |
 
    def load_file(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Open Flight Log", "", "CSV Files (*.csv);;All Files (*)"
        )
        if not path:
            return
 
        try:
            with open(path, newline="", encoding="utf-8") as f:
                reader = csv.DictReader(f)
                self._columns = reader.fieldnames or []
                self._rows = list(reader)
        except Exception as e:
            self.lbl_status.setText(f"Error loading file: {e}")
            return
 
        filename = path.split("/")[-1].split("\\")[-1]
        self.lbl_file.setText(f"  {filename}")
        self.lbl_file.setStyleSheet("color: #2ecc71; font-size: 13px; font-weight: bold;")
 
        self._populate_table()
        self._populate_summary()
        self.lbl_status.setText(
            f"Loaded {len(self._rows)} rows  ·  {len(self._columns)} columns  ·  {path}"
        )
 
    def _populate_table(self):
        self.table.setSortingEnabled(False)
        self.table.setUpdatesEnabled(False)
 
        self.table.setColumnCount(len(self._columns))
        self.table.setHorizontalHeaderLabels(
            [COLUMN_HEADERS.get(c, c) for c in self._columns]
        )
        self.table.setRowCount(len(self._rows))
 
        for r, row in enumerate(self._rows):
            for c, col in enumerate(self._columns):
                raw = row.get(col, "")
                text = format_cell(col, raw) if col != "timestamp" else raw
 
                try:
                    sort_val = float(raw)
                    item = NumericTableWidgetItem(text, sort_val)
                except (ValueError, TypeError):
                    item = QTableWidgetItem(text)

                item.setTextAlignment(Qt.AlignCenter)

                if col == "arm_status":
                    item.setForeground(QColor("#e74c3c" if text == "ARMED" else "#2ecc71"))

                self.table.setItem(r, c, item)
 
        # fit columns
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        if "timestamp" in self._columns:
            ts_idx = self._columns.index("timestamp")
            self.table.horizontalHeader().setSectionResizeMode(ts_idx, QHeaderView.Stretch)
 
        self.table.setUpdatesEnabled(True)
        self.table.setSortingEnabled(True)
 
    def _populate_summary(self):
        if not self._rows:
            return
 
        # date + duration (from timestamps)
        try:
            fmt = "%Y-%m-%d %H:%M:%S.%f"
            fmt2 = "%Y-%m-%d %H:%M:%S"
            def parse_ts(s):
                for f in (fmt, fmt2):
                    try: return datetime.strptime(s, f)
                    except: pass
                return None
 
            timestamps = [parse_ts(r.get("timestamp", "")) for r in self._rows]
            timestamps = [t for t in timestamps if t]
            if timestamps:
                t0, t1 = timestamps[0], timestamps[-1]
                self.cards["date"].set(t0.strftime("%Y-%m-%d"))
                secs = int((t1 - t0).total_seconds())
                self.cards["duration"].set(f"{secs//60:02d}:{secs%60:02d}")
        except Exception:
            pass
 
        self.cards["rows"].set(str(len(self._rows)))
 
        def col_floats(col):
            vals = []
            for r in self._rows:
                try: vals.append(float(r.get(col, "")))
                except: pass
            return vals
 
        # max altitude
        alts = col_floats("altitude_m")
        if alts:
            self.cards["max_alt"].set(f"{max(alts):.1f} m")
 
        # max speed
        spds = col_floats("groundspeed_ms")
        if spds:
            self.cards["max_spd"].set(f"{max(spds):.1f} m/s")
 
        # min voltage
        volts = col_floats("voltage_mV")
        if volts:
            self.cards["min_volt"].set(f"{min(volts)/1000:.2f} V")
 
        # max current
        currs = col_floats("current_cA")
        if currs:
            self.cards["max_curr"].set(f"{max(currs)/100:.1f} A")
 
        # avg gps
        sats = col_floats("gps_satellites")
        if sats:
            self.cards["gps_avg"].set(f"{sum(sats)/len(sats):.1f}")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    win = LogViewer()
    win.show()
    sys.exit(app.exec_())
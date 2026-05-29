import sys
import csv
import math
from datetime import datetime

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QFileDialog, QTableWidget, QTableWidgetItem,
    QHeaderView, QGroupBox, QGridLayout, QSizePolicy, QCheckBox,
    QScrollArea, QTabWidget, QFrame, QLineEdit
)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont, QColor

import matplotlib
matplotlib.use("Qt5Agg")
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure

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
        main = (v >> 16) & 0xFF
        sub  = (v >> 24) & 0xFF
        name = PX4_MAIN_MODES.get(main, f"MODE({main})")
        if main == 4 and sub in PX4_SUB_MODES_AUTO:
            return f"AUTO:{PX4_SUB_MODES_AUTO[sub]}"
        return name
    except Exception:
        return str(raw)

COLUMN_HEADERS = {
    "timestamp":      "Timestamp",
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
    "motor1_us":      "Motor 1",
    "motor2_us":      "Motor 2",
    "motor3_us":      "Motor 3",
    "motor4_us":      "Motor 4",
}

PLOTTABLE_COLUMNS = {
    "altitude_m":     ("Altitude",    "m"),
    "groundspeed_ms": ("Speed",       "m/s"),
    "voltage_mV":     ("Voltage",     "V"),
    "current_cA":     ("Current",     "A"),
    "gps_satellites": ("GPS Sats",    "sats"),
    "lidar_cm":       ("Lidar",       "m"),
    "roll_rad":       ("Roll",        "deg"),
    "pitch_rad":      ("Pitch",       "deg"),
    "yaw_rad":        ("Yaw",         "deg"),
    "vib_x":          ("Vib X",       ""),
    "vib_y":          ("Vib Y",       ""),
    "vib_z":          ("Vib Z",       ""),
    "wp_distance_m":  ("WP Distance", "m"),
    "motor1_us":      ("Motor 1",     "µs"),
    "motor2_us":      ("Motor 2",     "µs"),
    "motor3_us":      ("Motor 3",     "µs"),
    "motor4_us":      ("Motor 4",     "µs"),
}

PLOT_COLORS = [
    "#3498db", "#2ecc71", "#e74c3c", "#f1c40f",
    "#9b59b6", "#1abc9c", "#e67e22", "#e91e63",
    "#00bcd4", "#8bc34a", "#ff5722", "#607d8b",
]

FLIGHT_COLORS = [
    "#A67DA8",  # dark lilac
    "#3498db",  # blue
    "#e74c3c",  # red
    "#2ecc71",  # green
    "#e67e22",  # orange
    "#9b59b6",  # purple
    "#1abc9c",  # teal
    "#e91e63",  # pink
]

DEFAULT_PLOT_COLS = ["altitude_m", "groundspeed_ms", "voltage_mV"]


def format_cell(col, raw):
    if raw is None or str(raw).strip() == "":
        return "--"
    try:
        if col == "voltage_mV":     return f"{float(raw) / 1000:.2f} V"
        if col == "current_cA":     return f"{float(raw) / 100:.1f} A"
        if col in ("roll_rad", "pitch_rad", "yaw_rad"):
            return f"{math.degrees(float(raw)):.1f}°"
        if col == "altitude_m":     return f"{float(raw):.1f} m"
        if col == "groundspeed_ms": return f"{float(raw):.1f} m/s"
        if col == "lidar_cm":       return f"{float(raw) / 100:.2f} m"
        if col == "flight_mode":    return decode_px4_mode(raw)
        if col == "arm_status":
            return "ARMED" if (int(float(raw)) & 128) else "DISARMED"
        if col in ("lat", "lon"):
            v = float(raw)
            return f"{v / 1e7:.6f}" if abs(v) > 1000 else f"{v:.6f}"
        if col == "wp_distance_m":  return f"{float(raw):.1f} m"
        if col in ("motor1_us", "motor2_us", "motor3_us", "motor4_us"):
            return f"{int(float(raw))} µs"
        if col == "wp_current":     return str(int(float(raw)))

        # if col in ("vib_x", "vib_y", "vib_z"): return f"{float(raw):.3f}"
        if col in ("vib_x", "vib_y", "vib_z"):
            v = float(raw)
            return f"{v:.2e}" if v < 0.0001 else f"{v:.5f}"

        if col == "gps_satellites": return str(int(float(raw)))
    except Exception:
        pass
    return str(raw)


def to_plot_value(col, raw):
    try:
        v = float(raw)
        if col == "voltage_mV":     return v / 1000
        if col == "current_cA":     return v / 100
        if col in ("roll_rad", "pitch_rad", "yaw_rad"): return math.degrees(v)
        if col == "lidar_cm":       return v / 100
        if col in ("lat", "lon"):   return v / 1e7 if abs(v) > 1000 else v
        return v
    except Exception:
        return None


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
        lbl_title.setStyleSheet("color: #8B7B8C; font-size: 13px;")
        lbl_title.setAlignment(Qt.AlignCenter)
        self.lbl_value = QLabel(value)
        self.lbl_value.setFont(QFont("Consolas", 17, QFont.Bold))
        self.lbl_value.setStyleSheet(f"color: {color};")
        self.lbl_value.setAlignment(Qt.AlignCenter)
        layout.addWidget(lbl_title)
        layout.addWidget(self.lbl_value)
        self.setStyleSheet("""
            QGroupBox {
                border: 2px solid #C8A2C9;
                border-radius: 6px;
                background-color: #FDF9FF;
            }
        """)

    def set(self, value):
        self.lbl_value.setText(value)


class TelemetryCanvas(FigureCanvas):
    def __init__(self):
        self.fig = Figure(facecolor="#FDF9FF")
        super().__init__(self.fig)
        self.setMinimumHeight(300)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def plot(self, time_labels, series):
        self.fig.clear()

        if not series:
            ax = self.fig.add_subplot(111)
            ax.set_facecolor("#FFFFFF")
            ax.text(0.5, 0.5,
                    "Select columns on the left and click Plot.",
                    ha="center", va="center", color="#8B7B8C",
                    fontsize=13, transform=ax.transAxes)
            ax.set_xticks([])
            ax.set_yticks([])
            for spine in ax.spines.values():
                spine.set_edgecolor("#C8A2C9")
            self.draw()
            return

        n = len(series)
        axes = self.fig.subplots(n, 1, sharex=True)
        if n == 1:
            axes = [axes]

        self.fig.subplots_adjust(
            left=0.09, right=0.97,
            top=0.97,  bottom=0.09,
            hspace=0.12
        )

        for ax, (_, name, unit, values, color) in zip(axes, series):
            ax.set_facecolor("#FFFFFF")
            xs = [i for i, v in enumerate(values) if v is not None]
            ys = [v for v in values if v is not None]
            if xs:
                ax.plot(xs, ys, color=color, linewidth=1.4, alpha=0.95)
                ax.fill_between(xs, ys, alpha=0.08, color=color)
            ylabel = f"{name}" + (f"\n({unit})" if unit else "")
            ax.set_ylabel(ylabel, color="#8B7B8C", fontsize=9, labelpad=4)
            ax.tick_params(colors="#6B5B6E", labelsize=8)
            for spine in ax.spines.values():
                spine.set_edgecolor("#C8A2C9")
            ax.grid(True, color="#E8D5E9", linewidth=0.7, linestyle="--")

        total = len(time_labels)
        if total:
            step = max(1, total // 8)
            ticks = list(range(0, total, step))
            axes[-1].set_xticks(ticks)
            axes[-1].set_xticklabels(
                [str(time_labels[i])[-12:] for i in ticks],
                rotation=25, ha="right", fontsize=8, color="#6B5B6E"
            )

        self.draw()

class CompareCanvas(FigureCanvas):
    def __init__(self):
        self.fig = Figure(facecolor="#FDF9FF")
        super().__init__(self.fig)
        self.setMinimumHeight(300)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def plot(self, flights):
        """
        flights: list of {name, color, series}
        series:  list of (col, display_name, unit, xs, ys)
        """
        self.fig.clear()

        if not flights:
            ax = self.fig.add_subplot(111)
            ax.set_facecolor("#FFFFFF")
            ax.text(0.5, 0.5,
                    "Load flight files, select columns and click Compare.",
                    ha="center", va="center", color="#8B7B8C",
                    fontsize=13, transform=ax.transAxes)
            ax.set_xticks([])
            ax.set_yticks([])
            for spine in ax.spines.values():
                spine.set_edgecolor("#C8A2C9")
            self.draw()
            return

        all_cols = list(dict.fromkeys(
            col for f in flights for col, *_ in f["series"]
        ))
        if not all_cols:
            self.draw()
            return

        n = len(all_cols)
        axes = self.fig.subplots(n, 1, sharex=True)
        if n == 1:
            axes = [axes]

        self.fig.subplots_adjust(
            left=0.09, right=0.97, top=0.97, bottom=0.09, hspace=0.15
        )

        line_styles = ["-", "--", "-.", ":", "-", "--", "-.", ":"]

        for ax, col in zip(axes, all_cols):
            ax.set_facecolor("#FFFFFF")
            display_name = unit = ""
            for fi, flight in enumerate(flights):
                match = next((s for s in flight["series"] if s[0] == col), None)
                if match is None:
                    continue
                _, display_name, unit, xs, ys = match
                ls = line_styles[fi % len(line_styles)]
                ax.plot(xs, ys, color=flight["color"], linewidth=1.5,
                        linestyle=ls, alpha=0.9, label=flight["name"])
                ax.fill_between(xs, ys, alpha=0.06, color=flight["color"])

            ylabel = display_name + (f"\n({unit})" if unit else "")
            ax.set_ylabel(ylabel, color="#8B7B8C", fontsize=9, labelpad=4)
            ax.tick_params(colors="#6B5B6E", labelsize=8)
            for spine in ax.spines.values():
                spine.set_edgecolor("#C8A2C9")
            ax.grid(True, color="#E8D5E9", linewidth=0.7, linestyle="--")
            ax.legend(fontsize=8, facecolor="#FDF9FF",
                      edgecolor="#C8A2C9", labelcolor="#2D2D2D",
                      loc="upper right")

        axes[-1].set_xlabel("Time (s)", color="#8B7B8C", fontsize=9)
        axes[-1].tick_params(colors="#6B5B6E", labelsize=8)
        self.draw()

STYLE = """
    QMainWindow, QWidget {
        background-color: #F8F4FF;
        color: #2D2D2D;
        font-family: Consolas, Arial;
    }
    QGroupBox {
        border: 2px solid #C8A2C9; border-radius: 6px;
        margin-top: 10px; font-weight: bold;
        padding: 6px; color: #2D2D2D;
    }
    QGroupBox::title { padding: 0 4px; }
    QPushButton {
        padding: 9px 22px; font-weight: bold;
        font-size: 15px; border-radius: 5px; border: none;
    }
    QTabWidget::pane { border: 2px solid #C8A2C9; border-radius: 4px; }
    QTabBar::tab {
        background: #E8D5E9; color: #8B7B8C;
        padding: 10px 32px; font-size: 15px; font-weight: bold; min-width: 160px;
    }
    QTabBar::tab:selected {
        background: #FDF9FF; color: #2D2D2D;
        border-top: 2px solid #C8A2C9;
    }
    QTableWidget {
        background-color: #FFFFFF; color: #2D2D2D;
        font-family: Consolas, monospace; font-size: 14px;
        border: 1px solid #C8A2C9; gridline-color: #E8D5E9;
    }
    QTableWidget::item:selected { background-color: #C8A2C9; color: #FDF9FF; }
    QHeaderView::section {
        background-color: #E8D5E9; color: #2D2D2D;
        padding: 6px; border: none;
        font-weight: bold; font-size: 14px;
    }
    QScrollBar:vertical { background: #E8D5E9; width: 10px; border-radius: 5px; }
    QScrollBar::handle:vertical { background: #C8A2C9; border-radius: 5px; }
    QCheckBox { color: #2D2D2D; font-size: 14px; padding: 3px 0; }
    QCheckBox::indicator {
        width: 14px; height: 14px;
        border: 1px solid #C8A2C9; border-radius: 3px;
        background-color: #FFFFFF;
    }
    QCheckBox::indicator:checked { background-color: #C8A2C9; border-color: #C8A2C9; }
    QLineEdit {
        background-color: #FFFFFF; color: #2D2D2D;
        border: 1px solid #C8A2C9; border-radius: 3px; padding: 3px 6px;
    }
"""


class LogViewer(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("px4-dash-csv-log-reader")
        self.setMinimumSize(1100, 650)
        self.resize(1350, 780)
        self.setStyleSheet(STYLE)
        self._rows              = []
        self._columns           = []
        self._checkboxes        = {}
        self._compare_flights   = [{"rows": [], "name": None},
                                    {"rows": [], "name": None}]
        self._compare_cbs       = {}
        self._setup_ui()

    def _setup_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        main = QVBoxLayout(root)
        main.setSpacing(10)
        main.setContentsMargins(14, 14, 14, 14)

        top = QHBoxLayout()
        self.lbl_file = QLabel("No file loaded")
        self.lbl_file.setStyleSheet("color: #8B7B8C; font-size: 15px;")
        self.btn_load = QPushButton("Load Log File")
        self.btn_load.setStyleSheet("background-color: #A67DA8; color: #FDF9FF;")
        self.btn_load.setFixedWidth(180)
        self.btn_load.clicked.connect(self.load_file)
        top.addWidget(self.lbl_file)
        top.addStretch()
        top.addWidget(self.btn_load)
        main.addLayout(top)

        self.main_tabs = QTabWidget()
        main.addWidget(self.main_tabs)
        self.main_tabs.addTab(self._build_summary_tab(),  "Flight summary")
        self.main_tabs.addTab(self._build_graphs_tab(),   "Telemetry graphs")
        self.main_tabs.addTab(self._build_anomaly_tab(),  "Anomalies")
        self.main_tabs.addTab(self._build_compare_tab(),  "Compare flights")

        self.lbl_status = QLabel("Load a log file to get started.")
        self.lbl_status.setStyleSheet("color: #8B7B8C; font-size: 14px;")
        main.addWidget(self.lbl_status)

    def _build_summary_tab(self):
        w = QWidget()
        lay = QVBoxLayout(w)
        lay.setSpacing(10)
        lay.setContentsMargins(10, 10, 10, 6)

        summary_box = QGroupBox("Flight Summary")
        sl = QHBoxLayout(summary_box)
        sl.setSpacing(10)
        self.cards = {
            "date":     SummaryCard("Date",         "--", "#6915df"),
            "duration": SummaryCard("Duration",     "--", "#9b59b6"),
            "rows":     SummaryCard("Log Rows",     "--", "#3498db"),
            "max_alt":  SummaryCard("Max Altitude", "--", "#1abc9c"),
            "max_spd":  SummaryCard("Max Speed",    "--", "#e67e22"),
            "min_volt": SummaryCard("Min Voltage",  "--", "#f1c40f"),
            "max_curr": SummaryCard("Max Current",  "--", "#e74c3c"),
            "gps_avg":  SummaryCard("Avg GPS Sats", "--", "#2ecc71"),
        }
        for card in self.cards.values():
            sl.addWidget(card)
        lay.addWidget(summary_box)

        self.table = QTableWidget()
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.setSelectionBehavior(QTableWidget.SelectRows)
        self.table.verticalHeader().setVisible(False)
        self.table.setSortingEnabled(True)
        self.table.setAlternatingRowColors(True)
        self.table.setStyleSheet(
            "QTableWidget { alternate-background-color: #F5EAF6; }"
        )
        lay.addWidget(self.table)
        return w

    def _build_graphs_tab(self):
        w = QWidget()
        lay = QHBoxLayout(w)
        lay.setContentsMargins(10, 10, 10, 10)
        lay.setSpacing(10)

        sidebar = QGroupBox("Columns to Plot")
        sidebar.setFixedWidth(210)
        sb_lay = QVBoxLayout(sidebar)
        sb_lay.setSpacing(4)

        btn_row = QHBoxLayout()
        for label, slot in [("All", self._check_all), ("None", self._check_none)]:
            b = QPushButton(label)
            b.setStyleSheet(
                "background-color: #E8D5E9; color: #2D2D2D; "
                "padding: 4px 10px; font-size: 13px;"
            )
            b.clicked.connect(slot)
            btn_row.addWidget(b)
        sb_lay.addLayout(btn_row)

        sep = QFrame()
        sep.setFrameShape(QFrame.HLine)
        sep.setStyleSheet("color: #C8A2C9;")

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet("QScrollArea { border: none; background: transparent; }")
        cb_container = QWidget()
        self.cb_layout = QVBoxLayout(cb_container)
        self.cb_layout.setSpacing(2)
        self.cb_layout.setContentsMargins(4, 4, 4, 4)
        self.cb_layout.addStretch()
        scroll.setWidget(cb_container)
        sb_lay.addWidget(scroll)

        self.btn_plot = QPushButton("PLOT")
        self.btn_plot.setStyleSheet("background-color: #A67DA8; color: #FDF9FF;")
        self.btn_plot.clicked.connect(self._do_plot)
        sb_lay.addWidget(self.btn_plot)

        lay.addWidget(sidebar)

        right = QWidget()
        rl = QVBoxLayout(right)
        rl.setContentsMargins(0, 0, 0, 0)
        rl.setSpacing(4)
        self.canvas = TelemetryCanvas()
        self.toolbar = NavigationToolbar(self.canvas, right)
        self.toolbar.setStyleSheet("background-color: #E8D5E9; color: #2D2D2D;")
        rl.addWidget(self.toolbar)
        rl.addWidget(self.canvas)
        lay.addWidget(right)

        self.canvas.plot([], [])
        return w

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

        filename = path.replace("\\", "/").split("/")[-1]
        self.lbl_file.setText(f"  {filename}")
        self.lbl_file.setStyleSheet(
            "color: #A67DA8; font-size: 15px; font-weight: bold;"
        )
        self._populate_table()
        self._populate_summary()
        self._populate_checkboxes()
        self._run_anomaly_detection()
        self.lbl_status.setText(
            f"Loaded {len(self._rows)} rows  ·  "
            f"{len(self._columns)} columns  ·  {path}"
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
                raw  = row.get(col, "")
                text = format_cell(col, raw) if col != "timestamp" else raw
                try:
                    item = NumericTableWidgetItem(text, float(raw))
                except (ValueError, TypeError):
                    item = QTableWidgetItem(text)
                item.setTextAlignment(Qt.AlignCenter)
                if col == "arm_status":
                    item.setForeground(
                        QColor("#e74c3c" if text == "ARMED" else "#2ecc71")
                    )
                self.table.setItem(r, c, item)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        if "timestamp" in self._columns:
            self.table.horizontalHeader().setSectionResizeMode(
                self._columns.index("timestamp"), QHeaderView.Stretch
            )
        self.table.setUpdatesEnabled(True)
        self.table.setSortingEnabled(True)

    def _populate_summary(self):
        if not self._rows:
            return
        try:
            def parse_ts(s):
                for fmt in ("%Y-%m-%d %H:%M:%S.%f", "%Y-%m-%d %H:%M:%S"):
                    try: return datetime.strptime(s, fmt)
                    except: pass
                return None
            ts = [parse_ts(r.get("timestamp", "")) for r in self._rows]
            ts = [t for t in ts if t]
            if ts:
                self.cards["date"].set(ts[0].strftime("%Y-%m-%d"))
                secs = int((ts[-1] - ts[0]).total_seconds())
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

        alts  = col_floats("altitude_m")
        spds  = col_floats("groundspeed_ms")
        volts = col_floats("voltage_mV")
        currs = col_floats("current_cA")
        sats  = col_floats("gps_satellites")
        if alts:  self.cards["max_alt"].set(f"{max(alts):.1f} m")
        if spds:  self.cards["max_spd"].set(f"{max(spds):.1f} m/s")
        if volts: self.cards["min_volt"].set(f"{min(volts)/1000:.2f} V")
        if currs: self.cards["max_curr"].set(f"{max(currs)/100:.1f} A")
        if sats:  self.cards["gps_avg"].set(f"{sum(sats)/len(sats):.1f}")

    def _populate_checkboxes(self):
        while self.cb_layout.count() > 1:
            item = self.cb_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()
        self._checkboxes.clear()

        plottable = [c for c in self._columns if c in PLOTTABLE_COLUMNS]
        for i, col in enumerate(plottable):
            name, unit = PLOTTABLE_COLUMNS[col]
            label = f"{name}" + (f"  ({unit})" if unit else "")
            color = PLOT_COLORS[i % len(PLOT_COLORS)]
            cb = QCheckBox(label)
            cb.setChecked(col in DEFAULT_PLOT_COLS)
            cb.setStyleSheet(f"QCheckBox {{ color: {color}; }}")
            self._checkboxes[col] = cb
            self.cb_layout.insertWidget(self.cb_layout.count() - 1, cb)

    def _check_all(self):
        for cb in self._checkboxes.values():
            cb.setChecked(True)

    def _check_none(self):
        for cb in self._checkboxes.values():
            cb.setChecked(False)

    def _do_plot(self):
        if not self._rows:
            self.lbl_status.setText("No data loaded yet.")
            return
        selected = [col for col, cb in self._checkboxes.items() if cb.isChecked()]
        if not selected:
            self.canvas.plot([], [])
            return
        time_labels = [r.get("timestamp", str(i)) for i, r in enumerate(self._rows)]
        series = []
        col_list = list(self._checkboxes.keys())
        for col in selected:
            name, unit = PLOTTABLE_COLUMNS[col]
            color = PLOT_COLORS[col_list.index(col) % len(PLOT_COLORS)]
            values = [to_plot_value(col, r.get(col, "")) for r in self._rows]
            series.append((col, name, unit, values, color))
        self.canvas.plot(time_labels, series)
        self.main_tabs.setCurrentIndex(1)

    def _build_anomaly_tab(self):
        w = QWidget()
        lay = QHBoxLayout(w)
        lay.setContentsMargins(10, 10, 10, 10)
        lay.setSpacing(10)

        sidebar = QGroupBox("Thresholds")
        sidebar.setFixedWidth(220)
        sb = QVBoxLayout(sidebar)
        sb.setSpacing(6)

        def thresh_row(label, default):
            row = QHBoxLayout()
            lbl = QLabel(label)
            lbl.setStyleSheet("color: #8B7B8C; font-size: 13px;")
            lbl.setFixedWidth(140)
            inp = QLineEdit(default)
            inp.setFixedWidth(55)
            inp.setStyleSheet(
                "background:#FFFFFF; color:#2D2D2D; "
                "padding:3px; border:1px solid #C8A2C9; border-radius:3px;"
            )
            row.addWidget(lbl)
            row.addWidget(inp)
            sb.addLayout(row)
            return inp

        sb.addWidget(QLabel("Voltage"))
        self.t_volt_warn = thresh_row("Warning below (V):",  "14.0")
        self.t_volt_crit = thresh_row("Critical below (V):", "13.0")
        sb.addWidget(self._sep())

        sb.addWidget(QLabel("Vibration"))
        self.t_vib_warn = thresh_row("Warning above:",  "0.3")
        self.t_vib_crit = thresh_row("Critical above:", "0.6")
        sb.addWidget(self._sep())

        sb.addWidget(QLabel("GPS Satellites"))
        self.t_gps_warn = thresh_row("Warning below:", "6")
        self.t_gps_crit = thresh_row("Critical below:", "4")
        sb.addWidget(self._sep())

        sb.addWidget(QLabel("Altitude"))
        self.t_alt_jump = thresh_row("Jump warning (m):", "8.0")
        sb.addWidget(self._sep())

        sb.addWidget(QLabel("Current"))
        self.t_curr_warn = thresh_row("Warning above (A):", "40.0")

        sb.addStretch()
        btn_detect = QPushButton("Run detection")
        btn_detect.setStyleSheet("background-color: #A67DA8; color: #FDF9FF;")
        btn_detect.clicked.connect(self._run_anomaly_detection)
        sb.addWidget(btn_detect)
        lay.addWidget(sidebar)

        right = QWidget()
        rl = QVBoxLayout(right)
        rl.setContentsMargins(0, 0, 0, 0)
        rl.setSpacing(8)

        badge_row = QHBoxLayout()
        self.badge_crit = self._badge("Critical: --",    "#e74c3c")
        self.badge_warn = self._badge("Warnings: --",    "#f39c12")
        self.badge_info = self._badge("Info events: --", "#3498db")
        badge_row.addWidget(self.badge_crit)
        badge_row.addWidget(self.badge_warn)
        badge_row.addWidget(self.badge_info)
        badge_row.addStretch()
        rl.addLayout(badge_row)

        self.anomaly_table = QTableWidget()
        self.anomaly_table.setColumnCount(4)
        self.anomaly_table.setHorizontalHeaderLabels(
            ["Severity", "Timestamp", "Type", "Details"]
        )
        self.anomaly_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self.anomaly_table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeToContents)
        self.anomaly_table.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeToContents)
        self.anomaly_table.horizontalHeader().setSectionResizeMode(3, QHeaderView.Stretch)
        self.anomaly_table.verticalHeader().setVisible(False)
        self.anomaly_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.anomaly_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.anomaly_table.setSortingEnabled(True)
        self.anomaly_table.setAlternatingRowColors(True)
        self.anomaly_table.setStyleSheet(
            "QTableWidget { alternate-background-color: #F5EAF6; }"
        )
        rl.addWidget(self.anomaly_table)
        lay.addWidget(right)
        return w

    def _sep(self):
        sep = QFrame()
        sep.setFrameShape(QFrame.HLine)
        sep.setStyleSheet("color: #C8A2C9; margin: 2px 0;")
        return sep

    def _badge(self, text, color):
        lbl = QLabel(text)
        lbl.setStyleSheet(f"""
            background-color: #FDF9FF; border: 2px solid {color};
            border-radius: 6px; color: {color};
            font-weight: bold; font-size: 15px; padding: 6px 16px;
        """)
        return lbl

    def _run_anomaly_detection(self):
        if not self._rows:
            return
        try:
            v_warn  = float(self.t_volt_warn.text())
            v_crit  = float(self.t_volt_crit.text())
            vib_w   = float(self.t_vib_warn.text())
            vib_c   = float(self.t_vib_crit.text())
            gps_w   = int(float(self.t_gps_warn.text()))
            gps_c   = int(float(self.t_gps_crit.text()))
            alt_j   = float(self.t_alt_jump.text())
            curr_w  = float(self.t_curr_warn.text())
        except ValueError:
            return

        anomalies = []
        prev_mode = None
        prev_arm  = None
        prev_alt  = None
        SEV_COLORS = {"CRITICAL": "#e74c3c", "WARNING": "#f39c12", "INFO": "#3498db"}

        def add(sev, ts, typ, detail):
            anomalies.append((sev, ts, typ, detail))

        for row in self._rows:
            ts = row.get("timestamp", "")

            try:
                v = float(row["voltage_mV"]) / 1000
                if v > 0:
                    if v < v_crit:
                        add("CRITICAL", ts, "Low Voltage", f"{v:.2f} V  (critical ≤ {v_crit} V)")
                    elif v < v_warn:
                        add("WARNING",  ts, "Low Voltage", f"{v:.2f} V  (warning ≤ {v_warn} V)")
            except Exception: pass

            for axis in ("vib_x", "vib_y", "vib_z"):
                try:
                    vib = abs(float(row[axis]))
                    ax  = axis[-1].upper()
                    if vib > vib_c:
                        add("CRITICAL", ts, f"High Vibration {ax}", f"{vib:.2e}  (critical > {vib_c})")
                    elif vib > vib_w:
                        add("WARNING",  ts, f"High Vibration {ax}", f"{vib:.2e}  (warning > {vib_w})")
                except Exception: pass

            try:
                sats = int(float(row["gps_satellites"]))
                if sats < gps_c:
                    add("CRITICAL", ts, "Low GPS Satellites", f"{sats} sats  (critical < {gps_c})")
                elif sats < gps_w:
                    add("WARNING",  ts, "Low GPS Satellites", f"{sats} sats  (warning < {gps_w})")
            except Exception: pass

            try:
                alt = float(row["altitude_m"])
                if prev_alt is not None and abs(alt - prev_alt) > alt_j:
                    add("WARNING", ts, "Sudden Altitude Change",
                        f"Δ {abs(alt - prev_alt):.1f} m in one log step")
                prev_alt = alt
            except Exception: pass

            try:
                curr = float(row["current_cA"]) / 100
                if curr > curr_w:
                    add("WARNING", ts, "High Current Draw", f"{curr:.1f} A  (warning > {curr_w} A)")
            except Exception: pass

            try:
                mode = decode_px4_mode(row.get("flight_mode", ""))
                if mode not in ("--", "") and prev_mode is not None and mode != prev_mode:
                    add("INFO", ts, "Flight Mode Change", f"{prev_mode}  →  {mode}")
                if mode not in ("--", ""):
                    prev_mode = mode
            except Exception: pass

            try:
                arm = "ARMED" if (int(float(row.get("arm_status", "0"))) & 128) else "DISARMED"
                if prev_arm is not None and arm != prev_arm:
                    add("INFO", ts, "Arm Status Change", f"{prev_arm}  →  {arm}")
                prev_arm = arm
            except Exception: pass

        self.anomaly_table.setSortingEnabled(False)
        self.anomaly_table.setUpdatesEnabled(False)
        self.anomaly_table.setRowCount(len(anomalies))
        counts = {"CRITICAL": 0, "WARNING": 0, "INFO": 0}
        for r, (sev, ts, typ, detail) in enumerate(anomalies):
            counts[sev] = counts.get(sev, 0) + 1
            color = SEV_COLORS.get(sev, "#ecf0f1")
            for c, text in enumerate([sev, ts, typ, detail]):
                item = QTableWidgetItem(text)
                item.setForeground(QColor(color))
                item.setTextAlignment(Qt.AlignVCenter | Qt.AlignLeft)
                self.anomaly_table.setItem(r, c, item)
        self.anomaly_table.setUpdatesEnabled(True)
        self.anomaly_table.setSortingEnabled(True)

        self.badge_crit.setText(f"Critical: {counts['CRITICAL']}")
        self.badge_warn.setText(f"Warnings: {counts['WARNING']}")
        self.badge_info.setText(f"Info events: {counts['INFO']}")

    def _build_compare_tab(self):
        w = QWidget()
        lay = QHBoxLayout(w)
        lay.setContentsMargins(10, 10, 10, 10)
        lay.setSpacing(10)

        sidebar = QGroupBox("Flights to Compare")
        sidebar.setFixedWidth(320)
        sb = QVBoxLayout(sidebar)
        sb.setSpacing(6)

        self.slots_scroll = QScrollArea()
        self.slots_scroll.setWidgetResizable(True)
        self.slots_scroll.setStyleSheet(
            "QScrollArea { border: none; background: transparent; }"
        )
        self.slots_scroll.setMinimumHeight(175)
        slots_container = QWidget()
        self.slots_layout = QVBoxLayout(slots_container)
        self.slots_layout.setSpacing(4)
        self.slots_layout.setContentsMargins(0, 0, 0, 0)
        self.slots_layout.addStretch()
        self.slots_scroll.setWidget(slots_container)
        sb.addWidget(self.slots_scroll)

        btn_add = QPushButton("＋  Add Flight")
        btn_add.setStyleSheet(
            "background-color: #E8D5E9; color: #2D2D2D; "
            "padding: 5px; font-size: 14px;"
        )
        btn_add.clicked.connect(self._add_compare_slot)
        sb.addWidget(btn_add)

        sb.addWidget(self._sep())

        self.cb_align = QCheckBox("Align logs to t = 0")
        self.cb_align.setChecked(True)
        sb.addWidget(self.cb_align)

        sb.addWidget(self._sep())

        sb.addWidget(self._compare_section_label("Columns"))
        scroll2 = QScrollArea()
        scroll2.setWidgetResizable(True)
        scroll2.setStyleSheet(
            "QScrollArea { border: none; background: transparent; }"
        )
        cb_container = QWidget()
        self.compare_cb_layout = QVBoxLayout(cb_container)
        self.compare_cb_layout.setSpacing(2)
        self.compare_cb_layout.setContentsMargins(2, 2, 2, 2)
        for i, (col, (name, unit)) in enumerate(PLOTTABLE_COLUMNS.items()):
            label = f"{name}" + (f"  ({unit})" if unit else "")
            color = PLOT_COLORS[i % len(PLOT_COLORS)]
            cb = QCheckBox(label)
            cb.setChecked(col in DEFAULT_PLOT_COLS)
            cb.setStyleSheet(f"QCheckBox {{ color: {color}; }}")
            self._compare_cbs[col] = cb
            self.compare_cb_layout.addWidget(cb)
        self.compare_cb_layout.addStretch()
        scroll2.setWidget(cb_container)
        sb.addWidget(scroll2)

        btn_plot = QPushButton("▶  Compare")
        btn_plot.setStyleSheet("background-color: #A67DA8; color: #FDF9FF;")
        btn_plot.clicked.connect(self._do_compare_plot)
        sb.addWidget(btn_plot)

        lay.addWidget(sidebar)

        right = QWidget()
        rl = QVBoxLayout(right)
        rl.setContentsMargins(0, 0, 0, 0)
        rl.setSpacing(4)
        self.compare_canvas = CompareCanvas()
        compare_toolbar = NavigationToolbar(self.compare_canvas, right)
        compare_toolbar.setStyleSheet(
            "background-color: #E8D5E9; color: #2D2D2D;"
        )
        rl.addWidget(compare_toolbar)
        rl.addWidget(self.compare_canvas)
        lay.addWidget(right)

        self._refresh_slots_ui()
        self.compare_canvas.plot([])
        return w

    def _compare_section_label(self, text):
        lbl = QLabel(text)
        lbl.setStyleSheet(
            "color: #2D2D2D; font-weight: bold; font-size: 14px;"
        )
        return lbl

    def _refresh_slots_ui(self):
        while self.slots_layout.count() > 1:
            item = self.slots_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()

        for i, flight in enumerate(self._compare_flights):
            color = FLIGHT_COLORS[i % len(FLIGHT_COLORS)]
            slot = QWidget()
            slot.setFixedHeight(30)
            row = QHBoxLayout(slot)
            row.setContentsMargins(0, 2, 0, 2)
            row.setSpacing(4)

            dot = QLabel("⬤")
            dot.setStyleSheet(f"color: {color}; font-size: 16px;")
            dot.setFixedWidth(18)

            if flight["name"]:
                # btn_load = QPushButton(flight["name"])
                if flight["name"]:
                    full_name = flight["name"]
                    display_name = (full_name[:25] + "...") if len(full_name) > 25 else full_name
                    btn_load = QPushButton(display_name)
                    btn_load.setToolTip(full_name)
                btn_load.setStyleSheet(
                    f"background-color: {color}; color: #FDF9FF; "
                    f"padding: 4px 6px; font-size: 12px; text-align: left;"
                )
            else:
                btn_load = QPushButton(f"Flight {i + 1}: Load file")
                btn_load.setStyleSheet(
                    f"background-color: #E8D5E9; color: #2D2D2D; "
                    f"padding: 4px 6px; font-size: 12px;"
                )
            btn_load.clicked.connect(
                lambda _, idx=i: self._load_compare_slot(idx)
            )

            btn_x = QPushButton("×")
            btn_x.setFixedSize(22, 22)
            btn_x.setStyleSheet(
                "background-color: #E8D5E9; color: #8B7B8C; "
                "padding: 0; font-size: 16px; font-weight: bold;"
            )
            btn_x.clicked.connect(
                lambda _, idx=i: self._remove_compare_slot(idx)
            )

            row.addWidget(dot)
            row.addWidget(btn_load, stretch=1)
            row.addWidget(btn_x)
            self.slots_layout.insertWidget(
                self.slots_layout.count() - 1, slot
            )

    def _add_compare_slot(self):
        self._compare_flights.append({"rows": [], "name": None})
        self._refresh_slots_ui()

    def _remove_compare_slot(self, index):
        if len(self._compare_flights) <= 1:
            return
        self._compare_flights.pop(index)
        self._refresh_slots_ui()

    def _load_compare_slot(self, index):
        path, _ = QFileDialog.getOpenFileName(
            self, "Open Flight Log", "", "CSV Files (*.csv);;All Files (*)"
        )
        if not path:
            return
        try:
            with open(path, newline="", encoding="utf-8") as f:
                rows = list(csv.DictReader(f))
        except Exception as e:
            self.lbl_status.setText(f"Error: {e}")
            return
        filename = path.replace("\\", "/").split("/")[-1]
        self._compare_flights[index]["rows"] = rows
        self._compare_flights[index]["name"] = filename
        self._refresh_slots_ui()
        self.lbl_status.setText(
            f"Loaded {filename} as Flight {index + 1}"
        )

    def _do_compare_plot(self):
        selected = [col for col, cb in self._compare_cbs.items()
                    if cb.isChecked()]
        if not selected:
            return

        align = self.cb_align.isChecked()

        def parse_ts(s):
            for fmt in ("%Y-%m-%d %H:%M:%S.%f", "%Y-%m-%d %H:%M:%S"):
                try: return datetime.strptime(s, fmt)
                except: pass
            return None

        def build_series(rows, selected):
            if not rows:
                return []
            timestamps = [parse_ts(r.get("timestamp", "")) for r in rows]
            t0 = next((t for t in timestamps if t), None)
            xs_base = [
                (t - t0).total_seconds() if (align and t0 and t) else j
                for j, t in enumerate(timestamps)
            ]
            series = []
            for col in selected:
                name, unit = PLOTTABLE_COLUMNS[col]
                xs, ys = [], []
                for j, row in enumerate(rows):
                    v = to_plot_value(col, row.get(col, ""))
                    if v is not None:
                        xs.append(xs_base[j])
                        ys.append(v)
                if xs:
                    series.append((col, name, unit, xs, ys))
            return series

        flights_data = []
        for i, flight in enumerate(self._compare_flights):
            if not flight["rows"]:
                continue
            series = build_series(flight["rows"], selected)
            if series:
                flights_data.append({
                    "name":   flight["name"],
                    "color":  FLIGHT_COLORS[i % len(FLIGHT_COLORS)],
                    "series": series,
                })

        if not flights_data:
            self.lbl_status.setText("No loaded flights to compare.")
            return

        self.compare_canvas.plot(flights_data)
        self.main_tabs.setCurrentIndex(3)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    win = LogViewer()
    win.show()
    sys.exit(app.exec_())
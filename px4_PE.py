import sys
import time
import math

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QLineEdit, QComboBox, QTableWidget,
    QTableWidgetItem, QHeaderView, QProgressBar, QFileDialog,
    QMessageBox
)

from PyQt5.QtCore import Qt
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QColor

try:
    from pymavlink import mavutil
except ImportError:
    print("mavlink not found - install w/ pip install pymavlink")
    sys.exit(1)

# const
MAV_PARAM_TYPES = {
    1: "uint8", 2: "int8",
    3: "uint16", 4: "int16",
    5: "uint32", 6: "int32",
    7: "uint64", 8: "int64",
    9: "float", 10: "double"
}

COL_NAME = 0
COL_VALUE = 1
COL_TYPE = 2
COL_GROUP = 3

STYLE = """
    QMainWindow, QWidget {
        background-color: #0a0a0a;
        color: #ecf0f1;
        font-family: Consolas, Arial;
    }
    QGroupBox {
        border: 2px solid #2c3e50; border-radius: 6px;
        margin-top: 10px; font-weight: bold;
        padding: 6px; color: #ecf0f1;
    }
    QGroupBox::title { padding: 0 4px; }
    QPushButton {
        padding: 8px 16px; font-weight: bold;
        font-size: 12px; border-radius: 5px; border: none;
    }
    QLineEdit {
        background-color: #2c3e50; color: white;
        padding: 6px 10px; font-size: 12px;
        border: 1px solid #3d5166; border-radius: 4px;
    }
    QComboBox {
        background-color: #2c3e50; color: white;
        padding: 6px; font-size: 12px;
        border: none; border-radius: 3px;
    }
    QComboBox::drop-down { border: none; }
    QTableWidget {
        background-color: #0d0d0d; color: #ecf0f1;
        font-family: Consolas, monospace; font-size: 12px;
        border: 1px solid #2c3e50; gridline-color: #1a1a1a;
    }
    QTableWidget::item:selected { background-color: #2c3e50; }
    QHeaderView::section {
        background-color: #1a2535; color: #ecf0f1;
        padding: 6px; border: none;
        font-weight: bold; font-size: 12px;
    }
    QProgressBar {
        border: 1px solid #2c3e50; border-radius: 4px;
        background-color: #1a1a1a; text-align: center; color: #ecf0f1;
    }
    QProgressBar::chunk { background-color: #3498db; border-radius: 3px; }
    QScrollBar:vertical { background: #1a1a1a; width: 10px; border-radius: 5px; }
    QScrollBar::handle:vertical { background: #2c3e50; border-radius: 5px; }
"""

# MAIN WINDOW
class ParameterEditor(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("px4_param_editor")
        self.setMinimumSize(1000, 650)
        self.resize(1200, 760)
        self.setStyleSheet(STYLE)

        # connecting
        self.master = None
        self.is_connected = False
        self._waiting_heartbeat = False
        self._heartbeat_deadline = 0

        # param store: name => {value, ptype, original, changed, index}
        self.params= {}
        self.param_count_total = 0
        self.received_indices = set()

        # timers
        self.recv_timer = QTimer()
        self.recv_timer.timeout.connect(self._recv_tick)

        self.retry_timer = QTimer()
        self.retry_timer.setSingleShot(True)
        self.retry_timer.timeout.connect(self._retry_missing)

        self._setup_ui()

    
    # UI stuff
    def _setup_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        main = QVBoxLayout(root)
        main.setSpacing(8)
        main.setContentsMargins(12, 12, 12, 8)

        # topbar
        top = QHBoxLayout()
        top.setSpacing(8)

        self.cmb_mode = QComboBox()
        self.cmb_mode.addItems(["UDP (SITL)", "Serial"])
        self.cmb_mode.setFixedWidth(120)
        self.cmb_mode.currentTextChanged.connect(self._on_mode_changed)

        self.lbl_port = QLabel("Port:")
        self.lbl_port.setStyleSheet("color: #95a5a6;")
        self.input_port = QLineEdit("14550")
        self.input_port.setFixedWidth(90)

        self.lbl_baud = QLabel("Baud:")
        self.lbl_baud.setStyleSheet("color: #95a5a6;")
        self.lbl_baud.setVisible(False)
        self.cmb_baud = QComboBox()
        self.cmb_baud.addItems(["57600", "115200", "921600"])
        self.cmb_baud.setFixedWidth(90)
        self.cmb_baud.setVisible(False)

        self.btn_connect = QPushButton("Connect")
        self.btn_connect.setFixedWidth(120)
        self.btn_connect.setStyleSheet("background-color: #27ae60; color: white;")
        self.btn_connect.clicked.connect(self.toggle_connection)

        self.lbl_conn = QLabel("Not connected")
        self.lbl_conn.setStyleSheet("color: #7f8c8d; font-size: 12px;")
 
        top.addWidget(self.cmb_mode)
        top.addWidget(self.lbl_port)
        top.addWidget(self.input_port)
        top.addWidget(self.lbl_baud)
        top.addWidget(self.cmb_baud)
        top.addWidget(self.btn_connect)
        top.addWidget(self.lbl_conn)
        top.addStretch()
 
        self.btn_write = QPushButton("Write Changed  (0)")
        self.btn_write.setStyleSheet("background-color: #e67e22; color: white;")
        self.btn_write.setEnabled(False)
        self.btn_write.clicked.connect(self._write_all_changed)
 
        btn_export = QPushButton("Export .params")
        btn_export.setStyleSheet("background-color: #2c3e50; color: white;")
        btn_export.clicked.connect(self.export_params)
 
        btn_import = QPushButton("Import .params")
        btn_import.setStyleSheet("background-color: #2c3e50; color: white;")
        btn_import.clicked.connect(self.import_params)
 
        top.addWidget(self.btn_write)
        top.addWidget(btn_export)
        top.addWidget(btn_import)
        main.addLayout(top)

        # progressbar
        self.progress = QProgressBar()
        self.progress.setFixedHeight(18)
        self.progress.setVisible(False)
        main.addWidget(self.progress)

        # filter
        frow = QHBoxLayout()
        frow.setSpacing(8)

        lbl_s = QLabel("Search:")
        lbl_s.setStyleSheet("color: #95a5a6;")
        self.search = QLineEdit()
        self.search.setPlaceholderText("Filter by parameter name…")
        self.search.textChanged.connect(self._apply_filter)
 
        lbl_g = QLabel("Group:")
        lbl_g.setStyleSheet("color: #95a5a6;")
        self.cmb_group = QComboBox()
        self.cmb_group.setFixedWidth(150)
        self.cmb_group.addItem("All groups")
        self.cmb_group.currentTextChanged.connect(self._apply_filter)
 
        lbl_c = QLabel("Show:")
        lbl_c.setStyleSheet("color: #95a5a6;")
        self.cmb_show = QComboBox()
        self.cmb_show.addItems(["All parameters", "Changed only"])
        self.cmb_show.setFixedWidth(160)
        self.cmb_show.currentTextChanged.connect(self._apply_filter)
 
        frow.addWidget(lbl_s)
        frow.addWidget(self.search, stretch=1)
        frow.addWidget(lbl_g)
        frow.addWidget(self.cmb_group)
        frow.addWidget(lbl_c)
        frow.addWidget(self.cmb_show)
        main.addLayout(frow)

        # table
        self.table = QTableWidget()
        self.table.setColumnCount(4)
        self.table.setHorizontalHeaderLabels(["Parameter", "Value", "Type", "Group"])
        self.table.horizontalHeader().setSectionResizeMode(COL_NAME,  QHeaderView.Stretch)
        self.table.horizontalHeader().setSectionResizeMode(COL_VALUE, QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(COL_TYPE,  QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(COL_GROUP, QHeaderView.ResizeToContents)
        self.table.verticalHeader().setVisible(False)
        self.table.setSelectionBehavior(QTableWidget.SelectRows)
        self.table.setAlternatingRowColors(True)
        self.table.setStyleSheet(
            "QTableWidget { alternate-background-color: #0f0f0f; }"
        )
        self.table.setSortingEnabled(True)
        self.table.itemChanged.connect(self._on_item_changed)
        main.addWidget(self.table)

        # statusbar
        self.lbl_status = QLabel("Connect to a vehicle to load parameters.")
        self.lbl_status.setStyleSheet("color: #7f8c8d; font-size: 11px;")
        main.addWidget(self.lbl_status)
    
    def _on_mode_changed(self, mode):
        serial = (mode == "Serial")
        self.lbl_baud.setVisible(serial)
        self.cmb_baud.setVisible(serial)

        if serial:
            self.lbl_port.setText("Device:")

            if not self.input_port.text().startswith("/dev/") and \
            not self.input_port.text().upper().startswith("COM"):
                self.input_port.setText("/dev/ttyUSB0")
            
        else:
            self.lbl_port.setText("Port:")
            if self.input_port.text().startswith("/dev/") or \
            self.input_port.text().upper().startswith("COM"):
                self.input_port.setText("14550")
    
    # CONNECTION STUFF
    def toggle_connection(self):
        if self.is_connected:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self.input_port.text().strip()
        mode = self.cmb_mode.currentText()

        try:
            if mode == "Serial":
                baud = int(self.cmb_baud.currentText())
                self.master = mavutil.mavlink_connection(port, baud=baud)
            else:
                self.master = mavutil.mavlink_connection(
                    f"udpin:0.0.0.0:{port}"
                )

        except Exception as e:
            self.lbl_conn.setText(f"Error: {e}")
            self.lbl_conn.setStyleSheet("color: #e74c3c; font-size: 12px;")
            return

        self.is_connected = True
        self.btn_connect.setText("Disconnect")
        self.btn_connect.setStyleSheet("background-color: #e74c3c; color: white;")
        self.lbl_conn.setText("Waiting for heartbeat…")
        self.lbl_conn.setStyleSheet("color: #f39c12; font-size: 12px;")

        self._waiting_heartbeat  = True
        self._heartbeat_deadline = time.time() + 10
        self.recv_timer.start(50)
    
    def _disconnect(self):
        self.recv_timer.stop()
        self.retry_timer.stop()

        if self.master:
            try:
                self.master.close()

            except Exception:
                pass

            self.master = None

        self.is_connected = False
        self.btn_connect.setText("Connect")
        self.btn_connect.setStyleSheet("background-color: #27ae60; color: white;")
        self.lbl_conn.setText("Disconnected")
        self.lbl_conn.setStyleSheet("color: #7f8c8d; font-size: 12px;")
        self.progress.setVisible(False)

    def _start_load(self):
        self.params = {}
        self.received_indices = set()
        self.param_count_total = 0
        self._waiting_heartbeat = False

        self.table.setRowCount(0)
        self.progress.setValue(0)
        self.progress.setVisible(True)
        self.lbl_conn.setText("requesting parameters...")
        self.lbl_conn.setStyleSheet("color: #f39c12; font-size: 12px;")

        self.master.mav.param_request_list_send(
            self.master.target_system,
            self.master.target_component
        )

    def _recv_tick(self):
        if not self.master:
            return

        # waiting for first heartbeat
        if self._waiting_heartbeat:
            try:
                msg = self.master.recv_match(type="HEARTBEAT", blocking=False)
            except (ConnectionResetError, OSError):
                return
            if msg:
                self.master.target_system    = msg.get_srcSystem()
                self.master.target_component = msg.get_srcComponent()
                self._start_load()
            elif time.time() > self._heartbeat_deadline:
                self.lbl_conn.setText("No heartbeat — is PX4 running?")
                self.lbl_conn.setStyleSheet("color: #e74c3c; font-size: 12px;")
                self._waiting_heartbeat = False
                self.recv_timer.stop()
            return

        # draining PARAM_VALUE msgs
        for _ in range(100):
            try:
                msg = self.master.recv_match(type="PARAM_VALUE", blocking=False)
            except (ConnectionResetError, OSError):
                break
            if not msg:
                break

            name  = msg.param_id.strip("\x00")
            index = msg.param_index
            count = msg.param_count

            if count > 0 and self.param_count_total == 0:
                self.param_count_total = count
                self.progress.setMaximum(count)

            if name not in self.params:
                self.params[name] = {
                    "value": msg.param_value,
                    "ptype": msg.param_type,
                    "original": msg.param_value,
                    "changed": False,
                    "index": index,
                }
            self.received_indices.add(index)

            received = len(self.received_indices)
            self.progress.setValue(received)
            self.lbl_conn.setText(
                f"Loading...  {received} / {self.param_count_total}"
            )

        # check if done
        if (self.param_count_total > 0 and len(self.received_indices) >= self.param_count_total):
            self.recv_timer.stop()
            self._finish_load()
        else:
            # retry for missed packages (if theres any)
            self.retry_timer.start(4000)

    def _retry_missing(self):
        if not self.is_connected or not self.master:
            return
        
        missing = [i for i in range(self.param_count_total) if i not in self.received_indices]
        if not missing:
            self._finish_load()
            return
        
        self.lbl_conn.setText(
            f"AGAIN requesting {len(missing)} missing params.."
        )

        for idx in missing:
            self.master.mav.param_request_read_send(
                self.master.target_system,
                self.master.target_component,
                b"", idx
            )
        
        self.recv_timer.start(50)
        self.retry_timer.start(5000)
    
    def _finish_load(self):
        self.retry_timer.stop()
        self.recv_timer.stop()
        self.progress.setVisible(False)

        # group filter populating
        groups = sorted(set(n.split("_")[0] for n in self.params))
        self.cmb_group.blockSignals(True)
        self.cmb_group.clear()
        self.cmb_group.addItem("All groups")

        for g in groups:
            self.cmb_group.addItem(g)
        self.cmb_group.blockSignals(False)

        self._apply_filter()

        n = len(self.params)
        self.lbl_conn.setText(f"{n} params loaded")
        self.lbl_conn.setStyleSheet("color: #2ecc71; font-size: 12px;")
        self.lbl_status.setText(
            f"{n} params loaded | "
            "double click values to edit | "
            "click 'Write Changed' to send edits to vehicle"
        )

    # table
    def _apply_filter(self):
        search       = self.search.text().lower()
        group        = self.cmb_group.currentText()
        changed_only = (self.cmb_show.currentText() == "Changed only")

        names = []
        for name in sorted(self.params.keys()):
            p = self.params[name]
            if search and search not in name.lower():
                continue
            if group != "All groups" and not name.startswith(group + "_"):
                continue
            if changed_only and not p["changed"]:
                continue
            names.append(name)

        self._fill_table(names)
    
    def _fill_table(self, names):
        self.table.itemChanged.disconnect(self._on_item_changed)
        self.table.setSortingEnabled(False)
        self.table.setUpdatesEnabled(False)
        self.table.setRowCount(len(names))

        for r, name in enumerate(names):
            p = self.params[name]
            is_float  = (p["ptype"] == 9)
            if math.isnan(p["value"]):
                val_str = "NaN"
            elif is_float:
                val_str = f"{p['value']:.6g}"
            else:
                val_str = str(int(p["value"]))
            type_str  = MAV_PARAM_TYPES.get(p["ptype"], str(p["ptype"]))
            group_str = name.split("_")[0]

            row_data = [name, val_str, type_str, group_str]
            for c, text in enumerate(row_data):
                item = QTableWidgetItem(text)
                item.setTextAlignment(
                    Qt.AlignVCenter |
                    (Qt.AlignLeft if c == COL_NAME else Qt.AlignCenter)
                )
                if c != COL_VALUE:
                    item.setFlags(item.flags() & ~Qt.ItemIsEditable)
                if p["changed"]:
                    item.setBackground(QColor("#3d3000"))
                self.table.setItem(r, c, item)

        self.table.setUpdatesEnabled(True)
        self.table.setSortingEnabled(True)
        self.table.itemChanged.connect(self._on_item_changed)

    def _on_item_changed(self, item):
        if item.column() != COL_VALUE:
            return
        name_item = self.table.item(item.row(), COL_NAME)
        if not name_item:
            return
        name = name_item.text()
        if name not in self.params:
            return
 
        p = self.params[name]
 
        # validation + parsing
        try:
            new_val = float(item.text()) if p["ptype"] == 9 \
                      else float(int(item.text()))
        except ValueError:
            # restoring original display
            self.table.itemChanged.disconnect(self._on_item_changed)
            item.setText(
                f"{p['value']:.6g}" if p["ptype"] == 9
                else str(int(p["value"]))
            )
            self.table.itemChanged.connect(self._on_item_changed)
            return
 
        p["value"]   = new_val
        p["changed"] = (abs(new_val - p["original"]) > 1e-9)
 
        changed_bg = QColor("#3d3000") # horrendous colors only
        normal_bg  = QColor("#0d0d0d")
        for c in range(4):
            cell = self.table.item(item.row(), c)
            if cell:
                cell.setBackground(changed_bg if p["changed"] else normal_bg)
 
        self._update_write_btn()
    
    def _update_write_btn(self):
        n = sum(1 for p in self.params.values() if p["changed"])
        self.btn_write.setText(f"Write Changed  ({n})")
        self.btn_write.setEnabled(n > 0)
    
    # writing
    def _write_all_changed(self):
        if not self.is_connected or not self.master:
            QMessageBox.warning(self, "Not connected", "Connect to a vehicle first!")
            return
 
        changed = {n: p for n, p in self.params.items() if p["changed"]}
        if not changed:
            return
 
        preview = "\n".join(
            f"  {n}  =  {p['value']}"
            for n, p in list(changed.items())[:12]
        )
        if len(changed) > 12:
            preview += f"\n  … and {len(changed) - 12} more"
 
        reply = QMessageBox.question(
            self, "Write parameters",
            f"Write {len(changed)} parameter(s) to the vehicle?\n\n{preview}",
            QMessageBox.Yes | QMessageBox.No
        )
        if reply != QMessageBox.Yes:
            return
 
        # send PARAM_SET for each
        for name, p in changed.items():
            self.master.mav.param_set_send(
                self.master.target_system,
                self.master.target_component,
                name.encode("utf-8"),
                p["value"],
                p["ptype"]
            )
 
        # collect confirmations (----no blocking to UI updates)
        confirmed = set()
        deadline  = time.time() + 6
        while time.time() < deadline and len(confirmed) < len(changed):
            try:
                msg = self.master.recv_match(type="PARAM_VALUE", blocking=False)
            except (ConnectionResetError, OSError):
                msg = None
            if msg:
                n = msg.param_id.strip("\x00")
                if n in changed:
                    self.params[n]["value"]    = msg.param_value
                    self.params[n]["original"] = msg.param_value
                    self.params[n]["changed"]  = False
                    confirmed.add(n)
            QApplication.processEvents()
            time.sleep(0.005)
 
        self._apply_filter()
        self._update_write_btn()
 
        not_confirmed = len(changed) - len(confirmed)
        msg_text = f"Wrote {len(confirmed)} / {len(changed)} parameters."
        if not_confirmed:
            msg_text += f"  {not_confirmed} not confirmed — try again."
        self.lbl_status.setText(msg_text)
    
    # EXPORT / INPORT
    def export_params(self):
        if not self.params:
            QMessageBox.information(self, "No parameters", "Load parameters first!")
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "Export Parameters", "px4_params.params",
            "Params Files (*.params);;All Files (*)"
        )
        if not path:
            return
        try:
            with open(path, "w") as f:
                f.write("# px4 parameter file\n")
                f.write("# Parameter\tValue\tType\n")
                for name in sorted(self.params.keys()):
                    p = self.params[name]
                    f.write(f"{name}\t{p['value']}\t{p['ptype']}\n")
            self.lbl_status.setText(
                f"exported {len(self.params)} parameters to {path}"
            )
        except Exception as e:
            QMessageBox.warning(self, "EXPORT ERROR:", str(e))
    

    def import_params(self):
        if not self.is_connected:
            QMessageBox.warning(self, "Not connected", "Connect to a vehicle before importing!")
            return
        if not self.params:
            QMessageBox.warning(self, "No parameters", "Load parameters from the vehicle first")
            return
 
        path, _ = QFileDialog.getOpenFileName(
            self, "Import Parameters", "",
            "Params Files (*.params);;All Files (*)"
        )
        if not path:
            return
 
        try:
            to_import = {}
            with open(path, "r") as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith("#"):
                        continue

                    parts = line.split("\t")
                    if len(parts) >= 2:
                        name  = parts[0].strip()
                        value = float(parts[1].strip())
                        ptype = int(parts[2].strip()) if len(parts) >= 3 else 9
                        to_import[name] = (value, ptype)
    
        except Exception as e:
            QMessageBox.warning(self, "IMPORT ERROR:", str(e))
            return
 
        applied = 0
        for name, (value, ptype) in to_import.items():
            if name in self.params:
                self.params[name]["value"]   = value
                self.params[name]["changed"] = (
                    abs(value - self.params[name]["original"]) > 1e-9
                )
                applied += 1
 
        self._apply_filter()
        self._update_write_btn()
        self.lbl_status.setText(
            f"Imported {applied} / {len(to_import)} parameters from file."
            "Review the highlighted changes before clicking 'Write Changed'!"
        )

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    win = ParameterEditor()
    win.show()
    sys.exit(app.exec_())
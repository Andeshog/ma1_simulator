import sys
import os
import signal
import subprocess
import threading
import math
import pathlib
from typing import Any, Dict

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy

from PyQt6.QtWidgets import (
    QApplication,
    QWidget,
    QPushButton,
    QComboBox,
    QHBoxLayout,
    QVBoxLayout,
    QFrame,
    QTextEdit,
    QTabWidget,
    QFormLayout,
    QLineEdit,
    QMessageBox,
    QSizePolicy,
    QScrollArea,
)
from PyQt6.QtCore import Qt, pyqtSignal, QTimer, QProcess
from PyQt6.QtGui import QPainter, QPen, QBrush, QColor, QFontMetrics, QPalette
import pyqtgraph as pg

from ruamel.yaml import YAML

class ParameterEditor(QWidget):
    """A minimal YAML parameter editor for ROS 2 config files."""

    def __init__(self, yaml_path: str | pathlib.Path, parent: QWidget | None = None):
        super().__init__(parent)
        self.yaml_path = pathlib.Path(yaml_path).expanduser().resolve()

        self._yaml = YAML()
        self._yaml.preserve_quotes = True
        try:
            self._doc: Dict[str, Any] = self._yaml.load(self.yaml_path.read_text())
        except FileNotFoundError:
            raise RuntimeError(f"YAML file not found: {self.yaml_path}")
        except Exception as exc:
            raise RuntimeError(f"Cannot read YAML file {self.yaml_path}: {exc}")

        self._widgets: Dict[tuple[str, str], QLineEdit] = {}
        self._build_ui()

    def _build_ui(self):
        self.setWindowTitle(f"Parameters — {self.yaml_path.name}")
        layout = QVBoxLayout(self)

        self.form = QFormLayout()
        layout.addLayout(self.form)
        self._populate_form()

        btn_save = QPushButton("Save")
        btn_save.clicked.connect(self.save_yaml)
        layout.addWidget(btn_save, alignment=Qt.AlignmentFlag.AlignRight)

    def _populate_form(self):
        for node_name, node_block in self._doc.items():
            params = node_block.get("ros__parameters", {})
            for param, value in params.items():
                le = QLineEdit(str(value))
                self._widgets[(node_name, param)] = le
                self.form.addRow(f"{param}", le)

    @staticmethod
    def _cast(text: str, ref: Any) -> Any:
        match ref:
            case bool():
                return text.lower() in {"true", "1", "yes", "y"}
            case int():
                return int(text)
            case float():
                return float(text)
            case _:
                return text

    def save_yaml(self):
        for (node, param), w in self._widgets.items():
            orig = self._doc[node]["ros__parameters"][param]
            self._doc[node]["ros__parameters"][param] = self._cast(w.text(), orig)

        tmp = self.yaml_path.with_suffix(".tmp")
        with tmp.open("w", encoding="utf-8") as fh:
            self._yaml.dump(self._doc, fh)
        tmp.replace(self.yaml_path)

        QMessageBox.information(self, "Saved", f"Parameters written to\n{self.yaml_path}")

class CompassWidget(QWidget):
    def __init__(self, diameter=150, parent=None):
        super().__init__(parent)
        self.setFixedSize(diameter, diameter)
        self._display_heading = 0.0
        self._target_heading = 0.0
        self._anim_timer = QTimer(self)
        self._anim_timer.timeout.connect(self._animate)
        self._anim_timer.start(30)

    def setHeading(self, heading_rad: float):
        self._target_heading = heading_rad % (2 * math.pi)

    def _animate(self):
        delta = (self._target_heading - self._display_heading + math.pi) % (2 * math.pi) - math.pi
        step = delta * 0.2
        if abs(step) < 1e-3:
            self._display_heading = self._target_heading
        else:
            self._display_heading += step
        self.update()

    def paintEvent(self, event):
        size = min(self.width(), self.height())
        r = size / 2 * 0.9
        cx, cy = self.width() / 2, self.height() / 2

        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)

        p.setPen(QPen(Qt.GlobalColor.black, 2))
        p.setBrush(QBrush(Qt.GlobalColor.white))
        p.drawEllipse(int(cx - r), int(cy - r), int(2 * r), int(2 * r))

        labels = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
        metrics = QFontMetrics(p.font())
        label_r = r * 0.65
        for i, lbl in enumerate(labels):
            ang = math.radians(45 * i)
            x1, y1 = cx + math.sin(ang) * (r * 0.9), cy - math.cos(ang) * (r * 0.9)
            x2, y2 = cx + math.sin(ang) * r, cy - math.cos(ang) * r
            p.drawLine(int(x1), int(y1), int(x2), int(y2))
            tx, ty = cx + math.sin(ang) * label_r, cy - math.cos(ang) * label_r
            w = metrics.horizontalAdvance(lbl); h = metrics.ascent()
            p.drawText(int(tx - w / 2), int(ty + h / 2), lbl)

        p.setPen(QPen(Qt.GlobalColor.red, 3))
        nx, ny = cx + math.sin(self._display_heading) * (r * 0.8), cy - math.cos(self._display_heading) * (r * 0.8)
        p.drawLine(int(cx), int(cy), int(nx), int(ny))

best_effort_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

class Ros2Launcher(QWidget):
    output_signal = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Simulation Tool")
        self.process = None

        self.param_files = {
            "mclsimpy" : "~/ma1_sim/src/ma1_simulator/ma1_mclsimpy/config/sim_params.yaml",
        }

        self.x_m = []; self.y_m = []
        self.x_s = []; self.y_s = []

        # ROS2 setup
        rclpy.init(args=None)
        self.ros_node = rclpy.create_node("ros2_launch_gui")
        self.ros_node.create_subscription(Odometry, "/mclsimpy_odom", self.odom_mcsimpy, best_effort_qos)
        self.ros_node.create_subscription(Odometry, "/ma1/odom", self.odom_stonefish, best_effort_qos)

        self.init_ui()
        self.output_signal.connect(self.log.append)

        self.spin_timer = QTimer(self)
        self.spin_timer.timeout.connect(self.ros_spin)
        self.spin_timer.start(50)

    def init_ui(self):
        main_layout = QHBoxLayout(self)

        ctrl = QFrame(); ctrl.setFrameShape(QFrame.Shape.StyledPanel)
        left_layout = QVBoxLayout(ctrl)

        self.combo = QComboBox()
        self.launch_options = {
            "mclsimpy": ("ma1_mclsimpy", "ma1_mclsimpy.launch.py"),
            "Stonefish": ("ma1_stonefish", "simulation.launch.py"),
        }
        self.combo.addItems(self.launch_options.keys())
        left_layout.addWidget(self.combo)

        self.log = QTextEdit(readOnly=True)
        left_layout.addWidget(self.log)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        left_layout.addWidget(scroll, 0)
        self._param_host = QWidget()
        self._param_vbox = QVBoxLayout(self._param_host)
        self._param_vbox.setContentsMargins(0,0,0,0)
        scroll.setWidget(self._param_host)

        btn_row = QHBoxLayout()
        self.btn = QPushButton("Start"); self.btn.clicked.connect(self.toggle_launch)
        btn_row.addWidget(self.btn)

        clear_btn = QPushButton("Clear Log"); clear_btn.clicked.connect(self.log.clear)
        btn_row.addWidget(clear_btn)

        quit_btn = QPushButton("Quit"); quit_btn.clicked.connect(self.close)
        btn_row.addWidget(quit_btn)

        self._reload_params() 

        left_layout.addLayout(btn_row)
        main_layout.addWidget(ctrl, 1)

        tabs = QTabWidget()

        # mclsimpy
        t1 = QWidget(); t1l = QVBoxLayout(t1)
        self.compass_m = CompassWidget(); t1l.addWidget(self.compass_m, alignment=Qt.AlignmentFlag.AlignHCenter)
        self.plot_m = pg.PlotWidget(title="North-East plot mclsimpy")
        self.plot_m.setStyleSheet("border: 1px solid #bfbfbf;")
        self.plot_m.showGrid(x=True, y=True, alpha=0.2)
        self.plot_m.setLabel('left',   "North [m]")
        self.plot_m.setLabel('bottom', "East [m]")
        red_pen = pg.mkPen('#8B0000', width=2)
        self.curve_m = self.plot_m.plot(self.x_m, self.y_m, pen=red_pen)
        t1l.addWidget(self.plot_m)
        tabs.addTab(t1, "mcsimpy")

        # stonefish
        t2 = QWidget(); t2l = QVBoxLayout(t2)
        self.compass_s = CompassWidget(); t2l.addWidget(self.compass_s, alignment=Qt.AlignmentFlag.AlignHCenter)
        self.plot_s = pg.PlotWidget(title="North-East plot Stonefish")
        self.plot_s.setStyleSheet("border: 1px solid #bfbfbf;")
        self.plot_s.showGrid(x=True, y=True, alpha=0.2)
        self.plot_s.setLabel('left',   "North", units='m')
        self.plot_s.setLabel('bottom', "East",  units='m')
        red_pen = pg.mkPen('#8B0000', width=2)
        self.curve_m = self.plot_m.plot(self.x_m, self.y_m, pen=red_pen)
        t2l.addWidget(self.plot_s)
        tabs.addTab(t2, "Stonefish")

        main_layout.addWidget(tabs, 3)

    def _reload_params(self):
        """Recreate the ParameterEditor for the currently selected launch."""
        while self._param_vbox.count():
            w = self._param_vbox.takeAt(0).widget()
            if w:
                w.deleteLater()

        opt = self.combo.currentText()
        yaml_path = self.param_files.get(opt)
        if yaml_path:
            yaml_path = os.path.expanduser(yaml_path)
        if not yaml_path or not os.path.exists(yaml_path):
            QMessageBox.warning(
                self, "No YAML",
                f"Cannot find a parameter file for “{opt}”.\n"
                "Update Ros2Launcher.param_files with the correct path."
            )
            return

        editor = ParameterEditor(yaml_path)
        editor.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        self._param_vbox.addWidget(editor)
        self._param_vbox.addStretch()

    def odom_mcsimpy(self, msg: Odometry):
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        self.x_m.append(x); self.y_m.append(y)
        self.curve_m.setData(self.x_m, self.y_m)

        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.compass_m.setHeading(yaw)

    def odom_stonefish(self, msg: Odometry):
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        self.x_s.append(x); self.y_s.append(y)
        self.curve_s.setData(self.x_s, self.y_s)

        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.compass_s.setHeading(yaw)

    def ros_spin(self):
        rclpy.spin_once(self.ros_node, timeout_sec=0)

    def _read_stream(self, stream):
        for line in iter(stream.readline, b""):
            self.output_signal.emit(line.decode(errors="ignore"))

    def toggle_launch(self):
        if self.process is None:
            opt = self.combo.currentText(); pkg, launch = self.launch_options[opt]
            env = os.environ.copy(); env.pop("LD_LIBRARY_PATH", None); env.pop("LD_PRELOAD", None)
            setup = "/opt/ros/humble/setup.bash"; ws = os.path.expanduser("~/ma1_sim/install/setup.bash")
            cmd = ["bash", "-lc", f"source {setup} && source {ws} && exec ros2 launch {pkg} {launch}"]
            self.process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, preexec_fn=os.setsid, env=env)
            threading.Thread(target=self._read_stream, args=(self.process.stdout,), daemon=True).start()
            threading.Thread(target=self._read_stream, args=(self.process.stderr,), daemon=True).start()
            self.log.append(f"Started (PID {self.process.pid})")
            self.btn.setText("Stop"); self.combo.setEnabled(False)
        else:
            pgid = os.getpgid(self.process.pid)
            self.log.append(f"SIGINT → pg {pgid}")
            os.killpg(pgid, signal.SIGINT)
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.log.append("Killing forcefully"); os.killpg(pgid, signal.SIGTERM); self.process.wait()
            self.log.append("Stopped")
            self.process = None
            self.btn.setText("Start"); self.combo.setEnabled(True)

    def closeEvent(self, event):
        if self.process:
            os.killpg(os.getpgid(self.process.pid), signal.SIGINT)
        self.ros_node.destroy_node(); rclpy.shutdown()
        super().closeEvent(event)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    pal = QPalette()
    pal.setColor(QPalette.ColorRole.Window,       QColor("white"))
    pal.setColor(QPalette.ColorRole.Base,         QColor("white"))
    pal.setColor(QPalette.ColorRole.Text,         QColor("black"))
    pal.setColor(QPalette.ColorRole.WindowText,   QColor("black"))
    app.setPalette(pal)
    app.setStyleSheet("""
        QWidget      { background: white;  color: black; }
        /* -------- editable fields -------- */
        QLineEdit, QTextEdit {
            background: #f2f2f2;
            border: 1px solid #bfbfbf;
            padding: 2px;
        }
        QLineEdit:focus, QTextEdit:focus {
            border: 1px solid #1e90ff;
        }
        /* -------- push buttons -------- */
        QPushButton {
            background: #e6e6e6;
            border: 1px solid #bfbfbf;
            padding: 4px 8px;
        }
        QPushButton:hover {
            background: #dcdcdc;
        }
        QPushButton:pressed {
            background: #c8c8c8;
        }
        /* -------- combo-box -------- */
        QComboBox {
            background: #e6e6e6;
            border: 1px solid #bfbfbf;
            padding: 2px 6px 2px 4px;      /* room for arrow */
        }
        QComboBox QAbstractItemView {      /* drop-down list */
            background: white;
            selection-background-color: #dcdcdc;
            selection-color: black;
        }
        /* -------- tab widget -------- */
        QTabWidget::pane {
            border: 1px solid #bfbfbf;
            top: -1px;                     /* flush with tabs */
        }
        QTabBar::tab {
            background: #e6e6e6;
            color: black;
            border: 1px solid #bfbfbf;
            border-bottom: none;
            padding: 4px 12px;
            margin-right: 2px;
        }
        QTabBar::tab:selected,
        QTabBar::tab:hover {
            background: #dcdcdc;
        }
    """)
    pg.setConfigOption('background', 'w')
    pg.setConfigOption('foreground', 'k')
    pg.setConfigOption('antialias', True)
    win = Ros2Launcher()
    win.showMaximized()
    sys.exit(app.exec())


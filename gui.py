import sys
import os
import signal
import subprocess
import threading
import math

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
    QTabWidget
)
from PyQt6.QtCore import Qt, pyqtSignal, QTimer
from PyQt6.QtGui import QPainter, QPen, QBrush, QColor, QFontMetrics
import pyqtgraph as pg

best_effort_qos = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
)

import math
from PyQt6.QtWidgets import QWidget
from PyQt6.QtGui     import QPainter, QPen, QBrush, QFontMetrics
from PyQt6.QtCore    import Qt, QTimer

class CompassWidget(QWidget):
    def __init__(self, diameter=150, parent=None):
        super().__init__(parent)
        self.setFixedSize(diameter, diameter)

        # animation state
        self._display_heading = 0.0    # current needle angle (rad)
        self._target_heading  = 0.0    # where needle should move
        self._anim_timer      = QTimer(self)
        self._anim_timer.timeout.connect(self._animate)
        self._anim_timer.start(30)     # ~33 Hz

    def setHeading(self, heading_rad: float):
        """Call this with the new absolute heading in radians."""
        self._target_heading = heading_rad % (2*math.pi)

    def _animate(self):
        # shortest‐path interpolation
        delta = (self._target_heading - self._display_heading + math.pi) % (2*math.pi) - math.pi
        step = delta * 0.2
        if abs(step) < 1e-3:
            self._display_heading = self._target_heading
        else:
            self._display_heading += step
        self.update()

    def paintEvent(self, event):
        size = min(self.width(), self.height())
        r = size/2 * 0.9
        cx, cy = self.width()/2, self.height()/2

        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)

        # 1) outer circle
        p.setPen(QPen(Qt.GlobalColor.black, 2))
        p.setBrush(QBrush(Qt.GlobalColor.white))
        p.drawEllipse(int(cx-r), int(cy-r), int(2*r), int(2*r))

        # 2) ticks + labels pushed inward (0.65*r)
        labels = ['N','NE','E','SE','S','SW','W','NW']
        metrics = QFontMetrics(p.font())
        label_radius = r * 0.65
        for i, label in enumerate(labels):
            angle = math.radians(360/8 * i)

            # tick
            x1 = cx + math.sin(angle) * (r*0.9)
            y1 = cy - math.cos(angle) * (r*0.9)
            x2 = cx + math.sin(angle) * r
            y2 = cy - math.cos(angle) * r
            p.setPen(QPen(Qt.GlobalColor.black, 2))
            p.drawLine(int(x1), int(y1), int(x2), int(y2))

            # centered label
            tx = cx + math.sin(angle) * label_radius
            ty = cy - math.cos(angle) * label_radius
            w = metrics.horizontalAdvance(label)
            h = metrics.ascent()
            p.drawText(int(tx - w/2), int(ty + h/2), label)

        # 3) animated needle
        p.setPen(QPen(Qt.GlobalColor.red, 3))
        nx = cx + math.sin(self._display_heading) * (r*0.8)
        ny = cy - math.cos(self._display_heading) * (r*0.8)
        p.drawLine(int(cx), int(cy), int(nx), int(ny))

        p.end()

class Ros2Launcher(QWidget):
    output_signal = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROS2 Launch GUI")
        self.process = None

        # data buffers
        self.x_m = []; self.y_m = []
        self.x_s = []; self.y_s = []

        # init ROS2
        rclpy.init(args=None)
        self.ros_node = rclpy.create_node('ros2_launch_gui')
        self.ros_node.create_subscription(Odometry, '/mclsimpy_odom',
                                          self.odom_mcsimpy, best_effort_qos)
        self.ros_node.create_subscription(Odometry, '/ma1/odom',
                                          self.odom_stonefish, best_effort_qos)

        self.init_ui()
        self.output_signal.connect(self.log.append)

        # spin timer
        self.spin_timer = QTimer(self)
        self.spin_timer.timeout.connect(self.ros_spin)
        self.spin_timer.start(50)

    def init_ui(self):
        main_layout = QHBoxLayout(self)

        # ——— LEFT PANEL ———
        ctrl = QFrame()
        ctrl.setFrameShape(QFrame.Shape.StyledPanel)
        left_layout = QVBoxLayout(ctrl)

        # Topic selector & log fill the space
        self.combo = QComboBox()
        self.launch_options = {
            "mclsimpy": ("ma1_mclsimpy", "ma1_mclsimpy.launch.py"),
            "Stonefish": ("ma1_stonefish", "simulation.launch.py")
        }
        self.combo.addItems(self.launch_options.keys())
        left_layout.addWidget(self.combo)

        self.log = QTextEdit(readOnly=True)
        left_layout.addWidget(self.log)

        # stretch so buttons get pushed to bottom
        left_layout.addStretch()

        # button row at bottom
        btn_row = QHBoxLayout()
        self.btn = QPushButton("Start")
        self.btn.clicked.connect(self.toggle_launch)
        btn_row.addWidget(self.btn)

        clear_btn = QPushButton("Clear Log")
        clear_btn.clicked.connect(self.log.clear)
        btn_row.addWidget(clear_btn)

        left_layout.addLayout(btn_row)

        main_layout.addWidget(ctrl, 1)

        # ——— RIGHT PANEL (tabs) ———
        tabs = QTabWidget()

        # mcsimpy tab
        t1 = QWidget(); t1l = QVBoxLayout(t1)
        self.compass_m = CompassWidget()
        t1l.addWidget(self.compass_m, alignment=Qt.AlignmentFlag.AlignHCenter)
        self.plot_m = pg.PlotWidget(title="mcsimpy XY")
        self.curve_m = self.plot_m.plot(self.x_m, self.y_m, pen='y')
        t1l.addWidget(self.plot_m)
        tabs.addTab(t1, "mcsimpy")

        # Stonefish tab
        t2 = QWidget(); t2l = QVBoxLayout(t2)
        self.compass_s = CompassWidget()
        t2l.addWidget(self.compass_s, alignment=Qt.AlignmentFlag.AlignHCenter)
        self.plot_s = pg.PlotWidget(title="Stonefish XY")
        self.curve_s = self.plot_s.plot(self.x_s, self.y_s, pen='y')
        t2l.addWidget(self.plot_s)
        tabs.addTab(t2, "Stonefish")

        main_layout.addWidget(tabs, 3)

    def odom_mcsimpy(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.x_m.append(x); self.y_m.append(y)
        self.curve_m.setData(self.x_m, self.y_m)

        # extract yaw from quaternion
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0*(q.w*q.z + q.x*q.y),
            1.0 - 2.0*(q.y*q.y + q.z*q.z)
        )
        self.compass_m.setHeading(yaw)

    def odom_stonefish(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.x_s.append(x); self.y_s.append(y)
        self.curve_s.setData(self.x_s, self.y_s)

        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0*(q.w*q.z + q.x*q.y),
            1.0 - 2.0*(q.y*q.y + q.z*q.z)
        )
        self.compass_s.setHeading(yaw)

    def ros_spin(self):
        rclpy.spin_once(self.ros_node, timeout_sec=0)

    def _read_stream(self, stream):
        for line in iter(stream.readline, b''):
            text = line.decode(errors='ignore')
            self.output_signal.emit(text)

    def toggle_launch(self):
        if self.process is None:
            opt = self.combo.currentText()
            pkg, launch = self.launch_options[opt]
            env = os.environ.copy()
            env.pop("LD_LIBRARY_PATH", None); env.pop("LD_PRELOAD", None)
            setup = "/opt/ros/humble/setup.bash"
            ws = os.path.expanduser("~/ma1_sim_ws/install/setup.bash")
            cmd = ["bash","-lc",f"source {setup} && source {ws} && exec ros2 launch {pkg} {launch}"]
            self.process = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                preexec_fn=os.setsid, env=env
            )
            threading.Thread(target=self._read_stream, args=(self.process.stdout,), daemon=True).start()
            threading.Thread(target=self._read_stream, args=(self.process.stderr,), daemon=True).start()
            self.log.append(f"Started (PID {self.process.pid})")
            self.btn.setText("Stop"); self.combo.setEnabled(False)
        else:
            pgid = os.getpgid(self.process.pid)
            self.log.append(f"SIGINT → pg {pgid}")
            os.killpg(pgid, signal.SIGINT)
            try: self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.log.append("Killing forcefully")
                os.killpg(pgid, signal.SIGTERM)
                self.process.wait()
            self.log.append("Stopped")
            self.process = None
            self.btn.setText("Start"); self.combo.setEnabled(True)

    def closeEvent(self, event):
        if self.process:
            os.killpg(os.getpgid(self.process.pid), signal.SIGINT)
        self.ros_node.destroy_node()
        rclpy.shutdown()
        super().closeEvent(event)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = Ros2Launcher()
    w.showMaximized() 
    sys.exit(app.exec())

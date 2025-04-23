import sys
import os
import signal
import subprocess
import threading
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
    QLabel,
    QFrame,
    QTextEdit
)
from PyQt6.QtCore import Qt, pyqtSignal, QTimer
import pyqtgraph as pg

best_effort_qos = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
)

class Ros2Launcher(QWidget):
    output_signal = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROS2 Launch GUI")
        self.process = None
        # Data storage for odometry
        self.x_data = []
        self.y_data = []

        # Initialize ROS2
        rclpy.init(args=None)
        # Create a node for subscriptions
        self.ros_node = rclpy.create_node('ros2_launch_gui')
        # Subscribe to odometry topic
        self.ros_node.create_subscription(
            Odometry,
            '/mclsimpy_odom',
            self.odom_callback,
            best_effort_qos
        )

        self.init_ui()
        self.output_signal.connect(self.log.append)

        # Timer to spin ROS2
        self.spin_timer = QTimer(self)
        self.spin_timer.timeout.connect(self.ros_spin)
        self.spin_timer.start(50)  # spin every 50ms

    def init_ui(self):
        main_layout = QHBoxLayout(self)

        # Left control panel
        control_panel = QFrame()
        control_panel.setFrameShape(QFrame.Shape.StyledPanel)
        left_layout = QVBoxLayout(control_panel)

        # Launch selection
        self.combo = QComboBox()
        self.launch_options = {
            "mclsimpy": ("ma1_mclsimpy", "ma1_mclsimpy.launch.py"),
            "Stonefish": ("ma1_stonefish", "simulation.launch.py")
        }
        self.combo.addItems(self.launch_options.keys())
        left_layout.addWidget(self.combo)

        self.log = QTextEdit()
        self.log.setReadOnly(True)
        left_layout.addWidget(self.log)

        # Start/Stop buttons
        self.btn = QPushButton("Start")
        self.btn.clicked.connect(self.toggle_launch)
        left_layout.addWidget(self.btn)

        clear_btn = QPushButton("Clear Log")
        clear_btn.clicked.connect(self.log.clear)
        left_layout.addWidget(clear_btn)

        left_layout.addStretch()

        # Log output

        main_layout.addWidget(control_panel, 1)

        # Right panel: real-time plot
        plot_panel = QFrame()
        plot_panel.setFrameShape(QFrame.Shape.StyledPanel)
        right_layout = QVBoxLayout(plot_panel)

        # Plot widget
        self.plot_widget = pg.PlotWidget(title="Vessel XY Position")
        self.plot_curve = self.plot_widget.plot(self.x_data, self.y_data, pen='y')
        right_layout.addWidget(self.plot_widget)
        right_layout.addStretch()

        main_layout.addWidget(plot_panel, 3)

    def odom_callback(self, msg: Odometry):
        # Append new data
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.x_data.append(x)
        self.y_data.append(y)
        # Update plot
        self.plot_curve.setData(self.x_data, self.y_data)

    def ros_spin(self):
        rclpy.spin_once(self.ros_node, timeout_sec=0)

    def _read_stream(self, stream):
        for line in iter(stream.readline, b''):
            text = line.decode(errors='ignore')
            self.output_signal.emit(text)

    def toggle_launch(self):
        if self.process is None:
            opt = self.combo.currentText()
            pkg, launch_file = self.launch_options[opt]

            env = os.environ.copy()
            env.pop("LD_LIBRARY_PATH", None)
            env.pop("LD_PRELOAD", None)

            setup_script = "/opt/ros/humble/setup.bash"
            workspace = os.path.expanduser("~/ma1_sim_ws/install/setup.bash")
            cmd = [
                "bash", "-lc",
                f"source {setup_script} && source {workspace} && exec ros2 launch {pkg} {launch_file}"
            ]

            self.process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid,
                env=env
            )

            threading.Thread(target=self._read_stream, args=(self.process.stdout,), daemon=True).start()
            threading.Thread(target=self._read_stream, args=(self.process.stderr,), daemon=True).start()

            self.log.append(f"Started launch (PID {self.process.pid})")
            self.btn.setText("Stop")
            self.combo.setEnabled(False)
        else:
            pgid = os.getpgid(self.process.pid)
            self.log.append(f"Sending SIGINT to process group {pgid}")
            os.killpg(pgid, signal.SIGINT)
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.log.append("Graceful shutdown failed, killing")
                os.killpg(pgid, signal.SIGTERM)
                self.process.wait()

            self.log.append("Process stopped")
            self.process = None
            self.btn.setText("Start")
            self.combo.setEnabled(True)

    def closeEvent(self, event):
        # Cleanup ROS2
        if self.process:
            pgid = os.getpgid(self.process.pid)
            os.killpg(pgid, signal.SIGINT)
        self.ros_node.destroy_node()
        rclpy.shutdown()
        super().closeEvent(event)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    launcher = Ros2Launcher()
    launcher.show()
    sys.exit(app.exec())

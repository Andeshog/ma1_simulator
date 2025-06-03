#!/usr/bin/env python3

from mclsimpy.simulator.gunnerus import RVG_DP_6DOF
from mclsimpy.waves import WaveLoad, JONSWAP
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Wrench, TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np

class SimNode(Node):
    def __init__(self):
        super().__init__('mclsimpy_node')

        self.init_simulator()

        best_effort_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.publisher_ = self.create_publisher(Odometry, 'mclsimpy_odom', best_effort_qos)
        self.tau_sub_ = self.create_subscription(Wrench, 'joystick/control_input', self.tau_callback, best_effort_qos)
        self.timer_ = self.create_timer(self.dt, self.timer_callback)

        self.time = 0.0

        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info('mcsimpy simulator has been started.')


    def init_simulator(self):
        params = [
            'hs', 'tp', 'gamma', 'w_min', 'w_max', 'N', 'dt'
        ]
        for param in params:
            self.declare_parameter(param, 0.0)
        self.dt = self.get_parameter('dt').value
        self.vessel = RVG_DP_6DOF(dt=self.dt, method='RK4')
        # self.vessel = CSAD_DP_6DOF(dt=self.dt, method='RK4')
        self.Uc = 0.0 # Current velocity
        self.beta_c = np.pi / 4 # Current direction
        self.tau = np.array([0.0, 0.0, 0.0 ,0.0, 0.0, 0.0])

        hs = self.get_parameter('hs').value
        tp = self.get_parameter('tp').value
        gamma = self.get_parameter('gamma').value
        wp = 2*np.pi/tp # Peak frequency
        wmin = self.get_parameter('w_min').value * wp
        wmax = self.get_parameter('w_max').value * wp

        N = int(self.get_parameter('N').value)

        wave_freqs = np.linspace(wmin, wmax, N)

        jonswap = JONSWAP(wave_freqs)

        _, wave_spectrum = jonswap(hs=hs, tp=tp, gamma=gamma)

        dw = (wmax - wmin) / N
        wave_amps = np.sqrt(2 * wave_spectrum * dw)
        rand_phase = np.random.uniform(0, 2*np.pi, size=N)
        wave_angles = np.ones(N) * np.pi / 4

        self.waveload = WaveLoad(
            wave_amps=wave_amps,
            freqs=wave_freqs,
            eps=rand_phase,
            angles=wave_angles,
            config_file=self.vessel._config_file,
            interpolate=True,
            qtf_method="geo-mean",      # Use geometric mean to approximate the QTF matrices.
            deep_water=True,            # Assume deep water conditions.
        )

        self.eta = self.vessel.get_eta()
        self.nu = self.vessel.get_nu()

    def tau_callback(self, msg: Wrench):
        self.tau[0] = msg.force.x
        self.tau[1] = msg.force.y
        self.tau[5] = msg.torque.z

    def timer_callback(self):
        # tau_wave = self.waveload(self.time, self.vessel.get_eta())
        tau_wave_wf = self.waveload.first_order_loads(self.time, self.vessel.get_eta())
        tau_wave_sv = self.waveload.second_order_loads(self.time, self.vessel.get_eta()[-1])
        tau_wave = tau_wave_wf + tau_wave_sv
        self.tau = tau_wave
        self.vessel.integrate(self.Uc, self.beta_c, self.tau)
        self.eta = self.vessel.get_eta()
        self.nu = self.vessel.get_nu()

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = self.eta[0]
        msg.pose.pose.position.y = self.eta[1]
        msg.pose.pose.position.z = self.eta[2]

        quat = self.euler_to_quat(self.eta[3], self.eta[4], self.eta[5])
        msg.pose.pose.orientation.w = quat[0]
        msg.pose.pose.orientation.x = quat[1]
        msg.pose.pose.orientation.y = quat[2]
        msg.pose.pose.orientation.z = quat[3]

        msg.twist.twist.linear.x = self.nu[0]
        msg.twist.twist.linear.y = self.nu[1]
        msg.twist.twist.linear.z = self.nu[2]
        msg.twist.twist.angular.x = self.nu[3]
        msg.twist.twist.angular.y = self.nu[4]
        msg.twist.twist.angular.z = self.nu[5]

        self.publisher_.publish(msg)

        self.broadcast_transform(quat)

        self.time += self.dt

    def broadcast_transform(self, quat: np.ndarray):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = self.eta[0]
        t.transform.translation.y = self.eta[1]
        t.transform.translation.z = self.eta[2]

        t.transform.rotation.w = quat[0]
        t.transform.rotation.x = quat[1]
        t.transform.rotation.y = quat[2]
        t.transform.rotation.z = quat[3]

        self.tf_broadcaster.sendTransform(t)

    @staticmethod
    def euler_to_quat(roll: float, pitch: float, yaw: float) -> np.ndarray:
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        w = cy * cp * cr + sy * sp * sr
        x = cy * cp * sr - sy * sp * cr
        y = sy * cp * sr + cy * sp * cr
        z = sy * cp * cr - cy * sp * sr

        return np.array([w, x, y, z])
    
def main(args=None):
    rclpy.init(args=args)
    sim_node = SimNode()
    rclpy.spin(sim_node)
    sim_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

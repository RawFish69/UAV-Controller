#!/usr/bin/env python3
"""
UAV Dynamics Simulator Node

Simulates rigid-body dynamics with RK4 integration at 200 Hz.
Subscribes to /cmd/final/body_rate_thrust (FRD frame).
Publishes odometry (ENU), attitude, angular velocity, and TF transforms.
"""

import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from common_msgs.msg import BodyRateThrust
from nav_msgs.msg import Odometry
from geometry_msgs.msg import (
    QuaternionStamped,
    Vector3Stamped,
    TransformStamped,
    PoseStamped
)
from visualization_msgs.msg import Marker
import tf2_ros


class DynamicsNode(Node):
    """UAV dynamics simulator with RK4 integration."""

    def __init__(self):
        super().__init__('dynamics_node')

        # Declare parameters
        self.declare_parameter('sim_rate', 200.0)
        self.declare_parameter('mass', 1.0)
        self.declare_parameter('hover_thrust', 0.5)
        self.declare_parameter('c0', 0.0)  # Thrust model: T = mg*(c0 + c1*u)
        self.declare_parameter('c1', 1.0)
        self.declare_parameter('kv_drag', 0.1)
        self.declare_parameter('gravity', 9.81)

        # Get parameters
        self.sim_rate = self.get_parameter('sim_rate').value
        self.mass = self.get_parameter('mass').value
        self.hover_thrust = self.get_parameter('hover_thrust').value
        self.c0 = self.get_parameter('c0').value
        self.c1 = self.get_parameter('c1').value
        self.kv_drag = self.get_parameter('kv_drag').value
        self.gravity = self.get_parameter('gravity').value

        # State: [x, y, z, vx, vy, vz] in ENU frame
        # Attitude: quaternion (ENU frame)
        # Body rates: [p, q, r] in FRD body frame
        self.position = np.array([0.0, 0.0, 1.0])  # Start at 1m altitude
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.attitude_quat = np.array([1.0, 0.0, 0.0, 0.0])  # [w, x, y, z]
        self.body_rates = np.array([0.0, 0.0, 0.0])  # FRD

        # Command input
        self.cmd_body_rates = np.array([0.0, 0.0, 0.0])
        self.cmd_thrust = 0.0
        self.last_cmd_time = self.get_clock().now()

        # Publishers
        self.pub_odom = self.create_publisher(Odometry, '/state/odom', 10)
        self.pub_attitude = self.create_publisher(
            QuaternionStamped, '/state/attitude', 10
        )
        self.pub_angular_vel = self.create_publisher(
            Vector3Stamped, '/state/angular_velocity', 10
        )
        self.pub_marker = self.create_publisher(Marker, '/visualization_marker', 10)

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Subscriber
        self.sub_cmd = self.create_subscription(
            BodyRateThrust,
            '/cmd/final/body_rate_thrust',
            self.cmd_callback,
            10
        )

        # Simulation timer
        dt = 1.0 / self.sim_rate
        self.timer = self.create_timer(dt, self.sim_step)

        self.get_logger().info(
            f'Dynamics simulator started at {self.sim_rate} Hz, '
            f'mass={self.mass} kg, hover_thrust={self.hover_thrust}'
        )

    def cmd_callback(self, msg):
        """Receive body rate and thrust commands."""
        self.cmd_body_rates = np.array([
            msg.body_rates.x,
            msg.body_rates.y,
            msg.body_rates.z
        ])
        self.cmd_thrust = msg.thrust
        self.last_cmd_time = self.get_clock().now()

    def sim_step(self):
        """RK4 integration step."""
        dt = 1.0 / self.sim_rate

        # Check for command timeout (1 second)
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > 1.0:
            # No commands, fall with zero thrust
            self.cmd_thrust = 0.0
            self.cmd_body_rates = np.array([0.0, 0.0, 0.0])

        # RK4 integration
        k1 = self.dynamics(self.position, self.velocity, self.attitude_quat,
                           self.body_rates)
        
        # k2
        pos2 = self.position + 0.5 * dt * k1['vel']
        vel2 = self.velocity + 0.5 * dt * k1['acc']
        quat2 = self.integrate_quaternion(self.attitude_quat, k1['omega_body'], 0.5 * dt)
        omega2 = self.body_rates + 0.5 * dt * k1['alpha']
        k2 = self.dynamics(pos2, vel2, quat2, omega2)

        # k3
        pos3 = self.position + 0.5 * dt * k2['vel']
        vel3 = self.velocity + 0.5 * dt * k2['acc']
        quat3 = self.integrate_quaternion(self.attitude_quat, k2['omega_body'], 0.5 * dt)
        omega3 = self.body_rates + 0.5 * dt * k2['alpha']
        k3 = self.dynamics(pos3, vel3, quat3, omega3)

        # k4
        pos4 = self.position + dt * k3['vel']
        vel4 = self.velocity + dt * k3['acc']
        quat4 = self.integrate_quaternion(self.attitude_quat, k3['omega_body'], dt)
        omega4 = self.body_rates + dt * k3['alpha']
        k4 = self.dynamics(pos4, vel4, quat4, omega4)

        # Update state
        self.position += (dt / 6.0) * (k1['vel'] + 2*k2['vel'] + 2*k3['vel'] + k4['vel'])
        self.velocity += (dt / 6.0) * (k1['acc'] + 2*k2['acc'] + 2*k3['acc'] + k4['acc'])
        self.body_rates += (dt / 6.0) * (k1['alpha'] + 2*k2['alpha'] + 2*k3['alpha'] + k4['alpha'])

        # Average omega for quaternion integration
        omega_avg = (k1['omega_body'] + 2*k2['omega_body'] + 2*k3['omega_body'] + k4['omega_body']) / 6.0
        self.attitude_quat = self.integrate_quaternion(self.attitude_quat, omega_avg, dt)
        self.attitude_quat /= np.linalg.norm(self.attitude_quat)  # Normalize

        # Publish state
        self.publish_state()

    def dynamics(self, pos, vel, quat, omega_body):
        """
        Compute dynamics derivatives.
        
        Returns:
            dict with keys: vel, acc, omega_body, alpha
        """
        # Thrust in body frame (FRD): [0, 0, -T] (down is positive thrust)
        thrust_magnitude = self.mass * self.gravity * (self.c0 + self.c1 * self.cmd_thrust)
        thrust_body_frd = np.array([0.0, 0.0, -thrust_magnitude])

        # Rotate thrust from FRD body to ENU world
        # FRD to ENU conversion: need to apply rotation and frame transformation
        rot = Rotation.from_quat([quat[1], quat[2], quat[3], quat[0]])  # scipy uses [x,y,z,w]
        thrust_body_ned = thrust_body_frd  # FRD is essentially NED body frame
        
        # Convert FRD body thrust to ENU world frame
        # This requires careful frame transformation
        # For simplicity, assuming thrust is always in body Z (up in FRD = down)
        # Rotate to world ENU and account for frame convention
        thrust_world_enu = rot.apply(np.array([0.0, 0.0, thrust_magnitude]))

        # Gravity in ENU (down is negative Z)
        gravity_force = np.array([0.0, 0.0, -self.mass * self.gravity])

        # Drag force (linear)
        drag_force = -self.kv_drag * vel

        # Total acceleration
        acc = (thrust_world_enu + gravity_force + drag_force) / self.mass

        # Body rate dynamics: driven by commanded rates (simple first-order model)
        # In reality, this would involve torques and inertia
        # For this sim, we assume body rates track commands with a time constant
        tau = 0.1  # Time constant
        alpha = (self.cmd_body_rates - omega_body) / tau

        return {
            'vel': vel,
            'acc': acc,
            'omega_body': omega_body,
            'alpha': alpha
        }

    def integrate_quaternion(self, quat, omega_body, dt):
        """
        Integrate quaternion with body rates (FRD frame).
        
        Args:
            quat: [w, x, y, z]
            omega_body: [p, q, r] in FRD
            dt: time step
        
        Returns:
            Updated quaternion [w, x, y, z]
        """
        # Quaternion derivative: dq/dt = 0.5 * q * omega
        # where omega is [0, wx, wy, wz] (pure quaternion)
        w, x, y, z = quat
        p, q_rate, r = omega_body

        # Quaternion multiplication for derivative
        dq = 0.5 * np.array([
            -x*p - y*q_rate - z*r,
            w*p + y*r - z*q_rate,
            w*q_rate - x*r + z*p,
            w*r + x*q_rate - y*p
        ])

        return quat + dt * dq

    def publish_state(self):
        """Publish current state to ROS topics."""
        now = self.get_clock().now().to_msg()

        # Odometry (ENU)
        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = 'map'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = self.position[0]
        odom_msg.pose.pose.position.y = self.position[1]
        odom_msg.pose.pose.position.z = self.position[2]
        odom_msg.pose.pose.orientation.w = self.attitude_quat[0]
        odom_msg.pose.pose.orientation.x = self.attitude_quat[1]
        odom_msg.pose.pose.orientation.y = self.attitude_quat[2]
        odom_msg.pose.pose.orientation.z = self.attitude_quat[3]
        odom_msg.twist.twist.linear.x = self.velocity[0]
        odom_msg.twist.twist.linear.y = self.velocity[1]
        odom_msg.twist.twist.linear.z = self.velocity[2]
        self.pub_odom.publish(odom_msg)

        # Attitude
        att_msg = QuaternionStamped()
        att_msg.header.stamp = now
        att_msg.header.frame_id = 'map'
        att_msg.quaternion.w = self.attitude_quat[0]
        att_msg.quaternion.x = self.attitude_quat[1]
        att_msg.quaternion.y = self.attitude_quat[2]
        att_msg.quaternion.z = self.attitude_quat[3]
        self.pub_attitude.publish(att_msg)

        # Angular velocity
        ang_vel_msg = Vector3Stamped()
        ang_vel_msg.header.stamp = now
        ang_vel_msg.header.frame_id = 'body'
        ang_vel_msg.vector.x = self.body_rates[0]
        ang_vel_msg.vector.y = self.body_rates[1]
        ang_vel_msg.vector.z = self.body_rates[2]
        self.pub_angular_vel.publish(ang_vel_msg)

        # TF transform
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now
        tf_msg.header.frame_id = 'map'
        tf_msg.child_frame_id = 'base_link'
        tf_msg.transform.translation.x = self.position[0]
        tf_msg.transform.translation.y = self.position[1]
        tf_msg.transform.translation.z = self.position[2]
        tf_msg.transform.rotation.w = self.attitude_quat[0]
        tf_msg.transform.rotation.x = self.attitude_quat[1]
        tf_msg.transform.rotation.y = self.attitude_quat[2]
        tf_msg.transform.rotation.z = self.attitude_quat[3]
        self.tf_broadcaster.sendTransform(tf_msg)

        # Visualization marker (quad body axes)
        marker = Marker()
        marker.header.stamp = now
        marker.header.frame_id = 'base_link'
        marker.ns = 'uav_body'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = 0.3
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.pose.orientation.w = 1.0
        self.pub_marker.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicsNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


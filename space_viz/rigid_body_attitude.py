import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class RigidBodyAttitude(Node):

    def __init__(self):
        super().__init__('rigid_body_attitude')

        # Initial quaternion [w,x,y,z]
        self.q = np.array([1.0, 0.0, 0.0, 0.0])

        # Initial angular velocity (body frame)
        self.omega = np.array([0.5, 0.2, 1.0])

        # Non-spherical inertia (change if you want)
        self.I = np.diag([2.0, 1.0, 3.0])
        self.I_inv = np.linalg.inv(self.I)

        self.dt = 0.01
        self.timer = self.create_timer(self.dt, self.step)

        self.br = TransformBroadcaster(self)

    # ------------------------------
    # Quaternion kinematics
    # ------------------------------
    def omega_matrix(self, omega):
        wx, wy, wz = omega
        return np.array([
            [0.0, -wx, -wy, -wz],
            [wx,  0.0,  wz, -wy],
            [wy, -wz,  0.0,  wx],
            [wz,  wy, -wx,  0.0]
        ])

    def normalize(self, q):
        return q / np.linalg.norm(q)

    # ------------------------------
    # Euler rigid body equations
    # ------------------------------
    def omega_dot(self, omega):
        return self.I_inv @ (-np.cross(omega, self.I @ omega))

    # ------------------------------
    # Time step
    # ------------------------------
    def step(self):

        # === Angular velocity propagation ===
        self.omega += self.omega_dot(self.omega) * self.dt

        # === Quaternion propagation ===
        Omega = self.omega_matrix(self.omega)
        q_dot = 0.5 * Omega @ self.q
        self.q += q_dot * self.dt
        self.q = self.normalize(self.q)

        # === Publish TF ===
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "base_link"

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        t.transform.rotation.x = float(self.q[1])
        t.transform.rotation.y = float(self.q[2])
        t.transform.rotation.z = float(self.q[3])
        t.transform.rotation.w = float(self.q[0])

        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = RigidBodyAttitude()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

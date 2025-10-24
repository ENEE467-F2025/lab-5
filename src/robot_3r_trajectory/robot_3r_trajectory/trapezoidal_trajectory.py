#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
import numpy as np

class TrapezoidalTrajectoryGenerator(Node):

    def __init__(self, node_name='trapezoidal_trajectory_node'):
        super().__init__(node_name)
        self.node_name = node_name
        self.get_logger().info(f"{self.node_name} started.")
        self.declare_parameter(
            'use_min_time',
            False,
            descriptor=ParameterDescriptor(
                description="Whether to use minimum time calculation"))
        self.use_min_time = (
            self.get_parameter('use_min_time')
            .get_parameter_value()
            .bool_value
        )

    def generate_trapezoidal(self, q0, qf, t0, tf, N, vmax, acc_time, dec_time):
        q0 = np.asarray(q0, dtype=float)
        qf = np.asarray(qf, dtype=float)
        if vmax is None or acc_time is None or dec_time is None:
            raise ValueError("vmax, acc_time and dec_time must be provided for trapezoidal profile")

        a = vmax / acc_time
        d = vmax / dec_time

        times = np.linspace(t0, tf, N)
        positions = np.zeros((N, q0.size))
        velocities = np.zeros((N, q0.size))
        accelerations = np.zeros((N, q0.size))

        constant_time = tf - t0 - acc_time - dec_time
        if constant_time < 0:
            raise ValueError("The sum of acceleration and deceleration times is greater\
                              than the total time.")
        if not self.use_min_time:
            # If not using minimum time, we can use the full duration
            acc_time = dec_time = (tf - t0) / 2
        else:
            # use minimum time calculation
            total_dist = np.abs(qf - q0)
            min_acc_time = vmax / a
            min_dec_time = vmax / d
            min_const_time = (total_dist - 0.5 * a * min_acc_time**2 - 0.5 * d * min_dec_time**2) / vmax
            if min_const_time < 0:
                raise ValueError("The specified vmax, acc_time, and dec_time do not allow for a feasible trapezoidal profile.")
            acc_time = min_acc_time
            dec_time = min_dec_time
            constant_time = min_const_time
            tf = t0 + acc_time + constant_time + dec_time
            times = np.linspace(t0, tf, N)

        for i, t in enumerate(times):
            if t < t0:
                positions[i] = q0
                velocities[i] = 0
                accelerations[i] = 0
            elif t0 <= t < t0 + acc_time:
                dt = t - t0
                positions[i] = q0 + 0.5 * a * dt ** 2
                velocities[i] = a * dt
                accelerations[i] = a
            elif t0 + acc_time <= t < t0 + acc_time + constant_time:
                dt = t - (t0 + acc_time)
                positions[i] = q0 + 0.5 * a * acc_time ** 2 + vmax * dt
                velocities[i] = vmax
                accelerations[i] = 0
            elif t0 + acc_time + constant_time <= t <= tf:
                dt = t - (t0 + acc_time + constant_time)
                positions[i] = qf - 0.5 * d * (dec_time - dt) ** 2
                velocities[i] = d * (dec_time - dt)
                accelerations[i] = -d
            else:
                positions[i] = qf
                velocities[i] = 0
                accelerations[i] = 0

        return times, positions, velocities, accelerations

def main():
    rclpy.init()
    trapezoidal_trajectory_node = TrapezoidalTrajectoryGenerator()
    rclpy.spin(trapezoidal_trajectory_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

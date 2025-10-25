#!/usr/bin/env python3

"""
ROS 2 node to generate and send polynomial joint trajectories (cubic or quintic)
to a FollowJointTrajectory action server for a 3R robot.

References:
- Trajectory generation adapted from: https://www.roboticsunveiled.com/robotics-trajectory-generation/
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rcl_interfaces.msg import ParameterDescriptor
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import numpy as np
from control_msgs.action import FollowJointTrajectory

class PolynomialTrajectoryNode(Node):

    def __init__(self, node_name='polynomial_trajectory'):
        super().__init__(node_name)
        
        # Action client for trajectory controller
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/threedofbot_joint_trajectory_controller/follow_joint_trajectory'
        )
        
        self.timer_ = self.create_timer(2.0, self.send_trajectory_goal)  # send goal every 2 seconds

        self.declare_parameter(
            'traj_type',
            'quintic',
            ParameterDescriptor(description='Trajectory polynomial type: "cubic" or "quintic"')
        )
        self.declare_parameter(
            'num_points',
            50,
            ParameterDescriptor(description='Number of points in the sampled trajectory')
        )
        self.declare_parameter(
            'goal_config',
            [1.51, 0.61, 1.41],
            ParameterDescriptor(description='Target joint positions for the 3R robot')
        )

        # Subscriber for joint states
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.get_logger().info(f"{node_name} started.")

        # Define joint names for a 3R robot
        self.joint_names = ['joint1', 'joint2', 'joint3']

        # Start and end configurations 
        self.q0 = None # Initial joint positions will be set from joint states
        self.qf = self.get_parameter('goal_config').get_parameter_value().double_array_value  # Target joint positions
        self.duration = 2.0       # Seconds
        
        # Trajectory parameters (choices: 'quintic' or 'cubic')
        self.traj_type = self.get_parameter('traj_type').value
        self.num_points = int(self.get_parameter('num_points').value)

        # Flag to track if we've received joint states
        self.joint_states_received = False

        # Action / goal state flags
        self.goal_active = False
        self.goal_reached = False

    def joint_state_callback(self, msg):
        """Callback to update q0 with current joint states."""
        if len(msg.position) >= 3:
            self.q0 = [float(msg.position[0]), float(msg.position[1]), float(msg.position[2])]
            if not self.joint_states_received:
                self.get_logger().info(f"Updated q0 with joint states: {self.q0}")
                self.joint_states_received = True

    def generate_trajectory(self, profile, q0, qf, t0, tf, N, max_vel=None, acc_time=None, dec_time=None, plot=False):
        """Generate a point-to-point trajectory.

        profile: 'cubic', 'quintic', or 'trapezoidal'
        q0, qf: iterable of joint positions (length = num_joints)
        t0, tf: start and end time
        N: number of points
        Returns: times, positions (N x num_joints), velocities, accelerations
        """
        q0 = np.asarray(q0, dtype=float)
        qf = np.asarray(qf, dtype=float)

        if profile == 'cubic':
            times, positions, velocities, accelerations = self._cubic_trajectory(q0, qf, t0, tf, N)
        elif profile == 'quintic':
            times, positions, velocities, accelerations = self._quintic_trajectory(q0, qf, t0, tf, N)
        else:
            raise ValueError("Invalid polynomial trajectory profile")

        if plot:
            try:
                self._plot_trajectory_profiles(times, positions, velocities, accelerations)
            except Exception as e:
                self.get_logger().warn(f"Plotting failed: {e}")

        return times, positions, velocities, accelerations

    def _cubic_trajectory(self, q0, qf, t0, tf, num_points):
        """
        Generate a cubic polynomial trajectory.
        """
        q0 = np.asarray(q0, dtype=float)
        qf = np.asarray(qf, dtype=float)
        A = np.array([[1, t0, t0**2, t0**3],
                      [1, tf, tf**2, tf**3],
                      [0, 1, 2*t0, 3*t0**2],
                      [0, 1, 2*tf, 3*tf**2]])

        b = np.vstack([q0, qf, np.zeros_like(q0), np.zeros_like(q0)])
        coeffs = np.linalg.solve(A, b)

        times = np.linspace(t0, tf, num_points)
        T = times[:, None]
        positions = coeffs[0] + coeffs[1] * T + coeffs[2] * T ** 2 + coeffs[3] * T ** 3
        velocities = coeffs[1] + 2 * coeffs[2] * T + 3 * coeffs[3] * T ** 2
        accelerations = 2 * coeffs[2] + 6 * coeffs[3] * T

        return times, positions, velocities, accelerations

    def _quintic_trajectory(self, q0, qf, t0, tf, num_points):
        """
        Generate a quintic polynomial trajectory.
        """
        q0 = np.asarray(q0, dtype=float)
        qf = np.asarray(qf, dtype=float)
        A = np.array([[1, t0, t0**2, t0**3, t0**4, t0**5],
                      [1, tf, tf**2, tf**3, tf**4, tf**5],
                      [0, 1, 2*t0, 3*t0**2, 4*t0**3, 5*t0**4],
                      [0, 1, 2*tf, 3*tf**2, 4*tf**3, 5*tf**4],
                      [0, 0, 2, 6*t0, 12*t0**2, 20*t0**3],
                      [0, 0, 2, 6*tf, 12*tf**2, 20*tf**3]])

        b = np.vstack([q0, qf, np.zeros_like(q0), np.zeros_like(q0), np.zeros_like(q0), np.zeros_like(q0)])
        coeffs = np.linalg.solve(A, b)

        times = np.linspace(t0, tf, num_points)
        T = times[:, None]
        positions = coeffs[0] + coeffs[1] * T + coeffs[2] * T ** 2 + coeffs[3] * T ** 3 + coeffs[4] * T ** 4 + coeffs[5] * T ** 5
        velocities = coeffs[1] + 2 * coeffs[2] * T + 3 * coeffs[3] * T ** 2 + 4 * coeffs[4] * T ** 3 + 5 * coeffs[5] * T ** 4
        accelerations = 2 * coeffs[2] + 6 * coeffs[3] * T + 12 * coeffs[4] * T ** 2 + 20 * coeffs[5] * T ** 3

        return times, positions, velocities, accelerations

    def _plot_trajectory_profiles(self, times, positions, velocities, accelerations):
        try:
            import matplotlib.pyplot as plt
        except Exception:
            raise

        num_joints = positions.shape[1]
        fig, axes = plt.subplots(3, 1, figsize=(8, 6), sharex=True)
        for j in range(num_joints):
            axes[0].plot(times, positions[:, j], label=f'joint{j+1}')
            axes[1].plot(times, velocities[:, j], label=f'joint{j+1}')
            axes[2].plot(times, accelerations[:, j], label=f'joint{j+1}')

        axes[0].set_ylabel('Position')
        axes[1].set_ylabel('Velocity')
        axes[2].set_ylabel('Acceleration')
        axes[2].set_xlabel('Time [s]')
        for ax in axes:
            ax.legend()
            ax.grid(True)
        plt.tight_layout()
        plt.show()

    def send_trajectory_goal(self):
        # Only send goal if we have received joint states
        if not self.joint_states_received:
            self.get_logger().info("Waiting for joint states...")
            return
        # Do not send new goals if one is active or already reached
        if self.goal_active:
            self.get_logger().info("Goal already active; skipping send.")
            return
        if self.goal_reached:
            self.get_logger().info("Goal already reached; not sending more trajectories.")
            return
            
        # Wait for action server
        if not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("Trajectory controller action server not available")
            return
        # Create full sampled trajectory using selected polynomial type
        times, positions, velocities, accelerations = self.generate_trajectory(
            self.traj_type, self.q0, self.qf, 0.0, self.duration, self.num_points
        )

        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        points = []
        for i, t in enumerate(times):
            p = JointTrajectoryPoint()
            p.positions = positions[i].tolist()
            # velocities and accelerations may be multi-dim; convert to list
            try:
                p.velocities = velocities[i].tolist()
            except Exception:
                p.velocities = [0.0] * len(self.joint_names)
            try:
                p.accelerations = accelerations[i].tolist()
            except Exception:
                p.accelerations = [0.0] * len(self.joint_names)

            # set time_from_start (sec, nanosec)
            sec = int(np.floor(t))
            nsec = int((t - sec) * 1e9)
            p.time_from_start.sec = int(sec)
            p.time_from_start.nanosec = int(nsec)

            points.append(p)

        traj.points = points

        # Create goal and send
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = traj

        self.get_logger().info(f"Sending {len(points)}-point {self.traj_type} trajectory from {self.q0} to {self.qf}")
        send_goal_future = self.action_client.send_goal_async(goal_msg)
        # mark active
        self.goal_active = True
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Trajectory goal rejected by controller")
            self.goal_active = False
            return

        self.get_logger().info("Trajectory goal accepted")
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Trajectory execution finished with result: {result}")
        # Goal finished
        self.goal_active = False
        # check result for success. Here assume non-empty result indicates success
        try:
            # if result has a 'error_code' or 'error' field use it; fallback to marking reached
            self.goal_reached = True
            # stop periodic sends once reached
            try:
                self.timer_.cancel()
            except Exception:
                pass
        except Exception:
            pass

def main():
        rclpy.init()
        node = PolynomialTrajectoryNode()
        rclpy.spin(node)
        rclpy.shutdown()

if __name__ == '__main__':
    main()

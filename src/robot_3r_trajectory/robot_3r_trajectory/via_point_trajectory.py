#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker, MarkerArray
from interactive_markers import InteractiveMarkerServer, MenuHandler
from geometry_msgs.msg import PoseStamped, Point
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rcl_interfaces.msg import ParameterDescriptor
import transforms3d as t3d
from pymoveit2.robots import robot_3r as robot
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
import numpy as np

# Raw waypoint list (start -> goal)
_raw_waypoints = np.array([
    [6.67572021e-06, 0.0, 0.0],
    [0.13899075, 0.04891747, 0.13524241],
    [0.27797483, 0.09783493, 0.27048481],
    [0.4169589, 0.1467524, 0.40572722],
    [0.52223452, -0.01725403, 0.36079465],
    [0.4169589, 0.1467524, 0.40572722],
    [0.55594298, 0.19566986, 0.54096962],
    [0.69492705, 0.24458733, 0.67621203],
    [0.83391113, 0.2935048, 0.81145444],
    [0.9728952, 0.34242226, 0.94669684],
    [1.11187928, 0.39133973, 1.08193925],
    [1.25086335, 0.44025719, 1.21718165],
    [1.38984743, 0.48917466, 1.35242406],
    [1.42345092, 0.31550748, 1.4457542],
    [1.55819323, 0.40559476, 1.56292414],
    [1.69293553, 0.49568204, 1.68009407],
    [1.85112115, 0.52571183, 1.56145378],
    [1.69293553, 0.49568204, 1.68009407],
    [1.98541976, 0.52483054, 1.70965333],
    [2.00098741, 0.6437181, 1.86972643],
    [2.0093, 0.7072, 1.9552],
])

# select 5 equally spaced waypoints including start and goal
_indices = np.linspace(0, len(_raw_waypoints) - 1, 5).astype(int)
VIA_POINT_PATH = _raw_waypoints[_indices].tolist()
# robot data
JOINT_NAMES = robot.joint_names("")
JOINT_LIMITS = robot.get_joint_limits()
GROUP_STATES = robot.get_named_group_states("")
DISABLED_COLLISION_PAIRS = robot.get_disabled_collision_pairs("")
BASE_LINK_NAME = robot.base_link_name("")
LINK_NAMES = robot.get_link_names("")
RTB_MODEL = robot.get_rtb_model()
WORLD_FRAME = robot.world_frame("")

class ViaPointTrajectoryGenerator(Node):
    def __init__(self, node_name="via_point_trajectory_node"):
        self.node_name = node_name
        super().__init__(self.node_name)
        # Marker publisher for via points
        self.declare_parameter('marker_frame', 'world')
        self.declare_parameter('marker_scale', 0.05)
        self.declare_parameter('marker_color', [1.0, 0.0, 0.0, 1.0])  # RGBA
        # publish MarkerArray for EE path markers
        self.marker_pub = self.create_publisher(MarkerArray, 'via_point_markers', 10)
        # publish last EE pose as PoseStamped
        self.ee_path_pub = self.create_publisher(PoseStamped, 'via_point/ee_pose', 10)
        # robot kinematics helpers
        self.RTB_MODEL = RTB_MODEL
        self.BASE_LINK_NAME = BASE_LINK_NAME
        # publish markers at 1 Hz
        self.create_timer(1.0, self.publish_markers)

        # Control logic (copied/adapted from polynomial_trajectory.py) ---
        # action client for controller
        self.declare_parameter('controller_action_topic', '/threedofbot_joint_trajectory_controller/follow_joint_trajectory')
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            self.get_parameter('controller_action_topic').get_parameter_value().string_value
        )

        # periodic sender (2s default)
        self.send_timer = self.create_timer(2.0, self.send_trajectory_goal)

        # trajectory params
        self.declare_parameter('traj_type', 'quintic', ParameterDescriptor(description='Trajectory polynomial type: "cubic" or "quintic"'))
        self.declare_parameter('traj_points', 10, ParameterDescriptor(description='Number of points in the sampled trajectory'))

        # subscriber for joint states to obtain q0
        self.q0 = None
        self.joint_states_received = False
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        # joint names for the 3R robot
        self.joint_names = ['joint1', 'joint2', 'joint3']

        # goal flags
        self.goal_active = False
        self.goal_reached = False

        # default duration for polynomial point-to-point
        self.duration = 2.0

        # expose goal via parameter
        self.declare_parameter('goal_config', VIA_POINT_PATH[-1])
        self.qf = self.get_parameter('goal_config').get_parameter_value().double_array_value
        # get traj params
        self.traj_type = self.get_parameter('traj_type').value
        self.traj_points = int(self.get_parameter('traj_points').value)


    def generate_via_point_trajectory(self, via_points, times_unused, N):
        """
        Generate a multi-segment polynomial trajectory by calling self.generate_trajectory()
        between each consecutive pair of via points.
        """

        via_points = [list(map(float, vp)) for vp in via_points]
        num_segments = len(via_points) - 1

        if num_segments < 1:
            self.get_logger().error("Not enough via points to generate trajectory.")
            return JointTrajectory()

        # Points per segment
        pts_per_seg = max(2, int(N / num_segments))

        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        for i in range(num_segments):
            q0 = via_points[i]
            qf = via_points[i + 1]

            times, positions, velocities, accelerations = self.generate_trajectory(
                self.traj_type, q0, qf, 0.0, self.duration, pts_per_seg
            )

            for k, t in enumerate(times):
                # Skip the first point of each segment except the first one
                if i > 0 and k == 0:
                    continue

                p = JointTrajectoryPoint()
                p.positions = positions[k].tolist()
                p.velocities = velocities[k].tolist() if velocities is not None else [0.0] * len(q0)
                p.accelerations = accelerations[k].tolist() if accelerations is not None else [0.0] * len(q0)

                sec = int(np.floor(t + i * self.duration))
                nsec = int(((t + i * self.duration) - sec) * 1e9)
                p.time_from_start.sec = sec
                p.time_from_start.nanosec = nsec

                traj.points.append(p)

        return traj


    # Polynomial generators copied from polynomial_trajectory.py
    def generate_trajectory(self, profile, q0, qf, t0, tf, N, max_vel=None, acc_time=None, dec_time=None, plot=False):
        q0 = np.asarray(q0, dtype=float)
        qf = np.asarray(qf, dtype=float)

        if profile == 'cubic':
            times, positions, velocities, accelerations = self._cubic_trajectory(q0, qf, t0, tf, N)
        elif profile == 'quintic':
            times, positions, velocities, accelerations = self._quintic_trajectory(q0, qf, t0, tf, N)
        else:
            raise ValueError("Invalid polynomial trajectory profile")

        return times, positions, velocities, accelerations

    def _cubic_trajectory(self, q0, qf, t0, tf, num_points):
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

    def joint_state_callback(self, msg: JointState):
        if len(msg.position) >= 3:
            self.q0 = [float(msg.position[0]), float(msg.position[1]), float(msg.position[2])]
            if not self.joint_states_received:
                self.get_logger().info(f"Updated q0 with joint states: {self.q0}")
                self.joint_states_received = True

    def send_trajectory_goal(self):
        if not self.joint_states_received:
            self.get_logger().info("Waiting for joint states...")
            return
        if self.goal_active:
            self.get_logger().info("Goal already active; skipping send.")
            return
        if self.goal_reached:
            self.get_logger().info("Goal already reached; not sending more trajectories.")
            return

        if not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("Trajectory controller action server not available")
            return

        # Generate full via-point stitched trajectory instead of point-to-point
        traj = self.generate_via_point_trajectory(
            via_points=VIA_POINT_PATH,
            times_unused=None,     
            N=self.traj_points * len(VIA_POINT_PATH)  
        )

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = traj
        self.get_logger().info(f"Sending {len(traj.points)}-point VIA trajectory to controller")
        send_goal_future = self.action_client.send_goal_async(goal_msg)
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
        self.goal_active = False
        try:
            self.goal_reached = True
            try:
                self.send_timer.cancel()
            except Exception:
                pass
        except Exception:
            pass

    def publish_markers(self):
        """
        Compute EE poses from joint-space via points using FK and publish:
         - a LINE_STRIP MarkerArray representing the EE path
         - the last EE PoseStamped on `via_point/ee_pose`
        """
        frame = self.get_parameter('marker_frame').value
        scale = float(self.get_parameter('marker_scale').value)
        color = self.get_parameter('marker_color').value

        points = []
        last_pose_stamped = None
        for q in VIA_POINT_PATH:
            try:
                # q is expected to be an iterable of joint values
                self.RTB_MODEL.q = q
                T = self.RTB_MODEL.fkine(q).A
            except Exception as e:
                self.get_logger().warning(f"FK failed for via point {q}: {e}")
                continue

            trans = T[:3, 3].tolist()
            R = T[:3, :3]
            quat = t3d.quaternions.mat2quat(R)  # (w, x, y, z)

            ps = PoseStamped()
            ps.header.frame_id = self.BASE_LINK_NAME
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.pose.position.x = float(trans[0])
            ps.pose.position.y = float(trans[1])
            ps.pose.position.z = float(trans[2])
            ps.pose.orientation.w = float(quat[0])
            ps.pose.orientation.x = float(quat[1])
            ps.pose.orientation.y = float(quat[2])
            ps.pose.orientation.z = float(quat[3])
            last_pose_stamped = ps

            points.append(Point(x=trans[0], y=trans[1], z=trans[2]))

        if points:
            markers = []
            # First: a green LINE_STRIP for the full EE path
            line_marker = Marker()
            line_marker.header.frame_id = self.BASE_LINK_NAME
            line_marker.header.stamp = self.get_clock().now().to_msg()
            line_marker.ns = "ee_path"
            line_marker.id = 0
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            line_marker.scale.x = max(0.001, float(scale) * 0.3)
            line_marker.color.a = 1.0
            line_marker.color.r = 1.0
            line_marker.color.g = 1.0
            line_marker.color.b = 1.0
            line_marker.points = points
            line_marker.lifetime = Duration(seconds=0).to_msg()
            markers.append(line_marker)

            # Then: green spheres for each via point
            for idx, p in enumerate(points, start=1):
                marker = Marker()
                marker.header.frame_id = self.BASE_LINK_NAME
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "ee_path_spheres"
                marker.id = idx
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                # set sphere pose from point
                marker.pose.position.x = float(p.x)
                marker.pose.position.y = float(p.y)
                marker.pose.position.z = float(p.z)
                marker.pose.orientation.w = 1.0
                # use parameterized scale for sphere diameter
                s = max(0.001, float(scale))
                marker.scale.x = s
                marker.scale.y = s
                marker.scale.z = s
                marker.color.a = 1.0
                marker.color.r = 0.5
                marker.color.g = 0.0
                marker.color.b = 0.5
                marker.lifetime = Duration(seconds=0).to_msg()
                markers.append(marker)

            self.marker_pub.publish(MarkerArray(markers=markers))

        if last_pose_stamped is not None:
            self.ee_path_pub.publish(last_pose_stamped)


def main():
    rclpy.init()
    via_point_trajectory_node = ViaPointTrajectoryGenerator()
    rclpy.spin(via_point_trajectory_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
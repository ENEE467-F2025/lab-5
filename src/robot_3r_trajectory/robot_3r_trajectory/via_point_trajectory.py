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
    [-0.0, 0.0, 0.0],
    [-0.05, 0.17, 0.09],
    [-0.2, 0.29, 0.12],
    [0.08, 0.3, 0.17],
    [0.17, 0.43, 0.3],
    [0.22, 0.58, 0.42],
    [0.4, 0.67, 0.46],
    [0.37, 0.82, 0.59],
    [0.59, 0.7, 0.45],
    [0.75, 0.71, 0.57],
    [0.87, 0.83, 0.69],
    [1.04, 0.91, 0.75],
    [1.13, 1.01, 0.9],
    [1.12, 1.01, 1.29],
    [1.17, 0.85, 1.4],
    [1.24, 1.32, 1.51],
    [1.18, 1.44, 1.66],
    [1.04, 1.48, 1.8],
    [1.01, 1.51, 1.41],
])

# select all waypoints
VIA_POINT_PATH = _raw_waypoints.tolist()

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
        self.marker_pub = self.create_publisher(MarkerArray, 'via_point_markers', 10)
        self.ee_path_pub = self.create_publisher(PoseStamped, 'via_point/ee_pose', 10)
        self.RTB_MODEL = RTB_MODEL
        self.BASE_LINK_NAME = BASE_LINK_NAME
        self.create_timer(1.0, self.publish_markers) # 1 Hz

        # action client for controller
        self.declare_parameter('controller_action_topic', '/threedofbot_joint_trajectory_controller/follow_joint_trajectory')
        self.controller_action_client = ActionClient(
            self,
            FollowJointTrajectory,
            self.get_parameter('controller_action_topic').get_parameter_value().string_value
        )

        # periodic sender
        # TODO (Exercise 2b): create a timer that calls self.send_trajectory_goal every second

        # trajectory params
        self.declare_parameter('points_per_segment', 10, ParameterDescriptor(description='Number of points in the sampled trajectory between each pair of via points'))
        self.declare_parameter('trajectory_duration', 2.0, ParameterDescriptor(description='Duration (s) for each point-to-point segment in the via-point trajectory'))

        # subscriber for joint states to obtain q0
        self.q0 = None
        self.joint_states_received = False
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        # joint names for the 3R robot
        self.joint_names = ['joint1', 'joint2', 'joint3']

        # goal flags
        self.goal_active = False
        self.goal_reached = False

        # expose goal via parameter
        self.declare_parameter('goal_config', VIA_POINT_PATH[-1])
        self.qf = self.get_parameter('goal_config').get_parameter_value().double_array_value

        # get traj params
        self.points_per_segment = int(self.get_parameter('points_per_segment').get_parameter_value().integer_value)
        self.duration = self.get_parameter('trajectory_duration').get_parameter_value().double_value


    def generate_via_point_trajectory(self, via_points):
        """
        Generate a multi-segment cubic trajectory through a list of via-points.
        Each segment connects consecutive via-points V[i] -> V[i+1].
        """
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        # Convert via-points to float arrays
        M = [list(map(float, q)) for q in via_points]
        num_segments = len(M) - 1
        for i in range(num_segments):
            ###########################################################
            # TODO (Exercise 2a) compute trajectory between consecutive 
            # via-point pairs (joint configs) in via_points
            ###########################################################
            
            # endpoints for the ith segment
            q0 = []   # <-- MODIFY: starting configuration of segment i
            qf = []   # <-- MODIFY: ending configuration of segment i

            # generate cubic trajectory for this segment
            times, positions, velocities, accelerations = \
                self._generate_cubic_trajectory(
                    q0=q0,
                    qf=qf,
                    t0=0.0,
                    tf=0.0,  # <-- MODIFY: duration of this segment
                    N=0      # <-- MODIFY: number of interpolated points
                )
            ##########################################################
            # TODO (END)
            ##########################################################

            # DO NOT MODIFY BELOW THIS LINE
            # Assemble JointTrajectory message
            for k, t in enumerate(times):
                # skip duplicate points at segment boundaries
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


    def _generate_cubic_trajectory(self, q0, qf, t0, tf, N):
        """
        Generate a cubic polynomial trajectory from q0 to qf between times t0 and tf.
        Combines trajectory generation and cubic polynomial calculation.
        """
        q0 = np.asarray(q0, dtype=float)
        qf = np.asarray(qf, dtype=float)

        # Solve cubic polynomial coefficients
        A = np.array([
            [1, t0, t0**2, t0**3],
            [0, 1, 2*t0, 3*t0**2],
            [1, tf, tf**2, tf**3],
            [0, 1, 2*tf, 3*tf**2]
        ])

        v0 = np.zeros_like(q0)
        vf = np.zeros_like(qf)
        b = np.vstack([q0, v0, qf, vf])
        coeffs = np.linalg.solve(A, b)

        # Generate times and trajectory points
        times = np.linspace(t0, tf, N)
        T = times[:, None]
        positions = coeffs[0] + coeffs[1] * T + coeffs[2] * T ** 2 + coeffs[3] * T ** 3
        velocities = coeffs[1] + 2 * coeffs[2] * T + 3 * coeffs[3] * T ** 2
        accelerations = 2 * coeffs[2] + 6 * coeffs[3] * T

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

        if not self.controller_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("Trajectory controller action server not available")
            return

        # Generate full via-point stitched trajectory using the configured
        # points_per_segment parameter. The generator will use
        # self.points_per_segment for each segment.
        traj = self.generate_via_point_trajectory(via_points=VIA_POINT_PATH)

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = traj
        self.get_logger().info(f"Sending {len(traj.points)}-point VIA trajectory to controller")
        send_goal_future = self.controller_action_client.send_goal_async(goal_msg)
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
        self.get_logger().info(
        f"\033[92m{result.error_string}\033[0m"
        )
        self.goal_active = False
        try:
            self.goal_reached = True
            try:
                # TODO (Exercise 2c): cancel the periodic sending of trajectory goals
                # replace the pass statement below with the appropriate code
                pass
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

            for idx, p in enumerate(points, start=1):
                marker = Marker()
                marker.header.frame_id = self.BASE_LINK_NAME
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "ee_path_spheres"
                marker.id = idx
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = float(p.x)
                marker.pose.position.y = float(p.y)
                marker.pose.position.z = float(p.z)
                marker.pose.orientation.w = 1.0
                s = max(0.001, float(scale))
                marker.scale.x = s
                marker.scale.y = s
                marker.scale.z = s
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 1.0
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
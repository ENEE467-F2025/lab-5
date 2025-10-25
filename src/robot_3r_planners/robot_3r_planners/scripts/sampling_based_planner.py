#!/usr/bin/env python3

"""
ROS 2 node to find a collision-free trajectory by sampling from the attendant collision-free subspace (in joint space).

Only the RRTStar (RRTstarkConfigDefault) algorithm is implemented in this script.
Support for RRT (RRTkConfigDefault) is planned.

Usage: ros2 run robot_3r_planners sampling_based_planner.py --ros-args -p goal_config:="[1.5093, 0.6072, 1.4052]" -p check_collision:=True

Author: Clinton Enwerem.
Developed for the course ENEE467: Robotics Projects Laboratory, Fall 2025, University of Maryland, College Park, MD.
"""

# This node focuses on wiring parameters, ROS I/O, and high-level planning flow.
# Heavyweight utilities are factored into:
#   - robot_3r_planners.utils.planning_utils: math helpers (distance, path cost)
#   - robot_3r_planners.utils.collision_utils: collision checking along path segments
#   - robot_3r_planners.visualization.path_markers: helpers to build RViz markers

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import numpy as np
import random
from pymoveit2.robots import robot_3r as robot
from robot_3r_interfaces.msg import JointWaypoint, JointSpacePath
from robot_3r_interfaces.srv import CheckCollision

# obstacles and collision detection
from robot_3r_interfaces.msg import SceneObstacles, RigidBodyGeom
from visualization_msgs.msg import MarkerArray
from robot_3r_planners.visualization.path_markers import (
    build_ee_path_line_strip,
    build_jsp_waypoint_markers,
)
from robot_3r_planners.utils.param_utils import (
    PlannerParams,
    declare_goal_config,
)
from robot_3r_planners.utils.planning_utils import (
     calc_distance_and_angle,
     compute_path_cost,
     RRTStar,
)

# robot data
JOINT_NAMES = robot.joint_names("")
JOINT_LIMITS = robot.get_joint_limits()
GROUP_STATES = robot.get_named_group_states("")
DISABLED_COLLISION_PAIRS = robot.get_disabled_collision_pairs("")
BASE_LINK_NAME = robot.base_link_name("")
LINK_NAMES = robot.get_link_names("")
RTB_MODEL = robot.get_rtb_model()
WORLD_FRAME = robot.world_frame("")

class SamplingBasedJSPlanner(Node):
    # init ROS2 Node
    def __init__(self, node_name:str="sampling_based_planner", queue_size=10):
        self.node_name = node_name
        self.queue_size = queue_size
        super().__init__(self.node_name)
        self.get_logger().info("")
        self.get_logger().info(f"\n--------------------------------------------------\nInitializing {self.node_name} node...\n--------------------------------------------------")

        ######################################################################################################################
        # declare and get params
        ######################################################################################################################
        self.params = PlannerParams()
        self.params.declare(self)
        self.params.load(self)

        # Mirror frequently accessed fields to keep code below unchanged
        self.verbose = self.params.verbose
        self.print_metrics = self.params.print_metrics
        self.def_group_state = self.params.def_group_state
        self.pl_alg = self.params.planning_algorithm
        self.moveit_config_name = self.params.moveit_config_name
        self.joint_limits = JOINT_LIMITS
        self.rrts_expand_dist = self.params.rrts_expand_dist
        self.rrts_path_resolution = self.params.rrts_path_resolution
        self.rrts_max_iter = int(self.params.rrts_max_iter)
        self.rrts_connect_circle_dist = int(self.params.rrts_connect_circle_dist)
        self.rrts_goal_sample_rate = self.params.rrts_goal_sample_rate
        self.rrts_search_until_max_iter = self.params.rrts_search_until_max_iter
        self.show_jsp_waypoints = self.params.show_jsp_waypoints
        self.show_ee_path = self.params.show_ee_path
        self.check_collision = self.params.check_collision
        self.stop_if_plan_found = self.params.stop_if_plan_found
        self.min_obs_dist = self.params.min_obs_dist
        self.collision_checker = self.params.collision_checker
        self.max_planning_attempts = int(self.params.max_planning_attempts)
        self.stop_on_failure = self.params.stop_on_failure
        self.proximity_alert = self.params.proximity_alert
        self.start_goal_collision = self.params.start_goal_collision
        self.use_goal_biased_sampling = self.params.use_goal_biased_sampling
        self.goal_noise_sigma = self.params.goal_noise_sigma
        self.use_collision_service = getattr(self.params, "use_collision_service", False)

        # Set random seed
        self.random_seed = int(self.params.random_seed)
        np.random.seed(self.random_seed)
        random.seed(self.random_seed)
        if self.verbose:
            self.get_logger().info(f"Random seed set to: {self.random_seed}")

        # Planning state tracking
        if self.stop_if_plan_found:
            self.planning_done = False
        self.planning_attempts = 0
        self.planning_failed = False

        # goal_config comes from SRDF named state when available
        self.goal_config = declare_goal_config(
            node=self,
            group_states=GROUP_STATES,
            def_group_state=self.def_group_state,
            fallback_goal=self.params.goal_config,
        )
        if self.verbose:
            self.get_logger().info(f"Planning to goal configuration: {self.goal_config} using {self.pl_alg} algorithm.")
        ######################################################################################################################
        # End declare and get params
        ######################################################################################################################

        ######################################################################################################################
        # subscribers and publishers
        ######################################################################################################################
        # subscriber for robot state
        self.create_subscription(
            JointState,
            "/joint_states",
            self.compute_plan,
            self.queue_size
        )
        # subscriber for obstacles
        self.create_subscription(
            SceneObstacles,
            "/scene_obstacles",
            self.get_scene_obs_cb,
            self.queue_size
        )
        # publisher for planned path; needed by the trajectory planner and executor nodes
        self.plan_pub = self.create_publisher(
            JointSpacePath,
            "smpb_planner/jsp_path",
            self.queue_size
        )
        # publisher for end effector pose along path
        self.ee_path_pub = self.create_publisher(
            PoseStamped,
            "smpb_planner/ee_path",
            self.queue_size
        )

        # publisher and tf listener for robot geometry
        self.create_subscription(
            RigidBodyGeom,
            "robot_geometry",
            self.robot_geom_cb,
            self.queue_size
        )

        # timer to stop the node if a valid plan is found
        if self.stop_if_plan_found:
            self.stop_timer = self.create_timer(1.0, self.stop_node_cb)

        # path marker array pub
        self.jsp_path_marker_pub = self.create_publisher(MarkerArray, "planned_jsp_path_markers", 10)
        self.ee_path_marker_pub = self.create_publisher(MarkerArray, "ee_path_marker", 10)

        # timer to periodically publish markers
        if self.show_jsp_waypoints:
            self.jsp_path_marker_pub_timer = self.create_timer(1.0, self.publish_jsp_path_markers_cb)
        if self.show_ee_path:
            self.ee_path_marker_pub_timer = self.create_timer(1.0, self.publish_ee_path_markers_cb)

        # planner init
        self.start_config = None
        self.obstacle_geom = None
        self.robot_geom = None
        
        self.rand_area = [(low, high) for (low, high) in JOINT_LIMITS] # range (in radians) for randomly sampling angles in joint space
        self.rrt_star = None 
        self.computed_path = None # store path for visualization

        # Optional collision service client
        self._collision_cli = None
        if self.use_collision_service:
            try:
                self._collision_cli = self.create_client(CheckCollision, 'check_collision')
                while not self._collision_cli.wait_for_service(timeout_sec=1.0):
                    self.get_logger().info('Waiting for collision checker service...')
                if self.verbose:
                    self.get_logger().info('Collision checker service available.')
            except Exception as e:
                self.get_logger().warn(f"Failed to create collision service client: {e}")
                self._collision_cli = None

    def stop_node_cb(self):
        if getattr(self, "planning_done", False):
            return
        elif getattr(self, "planning_failed", False) and self.stop_on_failure:
            self.get_logger().info("Stopping node due to planning failure.")
            raise SystemExit

    def compute_plan(self, msg:JointState):
        if getattr(self, "planning_done", False):
            return # dont replan if we already found a valid path
            
        if getattr(self, "planning_failed", False):
            return # dont replan if planning has permanently failed

        self.start_config = msg.position
        if self.start_config is not None and self.obstacle_geom is not None and self.robot_geom is not None:
            self.rrt_star = RRTStar(
                self,
                start=self.start_config,
                goal=self.goal_config,
                robot_geom=self.robot_geom,
                obstacle_geom=self.obstacle_geom,
                rand_area=self.rand_area,
                expand_dist=self.rrts_expand_dist,
                path_resolution=self.rrts_path_resolution,
                max_iter=self.rrts_max_iter,
                connect_circle_dist=self.rrts_connect_circle_dist,
                goal_sample_rate=self.rrts_goal_sample_rate,
                check_collision_param=self.check_collision,
                collision_fn=(
                    (lambda path_q, robot_geom, obstacles, node, rtb_model, base_link_name, world_frame_name: self.is_path_segment_collision_free_via_service(path_q))
                    if self._collision_cli is not None else None
                ),
                rtb_model=RTB_MODEL,
                base_link_name=BASE_LINK_NAME,
                world_frame=WORLD_FRAME,
            )
            
            self.planning_attempts += 1
            self.get_logger().info(f"Planning attempt {self.planning_attempts}/{self.max_planning_attempts}")
            
            computed_path = self.rrt_star.plan()
            if computed_path is None:
                if self.start_goal_collision == "start":
                    self.get_logger().warning("Could not find a path. Start configuration in collision.")
                elif self.start_goal_collision == "goal":
                    self.get_logger().warning("Could not find a path. Goal configuration in collision.")
                elif self.proximity_alert:
                    self.get_logger().warning(f"Could not find a path. Obstacles too close (attempt {self.planning_attempts}/{self.max_planning_attempts})")
                else:
                    self.get_logger().warning(f"Could not find a path (attempt {self.planning_attempts}/{self.max_planning_attempts})")
                
                # Check if we've exceeded maximum attempts
                if self.planning_attempts >= self.max_planning_attempts:
                    self.planning_failed = True
                    if self.max_planning_attempts > 1:
                        self.get_logger().warning(f"Planning failed after {self.max_planning_attempts} attempts. Stopping further planning.")
                    else:
                        self.get_logger().warning("Planning failed on single attempt. Stopping further planning.")
                    if self.stop_on_failure:
                        # Use a timer to shutdown gracefully
                        self.create_timer(1.0, lambda: rclpy.shutdown())
                        
            else:
                self.get_logger().info(f"Found a path on attempt {self.planning_attempts}!")
                # Reset attempt counter on success
                self.planning_attempts = 0
                
                # publish path
                plan_msg = JointSpacePath()
                plan_msg.joint_names = JOINT_NAMES
                for q in computed_path:
                    waypoint = JointWaypoint()
                    self.get_logger().info(f"\033[92mWaypoint: {np.round(q, 2)}\033[0m")
                    waypoint.positions = np.array(q).tolist()
                    plan_msg.waypoints.append(waypoint)
                self.get_logger().info(f"Num waypoints: {len(plan_msg.waypoints)}")
                path_cost = RRTStar.compute_path_cost(computed_path)
                self.get_logger().info(f"Path cost: {path_cost:.3f}")
                self.plan_pub.publish(plan_msg)
                if self.computed_path is None:
                    self.computed_path = computed_path
                self.computed_path = computed_path # store for vizualization
                if self.stop_if_plan_found:
                    self.planning_done = True # mark planning done so we stop trying

    def check_collision_service(self, q: np.ndarray, timeout_sec: float = 1.0) -> bool:
        """Return True if q is in collision (conservative on timeout/failure)."""
        if self._collision_cli is None:
            # No service, assume not in collision and let other checker decide
            return False
        try:
            req = CheckCollision.Request()
            req.joint_positions = np.asarray(q, dtype=float).tolist()
            future = self._collision_cli.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
            if not future.done() or future.result() is None:
                if self.verbose:
                    self.get_logger().warn('Collision service timeout/failure; treating as collision')
                return True
            return bool(future.result().in_collision)
        except Exception as e:
            self.get_logger().warn(f"Collision service error: {e}")
            return True

    def is_path_segment_collision_free_via_service(self, path_q: list[np.ndarray], timeout_sec: float = 0.5) -> bool:
        for q in path_q:
            if self.check_collision_service(q, timeout_sec=timeout_sec):
                return False
        return True

    def robot_geom_cb(self, msg:RigidBodyGeom):
        if self.robot_geom is None:
            self.robot_geom = msg
            if self.verbose:
                self.get_logger().info("Received robot geometry.")
                self.get_logger().info(f"Link names: {self.robot_geom.link_names}")
                self.get_logger().info(f"Number of link geometries: {len(self.robot_geom.link_geometries)}")
                self.get_logger().info(f"Number of link poses: {len(self.robot_geom.link_poses)}")

    def get_scene_obs_cb(self, msg:SceneObstacles):
        self.obstacle_geom = msg
        if self.verbose:
            self.get_logger().info(f"Received {len(msg.scene_obstacles)} obstacles.")

    def publish_ee_path_markers_cb(self):
        if self.computed_path is not None and self.show_ee_path:
            self.publish_ee_path(self.computed_path)

    def publish_ee_path(self, computed_path):
        """
        Publish the end effector pose for each waypoint in the path.
        """
        markers, last_pose_stamped = build_ee_path_line_strip(
            computed_path=computed_path,
            rtb_model=RTB_MODEL,
            base_link_name=BASE_LINK_NAME,
            node=self,
        )
        if markers is not None:
            self.ee_path_marker_pub.publish(markers)
        if last_pose_stamped is not None:
            self.ee_path_pub.publish(last_pose_stamped)

    def publish_jsp_path_markers_cb(self):
        if self.computed_path is not None and self.show_jsp_waypoints:
            self.publish_jsp_path_markers(self.computed_path)

    def publish_jsp_path_markers(self, computed_path):
        """
        path: list of numpy arrays (joint configurations)
        For each waypoint, compute FK for all links and publish a set of markers.
        """
        if self.robot_geom is None:
            self.get_logger().warning("No robot geometry available; skipping JSP marker publish.")
            return

        markers = build_jsp_waypoint_markers(
            computed_path=computed_path,
            robot_geom=self.robot_geom,
            rtb_model=RTB_MODEL,
            base_link_name=BASE_LINK_NAME,
            joint_limits=JOINT_LIMITS,
            node=self,
        )
        if markers is not None:
            self.jsp_path_marker_pub.publish(markers)
            if self.verbose:
                self.get_logger().info(
                    f"Published MarkerArray with {len(markers.markers)} markers for {len(computed_path)} waypoints."
                )

def main(args=None):
    rclpy.init(args=args)
    smpb_jsp_pl_node = SamplingBasedJSPlanner()
    try:
        rclpy.spin(smpb_jsp_pl_node)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass
    finally:
        smpb_jsp_pl_node.destroy_node()
        rclpy.shutdown()

if __name__=="__main__":
    main()
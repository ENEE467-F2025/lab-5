#!/usr/bin/env python3

# Lab 5: Collision-Free Kinematic Motion Planning in ROS 2 - Part I
# Copyright (C) 2025 Clinton Enwerem

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
ROS 2 node to find a collision-free trajectory by sampling from the attendant collision-free subspace (in joint space).

Only the RRTStar (RRTstarkConfigDefault) algorithm is implemented in this script.
Support for RRT (RRTkConfigDefault) is planned.

Usage: ros2 run robot_3r_planners sampling_based_planner.py --ros-args -p goal_config:="[1.5093, 0.6072, 1.4052]" -p check_collision:=True

Author: Clinton Enwerem.
Developed for the course ENEE467: Robotics Projects Laboratory, Fall 2025, University of Maryland, College Park, MD.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Pose, Point
from rclpy.duration import Duration
import numpy as np
import math, random
from pymoveit2.robots import robot_3r as robot
from robot_3r_interfaces.msg import JointWaypoint, JointSpacePath
from typing import List, Dict
import transforms3d as t3d
import spatialmath as sm
from numpy.typing import NDArray
from rcl_interfaces.msg import ParameterDescriptor

# obstacles and collision detection
import fcl
from robot_3r_interfaces.msg import SceneObstacles, RigidBodyGeom
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker, MarkerArray
from robot_3r_planners.utils.collision_utils import obstacle_to_fclobj, create_fcl_object

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
    class RRTStar:
        """
        Class for RRT/RRT* path planning.

        References:
        [1]. Paper: https://www.cs.cmu.edu/afs/cs/academic/class/15494-s12/readings/kuffner_icra2000.pdf
        [2]. Code: RRTStar class available at: https://github.com/AtsushiSakai/PythonRobotics/blob/master/ArmNavigation/rrt_star_seven_joint_arm_control/rrt_star_seven_joint_arm_control.py
        """
        # represents a node on the random tree
        class RRTStarConfigNode:
            def __init__(self, q):
                self.q = np.array(q, dtype=float)
                self.parent = None
                self.cost = 0.0   # cost(q)
                self.path_q = []  # list of intermediate configs

        # initialize RRTStar with params; these are set when the parent ROS2 Node is initialized
        def __init__(self, outer_instance: Node, start, goal, robot_geom, expand_dist, obstacle_geom, rand_area,
                    path_resolution, max_iter, connect_circle_dist, goal_sample_rate, check_collision_param=True
                     ):
            """
            start: start configuration [q1,...,qn]
            goal: goal configuration [q1,...,qn]
            obstacle_geom: obstacle geometries of type SolidPrimitive
            robot_geom: robot geometry of type SolidPrimitive
            rand_area: Random sampling area [min,max] in radians
            """
            self.outer = outer_instance
            self.start = self.RRTStarConfigNode(start)
            self.start.path_q = [start]  # Set path_q for collision checking
            self.end = self.RRTStarConfigNode(goal)
            self.end.path_q = [goal]  # Set path_q for collision checking
            self.dimension = len(start)
            self.robot_geom = robot_geom
            self.min_rand = rand_area[0]
            self.max_rand = rand_area[1]
            self.expand_dist = expand_dist
            self.path_resolution = path_resolution
            self.goal_sample_rate = goal_sample_rate
            self.max_iter = max_iter
            self.obstacle_geom = obstacle_geom
            self.connect_circle_dist = connect_circle_dist
            self.config_tree = []
            self.check_collision_param = check_collision_param
        
        def sample_free(self):
            """
            We do not explicit construct the C-space in this routine due to the intractability of such a computation.
            Instead we incrementally explore the C-space by search.
            See Section 7.1 of Robot Dynamics and Control by M. Spong et. al.
            We bias the sampling towards the goal configuration based on the goal_sample_rate.
            """
            try:
                if self.outer.pl_alg=="rrt_star":
                    if self.outer.use_goal_biased_sampling:
                        if np.random.rand() > self.goal_sample_rate:
                            samp_q = []
                            for joint_limit in self.outer.joint_limits:
                                q_curr_joint = np.random.uniform(joint_limit[0], joint_limit[1])
                                samp_q.append(q_curr_joint)
                            return self.RRTStarConfigNode(np.array(samp_q))
                        else: # goal point sampling
                            goal = np.array(self.end.q)
                            noise = np.random.normal(scale=self.outer.goal_noise_sigma, size=goal.shape)
                            goal_cand = np.clip(goal + noise, [jl[0] for jl in self.outer.joint_limits], [jl[1] for jl in self.outer.joint_limits])
                            return self.RRTStarConfigNode(goal_cand)
                    else:
                        samp_q = []
                        for joint_limit in self.outer.joint_limits:
                            q_curr_joint = np.random.uniform(joint_limit[0], joint_limit[1])
                            samp_q.append(q_curr_joint)
                        return self.RRTStarConfigNode(np.array(samp_q))
                elif self.outer.pl_alg=="rrt":
                    raise NotImplementedError("RRT algorithm not yet implemented.")
                else: 
                    self.outer.get_logger().warning("Only the RRTStar sampling-based planning algorithm is supported!")
            except Exception as e:
                self.outer.get_logger().error(f"{e}")

        def calc_dist_to_goal(self, q):
            distance = np.linalg.norm(np.array(q) - np.array(self.end.q))
            return distance
    
        def get_nearby_neighbors(self, x_new:RRTStarConfigNode) -> List[RRTStarConfigNode]:
            """
            Get nearby neighbors of a new node within a given radius (connect_circle_dist)
            """
            # ensure asymptotic optimality
            assert self.connect_circle_dist > 2*(1 + (1/self.dimension))**(1/self.dimension), "Invalid connect_circle_dist"
            curr_num_nodes = len(self.config_tree)
            # fallback for tiny trees
            if curr_num_nodes <= 1:
                near_radius = self.expand_dist
            else:
                near_radius = self.connect_circle_dist * (math.log(curr_num_nodes) / curr_num_nodes)**(1.0/self.dimension) 
                if self.expand_dist:
                    near_radius = min(near_radius, self.expand_dist)
            dists = [np.sum((np.array(nd.q)-np.array(x_new.q)) ** 2) for nd in self.config_tree]
            near_inds = [idx for idx, dist in enumerate(dists) if dist <= near_radius ** 2]
            return near_inds

        def choose_best_parent(self, new_node:RRTStarConfigNode, near_inds:List[int]) -> RRTStarConfigNode:
            # Choose the parent of the new node from the nearby neighbors with the lowest cost
            # if no valid neighbors are found, fallback to nearest node 
            if not near_inds:
                nearest_idx = self.get_nearest_node_index(self.config_tree, new_node)
                cand_new_node = self.steer(self.config_tree[nearest_idx], new_node)
                if cand_new_node and self.collision_free(cand_new_node, self.robot_geom, self.obstacle_geom, self.check_collision_param):
                    cand_new_node.parent = self.config_tree[nearest_idx]
                    cand_new_node.cost = self.config_tree[nearest_idx].cost + self.calc_new_cost(self.config_tree[nearest_idx], cand_new_node)
                    return cand_new_node
                return None
            costs = []
            for i in near_inds:
                near_node = self.config_tree[i]
                cand_new_node = self.steer(near_node, new_node)
                if cand_new_node and self.collision_free(cand_new_node, self.robot_geom, self.obstacle_geom, self.check_collision_param):
                    costs.append(self.calc_new_cost(near_node, cand_new_node))
                else:
                    costs.append(float("inf")) # collision incurs a high cost
            # Find the neighbor with the lowest cost
            min_cost = min(costs)
            if min_cost == float("inf"):
                self.outer.get_logger().info("Could not find a collision-free path to the new node from any of its neighbors.")
                return None
            min_ind = near_inds[costs.index(min_cost)]
            new_node = self.steer(self.config_tree[min_ind], new_node) # try steering from the best parent
            if not new_node:
                return None
            new_node.parent = self.config_tree[min_ind]
            new_node.cost = min_cost
            return new_node

        def steer(self, x_nearest:RRTStarConfigNode, x_random:RRTStarConfigNode):
            # steer from x_nearest towards x_random
            extend_length = self.expand_dist
            # ensure numpy arrays
            start = np.array(x_nearest.q, dtype=float)
            goal = np.array(x_random.q, dtype=float)

            new_node = self.RRTStarConfigNode(start.copy()) # initialize new node as start node

            d, _, _ = self.calc_distance_and_angle(x_nearest, x_random)   # distance between start and goal
            new_node.path_q = [start]

            if d == 0.0:
                return None
            # adjust tree growth factor
            if extend_length > d:
                extend_length = d
            n_expand = max(1, math.floor(extend_length / self.path_resolution)) # number of grids corresponding to the expansion length

            vec_diff = goal - start
            unit_vec = vec_diff / np.linalg.norm(vec_diff)

            for _ in range(n_expand): # expand by one cell storing the intermediate configurations
                new_node.q = new_node.q + unit_vec * self.path_resolution # update new node
                new_node.path_q.append(new_node.q.copy())  # store path taken to reach this node

            # if close enough to the random sample, snap to it
            d_after, _, _ = self.calc_distance_and_angle(new_node, x_random)
            if d_after <= self.path_resolution:
                new_node.q = goal.copy()
                new_node.path_q.append(goal)

            new_node.parent = x_nearest
            return new_node

        def rewire(self, new_node:RRTStarConfigNode, near_inds:List[int]):
            """
            Find a lower cost path by rereassigning parents along tree.
            """
            for i in near_inds:
                near_node = self.config_tree[i]
                cand_new_node = self.steer(new_node, near_node) # try to get closer to the near_node on the current tree
                if not cand_new_node:
                    continue
                no_collision = self.collision_free(cand_new_node, self.robot_geom, self.obstacle_geom, self.check_collision_param)
                # if the potentially-new path (terminating at near_node) has a lower cost, rewire the tree
                cost_is_improved =  cand_new_node.cost + self.calc_new_cost(cand_new_node, near_node) < near_node.cost
                if no_collision and cost_is_improved:
                    self.config_tree[i] = cand_new_node
                    self.propagate_cost_to_leaves(new_node)
        
        def find_best_goal_node(self):
            # Find the node in the current tree that is closest to the goal node
            dist_to_goal_list = [self.calc_dist_to_goal(nd.q)
                             for nd in self.config_tree]
            goal_inds = [dist_to_goal_list.index(i)
                        for i in dist_to_goal_list if i <= self.expand_dist]

            collision_free_goal_inds = []
            for goal_ind in goal_inds:
                cand_new_node = self.steer(self.config_tree[goal_ind], self.end)
                if cand_new_node and self.collision_free(cand_new_node,
                                        self.robot_geom,
                                        self.obstacle_geom,
                                        self.check_collision_param):
                    collision_free_goal_inds.append(goal_ind)

            if not collision_free_goal_inds:
                return None

            min_cost = min([self.config_tree[i].cost for i in collision_free_goal_inds])
            for i in collision_free_goal_inds:
                if self.config_tree[i].cost == min_cost:
                    return i

            return None

        def propagate_cost_to_leaves(self, parent_node:RRTStarConfigNode):
            """
            Recursively propagate the cost from the parent node to all its child nodes.
            """
            for node in self.config_tree:
                if node.parent == parent_node:
                    node.cost = self.calc_new_cost(parent_node, node)
                    self.propagate_cost_to_leaves(node)

        def calc_new_cost(self, from_node, to_node):
            # Get cumulative Euclidean distance cost to move from the from_node to the to_node
            d, _, _ = self.calc_distance_and_angle(from_node, to_node)
            return from_node.cost + d

        def generate_final_course(self, goal_ind):
            computed_path = [self.end.q]
            node = self.config_tree[goal_ind]
            while node.parent is not None:
                computed_path.append(node.q)
                node = node.parent
            computed_path.append(node.q)
            computed_path.reverse()
            return computed_path
    
        def plan(self):
            """
            Main module for planning
            """
            # check collision for start and end
            start_collision_free = self.collision_free(self.start, self.robot_geom, self.obstacle_geom, self.check_collision_param)
            
            # Create proper goal node for collision checking
            goal_node = self.RRTStarConfigNode(self.outer.goal_config)
            goal_node.path_q = [self.outer.goal_config]  # Set path_q to contain the goal configuration
            
            end_collision_free = self.collision_free(goal_node, self.robot_geom, self.obstacle_geom, self.check_collision_param)
            if start_collision_free is not None and end_collision_free is not None: 
                if not start_collision_free or not end_collision_free:
                    if not start_collision_free:
                        self.outer.get_logger().warning(f"Start configuration {np.round(self.start.q, 3)} is in collision!")
                        self.outer.start_goal_collision = "start"
                    if not end_collision_free:
                        self.outer.get_logger().warning(f"Goal configuration {np.round(self.outer.goal_config, 3)} is in collision!")
                        self.outer.start_goal_collision = "goal"
                    return None
            
            self.config_tree = [self.start]
            for i in range(self.max_iter):
                if self.outer.verbose:
                    self.outer.get_logger().info(f"Iteration {i}, number of nodes in tree: {len(self.config_tree)}")
                rnd_node = self.sample_free()
                nearest_ind = self.get_nearest_node_index(self.config_tree, rnd_node)
                new_node = self.steer(self.config_tree[nearest_ind], rnd_node)
                if new_node and self.collision_free(new_node, self.robot_geom, self.obstacle_geom, self.check_collision_param):
                    near_inds = self.get_nearby_neighbors(new_node)
                    new_node = self.choose_best_parent(new_node, near_inds)
                    if new_node:
                        self.config_tree.append(new_node)
                        self.rewire(new_node, near_inds)
                if (not self.outer.rrts_search_until_max_iter) and new_node:
                    last_index = self.find_best_goal_node()
                    if last_index is not None:
                        return self.generate_final_course(last_index)

            self.outer.get_logger().info("Reached max iteration of {}".format(self.max_iter))

            last_index = self.find_best_goal_node()
            if last_index is not None:
                return self.generate_final_course(last_index)

            return None

        def collision_free(self,
                    candidate_node: RRTStarConfigNode,
                    robot_geom: RigidBodyGeom,
                    obstacles: SceneObstacles,
                    check_collision: bool = True) -> bool:
            """
            Check if a candidate joint config is collision-free.

            Args:
                q_candidate: ndarray, joint angles
                robot_geom: object with .link_names, .link_geometries, .link_poses (local geom offset)
                obstacles: list of FCL collision objects 

            Returns:
                bool: True if collision-free, False otherwise
            """
            if not check_collision:
                self.outer.get_logger().warning("check_collision set to False. Path will be invalid.")
                return False

            # Make sure path_q is a dense interpolation of configs between parent and node
            nodes_to_check = []
            try:
                nodes_to_check = [SamplingBasedJSPlanner.RRTStar.RRTStarConfigNode(node) for node in candidate_node.path_q]
            except Exception as e:
                self.outer.get_logger().error(f"Error creating nodes to check for collision: {e}")
                return False

            # Convert obstacles once
            obs_fcl_objects = list(obstacle_to_fclobj(obstacles=obstacles))

            # For each intermediate configuration, check all links immediately
            collision_detected = False
            for c_node in nodes_to_check:
                q = c_node.q
                RTB_MODEL.q = q  # set joint angles

                # For each link compute FK and test collision
                for i, link in enumerate(RTB_MODEL.links):
                    link_name = link.name
                    # skip base/world pseudo-links
                    if link_name in [BASE_LINK_NAME, 'link0', WORLD_FRAME]:
                        continue

                    # map link name to robot_geom index safely
                    try:
                        try:
                            idx = robot_geom.link_names.index(link_name)
                        except ValueError:
                            # link not in provided robot_geom
                            continue

                        # Compute transform for link in world frame
                        T_fk_se3 = RTB_MODEL.fkine(q, end=link.name, include_base=True)  # SE3

                        # Build FCL collision geometry for the link using robot_geom entries
                        link_geometry = robot_geom.link_geometries[idx]
                        if link_geometry.type == SolidPrimitive.BOX:
                            x, y, z = link_geometry.dimensions
                            geom = fcl.Box(x, y, z)
                        elif link_geometry.type == SolidPrimitive.SPHERE:
                            geom = fcl.Sphere(link_geometry.dimensions[0])
                        elif link_geometry.type == SolidPrimitive.CYLINDER:
                            radius = link_geometry.dimensions[0]
                            height = link_geometry.dimensions[1]
                            geom = fcl.Cylinder(radius, height)
                        else:
                            continue

                        # Create FCL collision object using exact transform
                        rob_obj = create_fcl_object(robot.se3_to_pose_stamped(T_fk_se3, self.outer), geom)

                        # Check collision against every obstacle
                        for obs_obj in obs_fcl_objects:
                            if self.outer.collision_checker == 'bvol':
                                creq = fcl.CollisionRequest()
                                creq.enable_contact = True  # enable contact detection
                                # creq.num_max_contacts = 1   # we only need to know if collision exists
                                cres = fcl.CollisionResult()
                                
                                ret = fcl.collide(rob_obj, obs_obj, creq, cres)
                                
                                # FCL returns True if a collision detected
                                if cres.is_collision or ret > 0:
                                    if self.outer.verbose:
                                        self.outer.get_logger().info(f"Bvol collision detected on {link_name} at q={np.round(q,3)}")
                                    collision_detected = True
                                    break  # Break out of obstacle loop

                            # for proximity checks
                            if self.outer.collision_checker == 'proximity':
                                dreq = fcl.DistanceRequest(enable_signed_distance=True)
                                dres = fcl.DistanceResult()
                                _ = fcl.distance(rob_obj, obs_obj, dreq, dres)
                                min_dist = dres.min_distance
                                if min_dist < 0:
                                    if self.outer.verbose:
                                        self.outer.get_logger().info(f"\033[91mProximity collision detected on {link_name} at q={np.round(q,3)}\033[0m")
                                    # Set proximity alert and return False
                                    self.outer.proximity_alert = True
                                    collision_detected = True
                                    break  # Break out of obstacle loop
                                if min_dist < self.outer.min_obs_dist:
                                    self.outer.proximity_alert = True
                                    # Print in red if q is close to the goal configuration
                                    target_q = self.outer.goal_config
                                    start_q = self.start.q
                                    if np.linalg.norm(q - start_q) < self.outer.min_obs_dist:
                                        collision_detected = True
                                        break  # Break out of obstacle loop
                                    elif np.linalg.norm(q - target_q) < self.outer.min_obs_dist:
                                        collision_detected = True
                                        break  # Break out of obstacle loop
                                    else:
                                        if self.outer.verbose:
                                            self.outer.get_logger().info(f"Proximity collision detected on {link_name} at q={np.round(q,3)}")
                                    collision_detected = True
                                    break  # Break out of obstacle loop

                        # Break out of link loop if collision detected
                        if collision_detected:
                            break

                    except Exception as e:
                        self.outer.get_logger().error(f"Error checking link {link_name}: {e}")
                        collision_detected = True
                        break

                # Break out of node loop if collision detected
                if collision_detected:
                    break

            # Return False if any collision was detected
            if collision_detected:
                return False

            # If we reached here, all intermediate configs were free
            return True

        @staticmethod
        def get_nearest_node_index(config_tree, rnd_node):
            dlist = [np.sum((np.array(node.q) - np.array(rnd_node.q))**2)
                    for node in config_tree]
            min_idx = dlist.index(min(dlist))
            return min_idx

        @staticmethod
        def calc_distance_and_angle(from_node:RRTStarConfigNode, to_node:RRTStarConfigNode):
            dx = to_node.q[0] - from_node.q[0]
            dy = to_node.q[1] - from_node.q[1]
            dz = to_node.q[2] - from_node.q[2]
            d = np.sqrt(np.sum((np.array(to_node.q) - np.array(from_node.q))**2))
            phi = math.atan2(dy, dx) # vector direction in XY plane; azimuth
            theta = math.atan2(dz, math.hypot(dx, dy)) # elevation angle from Z axis
            return d, phi, theta
        
        @staticmethod
        def compute_path_cost(path:List[NDArray]) -> float:
            total_cost = 0.0
            for i in range(1, len(path)):
                total_cost += np.linalg.norm(path[i] - path[i-1])
            return total_cost
        
    # init ROS2 Node
    def __init__(self, node_name:str="sampling_based_planner", queue_size=10):
        self.node_name = node_name
        self.queue_size = queue_size
        super().__init__(self.node_name)
        self.get_logger().info("")
        self.get_logger().info(f"\n--------------------------------------------------\nInitializing {self.node_name} node...\n--------------------------------------------------")

        ######################################################################################################################
        ######################################################################################################################
        # declare and get params
        ######################################################################################################################
        ######################################################################################################################
        self.declare_parameter(
            "planning_algorithm",
            value="rrt_star",
            descriptor=ParameterDescriptor(description="Planning algorithm to use; only 'rrt_star' is supported for now, 'rrt' is planned.")
        )
        self.declare_parameter(
            "stop_if_plan_found",
            value=True,
            descriptor=ParameterDescriptor(description="Stop planning node if a valid plan is found.")
        )
        self.declare_parameter(
            "verbose",
            value=False,
            descriptor=ParameterDescriptor(description="Whether to print detailed info during planning.")
        )
        self.declare_parameter(
            "moveit_config_name",
            value="robot_3r_moveit_config",
            descriptor=ParameterDescriptor(description="Name of MoveIt config package.")
        )
        self.declare_parameter(
            "def_group_state",
            value="arm_ready",
            descriptor=ParameterDescriptor(description="Named configuration from the SRDF file.")
        )

        # RRTStar-specific params
        self.declare_parameter(
            "rrts_expand_dist",
            value=0.3,
            descriptor=ParameterDescriptor(
            description=" Maximum distance (in radians) between nodes in the RRT* tree.\n"
                    "                Controls how far the tree grows in a single iteration.\n"
                    "                Recommended value: 0.1 - 0.5 radians.\n"
                    "                Corresponds to the maximum_waypoint_distance parameter in OMPL/MoveIt."
            )
        )
        self.declare_parameter(
            "rrts_path_resolution",
            value=0.1,
            descriptor=ParameterDescriptor(description="Radians per cell side length; cells are square.")
        )
        self.declare_parameter(
            "rrts_max_iter",
            value=300,
            descriptor=ParameterDescriptor(description="Allowed planning time used by all OMPL planner plugins.")
        )
        self.declare_parameter(
            "rrts_connect_circle_dist",
            value=20,
            descriptor=ParameterDescriptor(description="Connection distance for RRT* (in number of cells, i.e., the maximum number of cells within which we consider nodes within the circle to be neighbors of the center node).")
        )
        self.declare_parameter(
            "rrts_search_until_max_iter",
            value=False,
            descriptor=ParameterDescriptor(description="Continue searching until max_iter is reached; determines if path is improved or not.")
        )
        self.declare_parameter(
            "rrts_goal_sample_rate",
            value=0.3,
            descriptor=ParameterDescriptor(description="Goal bias in OMPL RRTstarkConfigDefault plugin; expressed as value/100 in OMPL (range: (0, 1)).")
        )
        self.declare_parameter(
            "use_goal_biased_sampling",
            value=False,
            descriptor=ParameterDescriptor(description="Whether to use goal bias sampling; only applicable to RRT and RRT*.")
        )
        self.declare_parameter(
            "goal_noise_sigma",
            value=0.05,
            descriptor=ParameterDescriptor(description="Standard deviation of Gaussian noise added to goal during goal-biased sampling in radians.")
        )

        # collision checking
        self.declare_parameter(
            "min_obs_dist",
            value=0.1,
            descriptor=ParameterDescriptor(description="Minimum obstacle distance threshold in meters.")
        )
        self.declare_parameter(
            "collision_checker",
            value='proximity',
            descriptor=ParameterDescriptor(description="Type of collision checker to use; options are 'bvol' and 'proximity'.")
        )

        # random seed
        self.declare_parameter(
            "random_seed",
            value=42,
            descriptor=ParameterDescriptor(description="Random seed for reproducible results. Do not change this.")
        )

        # viz params
        self.declare_parameter(
            "show_jsp_waypoints",
            value=True,
            descriptor=ParameterDescriptor(description="Whether to visualize joint space waypoints.")
        )
        self.declare_parameter(
            "show_ee_path",
            value=False,
            descriptor=ParameterDescriptor(description="Whether to visualize end effector path.")
        )
        self.declare_parameter(
            "check_collision",
            value=True,
            descriptor=ParameterDescriptor(description="Whether to check for collisions.")
        )

        # metrics
        self.declare_parameter(
            "print_metrics",
            value=True,
            descriptor=ParameterDescriptor(description="Whether to print planning metrics.")
        )
        
        # planning failure handling
        self.declare_parameter(
            "max_planning_attempts",
            value=1,
            descriptor=ParameterDescriptor(description="Maximum number of planning attempts before giving up; 1 = single attempt.")
        )
        self.declare_parameter(
            "stop_on_failure",
            value=True,
            descriptor=ParameterDescriptor(description="Stop node when planning fails permanently.")
        )

        self.declare_parameter(
            "proximity_alert",
            value=False,
            descriptor=ParameterDescriptor(description="Whether to enable proximity alerts.")
        )
        self.declare_parameter(
            "start_goal_collision",
            value='',
            descriptor=ParameterDescriptor(description="Whether start or goal is in collision.")
        )

        self.verbose = self.get_parameter("verbose").get_parameter_value().bool_value
        self.print_metrics = self.get_parameter("print_metrics").get_parameter_value().bool_value
        self.def_group_state = self.get_parameter("def_group_state").get_parameter_value().string_value
        self.pl_alg = self.get_parameter("planning_algorithm").get_parameter_value().string_value
        self.moveit_config_name = self.get_parameter("moveit_config_name").get_parameter_value().string_value
        self.joint_limits = JOINT_LIMITS
        self.rrts_expand_dist = self.get_parameter("rrts_expand_dist").get_parameter_value().double_value
        self.rrts_path_resolution = self.get_parameter("rrts_path_resolution").get_parameter_value().double_value
        self.rrts_max_iter = self.get_parameter("rrts_max_iter").get_parameter_value().integer_value
        self.rrts_connect_circle_dist = self.get_parameter("rrts_connect_circle_dist").get_parameter_value().integer_value
        self.rrts_goal_sample_rate = self.get_parameter("rrts_goal_sample_rate").get_parameter_value().double_value
        self.rrts_search_until_max_iter = self.get_parameter("rrts_search_until_max_iter").get_parameter_value().bool_value
        self.rrts_expand_dist = self.get_parameter("rrts_expand_dist").get_parameter_value().double_value
        self.show_jsp_waypoints = self.get_parameter("show_jsp_waypoints").get_parameter_value().bool_value
        self.show_ee_path = self.get_parameter("show_ee_path").get_parameter_value().bool_value
        self.check_collision = self.get_parameter("check_collision").get_parameter_value().bool_value
        self.stop_if_plan_found = self.get_parameter("stop_if_plan_found").get_parameter_value().bool_value
        self.min_obs_dist = self.get_parameter("min_obs_dist").get_parameter_value().double_value
        self.collision_checker = self.get_parameter("collision_checker").get_parameter_value().string_value
        self.max_planning_attempts = self.get_parameter("max_planning_attempts").get_parameter_value().integer_value
        self.stop_on_failure = self.get_parameter("stop_on_failure").get_parameter_value().bool_value
        self.proximity_alert = self.get_parameter("proximity_alert").get_parameter_value().bool_value
        self.start_goal_collision = self.get_parameter("start_goal_collision").get_parameter_value().string_value
        self.use_goal_biased_sampling = self.get_parameter("use_goal_biased_sampling").get_parameter_value().bool_value
        self.goal_noise_sigma = self.get_parameter("goal_noise_sigma").get_parameter_value().double_value
        
        # Set random seed from parameter
        self.random_seed = self.get_parameter("random_seed").get_parameter_value().integer_value
        np.random.seed(self.random_seed)
        random.seed(self.random_seed)
        if self.verbose:
            self.get_logger().info(f"Random seed set to: {self.random_seed}")

        # Planning state tracking
        if self.stop_if_plan_found:
            self.planning_done = False
        self.planning_attempts = 0
        self.planning_failed = False

        if GROUP_STATES is not None:
            self.declare_parameter("goal_config", value=GROUP_STATES[self.def_group_state]) # we use a named configuration from the 
                                                                                            # SRDF file
        else:
            self.declare_parameter("goal_config", value=[1.5093, 0.6072, 1.4052])
        self.goal_config = self.get_parameter("goal_config").get_parameter_value().double_array_value
        if self.verbose:
            self.get_logger().info(f"Planning to goal configuration: {self.goal_config} using {self.pl_alg} algorithm.")
        ######################################################################################################################
        ######################################################################################################################
        # End declare and get params
        ######################################################################################################################
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
            self.rrt_star = self.RRTStar(
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
                check_collision_param=self.check_collision
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
                path_cost = self.rrt_star.compute_path_cost(computed_path)
                self.get_logger().info(f"Path cost: {path_cost:.3f}")
                self.plan_pub.publish(plan_msg)
                if self.computed_path is None:
                    self.computed_path = computed_path
                self.computed_path = computed_path # store for vizualization
                if self.stop_if_plan_found:
                    self.planning_done = True # mark planning done so we stop trying

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
        # collect poses using full transform matrix 
        points = []
        last_pose_stamped = None
        for q in computed_path:
            # RTB SE3 object = 4x4 matrix; rtvec fails for some reason
            try:
                RTB_MODEL.q = q # set joint angle
                T = RTB_MODEL.fkine(q).A
            except Exception:
                continue
            trans = T[:3, 3].tolist()
            R = T[:3, :3]
            quat = t3d.quaternions.mat2quat(R)  # returns (w, x, y, z)
            ps = PoseStamped()
            ps.header.frame_id = BASE_LINK_NAME
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.pose.position.x = trans[0]
            ps.pose.position.y = trans[1]
            ps.pose.position.z = trans[2]
            ps.pose.orientation.w = float(quat[0])
            ps.pose.orientation.x = float(quat[1])
            ps.pose.orientation.y = float(quat[2])
            ps.pose.orientation.z = float(quat[3])
            last_pose_stamped = ps

            # marker point
            points.append(Point(x=trans[0], y=trans[1], z=trans[2]))

        # use a continuous LINE_STRIP marker for the entire path
        if points:
            marker = Marker()
            marker.header.frame_id = BASE_LINK_NAME
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "ee_path"
            marker.id = 0
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.015  
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.points = points
            marker.lifetime = Duration(seconds=0).to_msg()
            self.ee_path_marker_pub.publish(MarkerArray(markers=[marker]))

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

        # prepare marker array
        markers = MarkerArray()

        n_waypoints = len(computed_path)
        for idx, q in enumerate(computed_path):
            marker_id = 0
            try:
                # Clamp each joint value in q between its joint limits
                clamped_q = [
                    max(JOINT_LIMITS[i][0], min(q[i], JOINT_LIMITS[i][1]))
                    for i in range(len(q))
                ]
                # RTB_MODEL.q = clamped_q  # set joint angle
                rtb_link_poses: Dict[str, PoseStamped] = {}
                for i, link in enumerate(RTB_MODEL.links):
                    link_pose = RTB_MODEL.fkine(q,end=link.name, include_base=True)  # SE3 object
                    rtb_link_poses[link.name] = robot.se3_to_pose_stamped(link_pose, node_obj=self, frame_id=BASE_LINK_NAME)

            except Exception as e:
                self.get_logger().warning(f"FK failed for waypoint {idx}: {e}")
                continue

            # set color dict waypoints
            colors = {
                "base_link": (0.2, 0.2, 0.2),
                "link0": (0.0, 0.0, 0.0),
                "link1": (0.122, 0.467, 0.706),  
                "link2": (0.122, 0.467, 0.706),  
                "link3": (0.122, 0.467, 0.706)   
            }

            # scale offset
            scale_offset = 0.000
            for i, link_name in enumerate(self.robot_geom.link_names):
                try:
                    link_geom = self.robot_geom.link_geometries[i]
                    link_geom_orig = self.robot_geom.link_geom_origins[i]
                except IndexError:
                    continue

                # fill marker pose
                p = Pose()
                link_pose = rtb_link_poses[link_name].pose
                link_T = sm.SE3(
                    link_pose.position.x,
                    link_pose.position.y,
                    link_pose.position.z
                ) * sm.SE3.RPY(
                    *t3d.euler.quat2euler([
                        link_pose.orientation.w,
                        link_pose.orientation.x,
                        link_pose.orientation.y,
                        link_pose.orientation.z
                    ], axes="sxyz")
                )
                #  offset from URDF
                origin_T = sm.SE3(
                    link_geom_orig.position.x,
                    link_geom_orig.position.y,
                    link_geom_orig.position.z
                ) * sm.SE3.RPY(
                    *t3d.euler.quat2euler([
                        link_geom_orig.orientation.w,
                        link_geom_orig.orientation.x,
                        link_geom_orig.orientation.y,
                        link_geom_orig.orientation.z
                    ], axes="sxyz")
                )
                geom_T = link_T * origin_T
                ps = robot.se3_to_pose_stamped(geom_T, node_obj=self, frame_id=BASE_LINK_NAME)
                p.position = ps.pose.position
                p.orientation = ps.pose.orientation

                # create Marker
                m = Marker()
                m.header.frame_id = BASE_LINK_NAME
                m.header.stamp = self.get_clock().now().to_msg()
                m.ns = f"path_wp_{idx}"
                m.id = marker_id
                marker_id += 1
                # marker_id = idx * len(self.robot_geom.link_names) + i

                # map SolidPrimitive to Marker types
                if link_geom.type == SolidPrimitive.BOX:
                    m.type = Marker.CUBE
                    sx, sy, sz = link_geom.dimensions
                    m.scale.x = sx + scale_offset
                    m.scale.y = sy + scale_offset
                    m.scale.z = sz + scale_offset
                elif link_geom.type == SolidPrimitive.SPHERE:
                    m.type = Marker.SPHERE
                    rads = link_geom.dimensions[0]
                    m.scale.x = m.scale.y = m.scale.z = 2 * rads
                elif link_geom.type == SolidPrimitive.CYLINDER:
                    m.type = Marker.CYLINDER
                    radius = link_geom.dimensions[0]
                    length = link_geom.dimensions[1]
                    m.scale.x = m.scale.y = 2 * radius
                    m.scale.z = length
                else:
                    # small point fallback visualization
                    m.type = Marker.SPHERE
                    m.scale.x = m.scale.y = m.scale.z = 0.02
                m.pose = p

                # color
                if idx == 0 and link_name in ["link1", "link2", "link3"]:
                    # use all purple for start config
                    m.color.r = 0.61624
                    m.color.g = 0.53967
                    m.color.b = 0.99264
                    m.color.a = 1.0
                elif idx == n_waypoints - 1 and link_name in ["link1", "link2", "link3"]:
                    # use blue for goal config
                    m.color.r = 0.122
                    m.color.g = 0.467
                    m.color.b = 0.706
                    m.color.a = 1.0
                elif link_name in ["link2", "link3"]:
                    # for all other waypoints, use color gradient, alpha=0.1
                    m.color.r = 0.0
                    m.color.g = 0.0
                    m.color.b = 0.0
                    m.color.a = 0.16
                else:
                    m.color.r = colors[link_name][0]
                    m.color.g = colors[link_name][1]
                    m.color.b = colors[link_name][2]
                    m.color.a = 1.0

                m.lifetime = Duration(seconds=0).to_msg() # make markers persistent
                markers.markers.append(m)

        # publish once
        self.jsp_path_marker_pub.publish(markers)
        if self.verbose:
            self.get_logger().info(f"Published MarkerArray with {len(markers.markers)} markers for {n_waypoints} waypoints.")

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
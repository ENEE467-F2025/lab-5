from typing import List, Tuple
import math
import numpy as np
from numpy.typing import NDArray
from rclpy.node import Node
from robot_3r_interfaces.msg import RigidBodyGeom, SceneObstacles
from .collision_utils import is_path_segment_collision_free


def calc_distance_and_angle(from_q: np.ndarray, to_q: np.ndarray) -> Tuple[float, float, float]:
    """
    Compute Euclidean distance and azimuth/elevation angles between two joint-space vectors.

    Args:
        from_q: numpy array of shape (n,)
        to_q: numpy array of shape (n,)

    Returns:
        d: Euclidean distance in joint space
        phi: azimuth in the plane of the first two joints (if present)
        theta: elevation relative to the third joint (if present)
    """
    from_q = np.asarray(from_q, dtype=float)
    to_q = np.asarray(to_q, dtype=float)
    dx = (to_q[0] - from_q[0]) if from_q.size > 0 else 0.0
    dy = (to_q[1] - from_q[1]) if from_q.size > 1 else 0.0
    dz = (to_q[2] - from_q[2]) if from_q.size > 2 else 0.0
    d = float(np.linalg.norm(to_q - from_q))
    phi = float(np.arctan2(dy, dx))
    theta = float(np.arctan2(dz, np.hypot(dx, dy)))
    return d, phi, theta


def compute_path_cost(path: List[np.ndarray]) -> float:
    """
    Sum of Euclidean distances between successive joint-space waypoints.
    """
    total = 0.0
    if not path:
        return 0.0
    for i in range(1, len(path)):
        total += float(np.linalg.norm(np.asarray(path[i]) - np.asarray(path[i - 1])))
    return total


class RRTStar:
    """
    RRT* for joint-space planning.

    This class is planner-agnostic with respect to the robot model and frames;
    those are injected via the constructor (rtb_model, base_link_name, world_frame).
    """

    class RRTStarConfigNode:
        def __init__(self, q):
            self.q = np.array(q, dtype=float)
            self.parent = None
            self.cost = 0.0
            self.path_q = []  # list of intermediate configs

    def __init__(
        self,
        outer_instance: Node,
        start,
        goal,
        robot_geom: RigidBodyGeom,
        obstacle_geom: SceneObstacles,
        rand_area,
        expand_dist: float,
        path_resolution: float,
        max_iter: int,
        connect_circle_dist: float,
        goal_sample_rate: float,
        check_collision_param: bool = True,
        collision_fn=None,
        *,
        rtb_model=None,
        base_link_name: str = "base_link",
        world_frame: str = "world",
    ):
        self.outer = outer_instance
        self.start = self.RRTStarConfigNode(start)
        self.start.path_q = [start]
        self.end = self.RRTStarConfigNode(goal)
        self.end.path_q = [goal]
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
        self.config_tree: List[RRTStar.RRTStarConfigNode] = []
        self.check_collision_param = check_collision_param
        self.collision_fn = collision_fn
        # Injected robot data
        self.rtb_model = rtb_model
        self.base_link_name = base_link_name
        self.world_frame = world_frame

    def sample_free(self):
        try:
            if self.outer.pl_alg == "rrt_star":
                if self.outer.use_goal_biased_sampling:
                    if np.random.rand() > self.goal_sample_rate:
                        samp_q = []
                        for joint_limit in self.outer.joint_limits:
                            q_curr_joint = np.random.uniform(joint_limit[0], joint_limit[1])
                            samp_q.append(q_curr_joint)
                        return self.RRTStarConfigNode(np.array(samp_q))
                    else:
                        goal = np.array(self.end.q)
                        noise = np.random.normal(
                            scale=self.outer.goal_noise_sigma, size=goal.shape
                        )
                        goal_cand = np.clip(
                            goal + noise,
                            [jl[0] for jl in self.outer.joint_limits],
                            [jl[1] for jl in self.outer.joint_limits],
                        )
                        return self.RRTStarConfigNode(goal_cand)
                else:
                    samp_q = []
                    for joint_limit in self.outer.joint_limits:
                        q_curr_joint = np.random.uniform(joint_limit[0], joint_limit[1])
                        samp_q.append(q_curr_joint)
                    return self.RRTStarConfigNode(np.array(samp_q))
            elif self.outer.pl_alg == "rrt":
                raise NotImplementedError("RRT algorithm not yet implemented.")
            else:
                self.outer.get_logger().warning(
                    "Only the RRTStar sampling-based planning algorithm is supported!"
                )
        except Exception as e:
            self.outer.get_logger().error(f"{e}")

    def calc_dist_to_goal(self, q):
        distance = np.linalg.norm(np.array(q) - np.array(self.end.q))
        return distance

    def get_nearby_neighbors(self, x_new: "RRTStar.RRTStarConfigNode") -> List[int]:
        # ensure asymptotic optimality
        assert (
            self.connect_circle_dist > 2 * (1 + (1 / self.dimension)) ** (1 / self.dimension)
        ), "Invalid connect_circle_dist"
        curr_num_nodes = len(self.config_tree)
        if curr_num_nodes <= 1:
            near_radius = self.expand_dist
        else:
            near_radius = self.connect_circle_dist * (
                math.log(curr_num_nodes) / curr_num_nodes
            ) ** (1.0 / self.dimension)
            if self.expand_dist:
                near_radius = min(near_radius, self.expand_dist)
        dists = [
            np.sum((np.array(nd.q) - np.array(x_new.q)) ** 2) for nd in self.config_tree
        ]
        near_inds = [idx for idx, dist in enumerate(dists) if dist <= near_radius**2]
        return near_inds

    def choose_best_parent(
        self, new_node: "RRTStar.RRTStarConfigNode", near_inds: List[int]
    ) -> "RRTStar.RRTStarConfigNode" | None:
        if not near_inds:
            nearest_idx = self.get_nearest_node_index(self.config_tree, new_node)
            cand_new_node = self.steer(self.config_tree[nearest_idx], new_node)
            if cand_new_node and self.collision_free(
                cand_new_node, self.robot_geom, self.obstacle_geom, self.check_collision_param
            ):
                cand_new_node.parent = self.config_tree[nearest_idx]
                cand_new_node.cost = self.config_tree[nearest_idx].cost + self.calc_new_cost(
                    self.config_tree[nearest_idx], cand_new_node
                )
                return cand_new_node
            return None
        costs = []
        for i in near_inds:
            near_node = self.config_tree[i]
            cand_new_node = self.steer(near_node, new_node)
            if cand_new_node and self.collision_free(
                cand_new_node, self.robot_geom, self.obstacle_geom, self.check_collision_param
            ):
                costs.append(self.calc_new_cost(near_node, cand_new_node))
            else:
                costs.append(float("inf"))
        min_cost = min(costs)
        if min_cost == float("inf"):
            self.outer.get_logger().info(
                "Could not find a collision-free path to the new node from any of its neighbors."
            )
            return None
        min_ind = near_inds[costs.index(min_cost)]
        new_node = self.steer(self.config_tree[min_ind], new_node)
        if not new_node:
            return None
        new_node.parent = self.config_tree[min_ind]
        new_node.cost = min_cost
        return new_node

    def steer(self, x_nearest: "RRTStar.RRTStarConfigNode", x_random: "RRTStar.RRTStarConfigNode"):
        extend_length = self.expand_dist
        start = np.array(x_nearest.q, dtype=float)
        goal = np.array(x_random.q, dtype=float)
        new_node = self.RRTStarConfigNode(start.copy())
        d, _, _ = self.calc_distance_and_angle(x_nearest, x_random)
        new_node.path_q = [start]
        if d == 0.0:
            return None
        if extend_length > d:
            extend_length = d
        n_expand = max(1, math.floor(extend_length / self.path_resolution))
        vec_diff = goal - start
        unit_vec = vec_diff / np.linalg.norm(vec_diff)
        for _ in range(n_expand):
            new_node.q = new_node.q + unit_vec * self.path_resolution
            new_node.path_q.append(new_node.q.copy())
        d_after, _, _ = self.calc_distance_and_angle(new_node, x_random)
        if d_after <= self.path_resolution:
            new_node.q = goal.copy()
            new_node.path_q.append(goal)
        new_node.parent = x_nearest
        return new_node

    def rewire(self, new_node: "RRTStar.RRTStarConfigNode", near_inds: List[int]):
        for i in near_inds:
            near_node = self.config_tree[i]
            cand_new_node = self.steer(new_node, near_node)
            if not cand_new_node:
                continue
            no_collision = self.collision_free(
                cand_new_node, self.robot_geom, self.obstacle_geom, self.check_collision_param
            )
            cost_is_improved = (
                cand_new_node.cost + self.calc_new_cost(cand_new_node, near_node) < near_node.cost
            )
            if no_collision and cost_is_improved:
                self.config_tree[i] = cand_new_node
                self.propagate_cost_to_leaves(new_node)

    def find_best_goal_node(self):
        dist_to_goal_list = [self.calc_dist_to_goal(nd.q) for nd in self.config_tree]
        goal_inds = [dist_to_goal_list.index(i) for i in dist_to_goal_list if i <= self.expand_dist]
        collision_free_goal_inds = []
        for goal_ind in goal_inds:
            cand_new_node = self.steer(self.config_tree[goal_ind], self.end)
            if cand_new_node and self.collision_free(
                cand_new_node, self.robot_geom, self.obstacle_geom, self.check_collision_param
            ):
                collision_free_goal_inds.append(goal_ind)
        if not collision_free_goal_inds:
            return None
        min_cost = min([self.config_tree[i].cost for i in collision_free_goal_inds])
        for i in collision_free_goal_inds:
            if self.config_tree[i].cost == min_cost:
                return i
        return None

    def propagate_cost_to_leaves(self, parent_node: "RRTStar.RRTStarConfigNode"):
        for node in self.config_tree:
            if node.parent == parent_node:
                node.cost = self.calc_new_cost(parent_node, node)
                self.propagate_cost_to_leaves(node)

    def calc_new_cost(self, from_node, to_node):
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
        start_collision_free = self.collision_free(
            self.start, self.robot_geom, self.obstacle_geom, self.check_collision_param
        )
        goal_node = self.RRTStarConfigNode(self.outer.goal_config)
        goal_node.path_q = [self.outer.goal_config]
        end_collision_free = self.collision_free(
            goal_node, self.robot_geom, self.obstacle_geom, self.check_collision_param
        )
        if start_collision_free is not None and end_collision_free is not None:
            if not start_collision_free or not end_collision_free:
                if not start_collision_free:
                    self.outer.get_logger().warning(
                        f"Start configuration {np.round(self.start.q, 3)} is in collision!"
                    )
                    self.outer.start_goal_collision = "start"
                if not end_collision_free:
                    self.outer.get_logger().warning(
                        f"Goal configuration {np.round(self.outer.goal_config, 3)} is in collision!"
                    )
                    self.outer.start_goal_collision = "goal"
                return None
        self.config_tree = [self.start]
        for i in range(self.max_iter):
            if self.outer.verbose:
                self.outer.get_logger().info(
                    f"Iteration {i}, number of nodes in tree: {len(self.config_tree)}"
                )
            rnd_node = self.sample_free()
            nearest_ind = self.get_nearest_node_index(self.config_tree, rnd_node)
            new_node = self.steer(self.config_tree[nearest_ind], rnd_node)
            if new_node and self.collision_free(
                new_node, self.robot_geom, self.obstacle_geom, self.check_collision_param
            ):
                near_inds = self.get_nearby_neighbors(new_node)
                new_node = self.choose_best_parent(new_node, near_inds)
                if new_node:
                    self.config_tree.append(new_node)
                    self.rewire(new_node, near_inds)
            if (not self.outer.rrts_search_until_max_iter) and new_node:
                last_index = self.find_best_goal_node()
                if last_index is not None:
                    return self.generate_final_course(last_index)
        self.outer.get_logger().info(f"Reached max iteration of {self.max_iter}")
        last_index = self.find_best_goal_node()
        if last_index is not None:
            return self.generate_final_course(last_index)
        return None

    def collision_free(
        self,
        candidate_node: "RRTStar.RRTStarConfigNode",
        robot_geom: RigidBodyGeom,
        obstacles: SceneObstacles,
        check_collision: bool = True,
    ) -> bool:
        if not check_collision:
            self.outer.get_logger().warning(
                "check_collision set to False. Path will be invalid."
            )
            return False
        try:
            path_q = list(candidate_node.path_q)
        except Exception as e:
            self.outer.get_logger().error(f"Error accessing candidate path_q: {e}")
            return False
        if self.collision_fn is not None:
            try:
                return bool(
                    self.collision_fn(
                        path_q=path_q,
                        robot_geom=robot_geom,
                        obstacles=obstacles,
                        node=self.outer,
                        rtb_model=self.rtb_model,
                        base_link_name=self.base_link_name,
                        world_frame_name=self.world_frame,
                    )
                )
            except TypeError:
                # Support simple signature: collision_fn(path_q) -> bool
                return bool(self.collision_fn(path_q))
        else:
            return is_path_segment_collision_free(
                path_q=path_q,
                robot_geom=robot_geom,
                obstacles=obstacles,
                node=self.outer,
                rtb_model=self.rtb_model,
                base_link_name=self.base_link_name,
                world_frame_name=self.world_frame,
            )

    @staticmethod
    def get_nearest_node_index(config_tree, rnd_node):
        dlist = [np.sum((np.array(node.q) - np.array(rnd_node.q)) ** 2) for node in config_tree]
        min_idx = dlist.index(min(dlist))
        return min_idx

    @staticmethod
    def calc_distance_and_angle(
        from_node: "RRTStar.RRTStarConfigNode", to_node: "RRTStar.RRTStarConfigNode"
    ):
        return calc_distance_and_angle(np.asarray(from_node.q), np.asarray(to_node.q))

    @staticmethod
    def compute_path_cost(path: List[NDArray]) -> float:
        return compute_path_cost(path)

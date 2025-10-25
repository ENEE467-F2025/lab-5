#!/usr/bin/env python3
"""
Class implementing the RRT* algorithm for a 2R planar robot arm.
Adapted from: https://github.com/AtsushiSakai/PythonRobotics/blob/master/ArmNavigation/rrt_star_seven_joint_arm_control/rrt_star_seven_joint_arm_control.py 

Author: Clinton Enwerem
Developed for the course ENEE467, Robotics Projects Laboratory, Fall 2025, University of Maryland, College Park, MD.
"""

import numpy as np
import math
import random
from typing import Literal
import matplotlib.pyplot as plt
from .helpers import (
    TwoRArm,
    check_collision,
    pi_frac_label
    )
from .params import (
    LINEWIDTH_PATH,
    LINEWIDTH_TREE,
    COST_LABEL_FONTSIZE,
    NODE_MARKER_SIZE,
    COST_LABEL_MARGIN,
    COST_LABEL_COLOR,
    NODE_COLOR,
    PATH_COLOR
)


def plot_tree_in_cspace(config_tree, joint_limits, ax=None, show_costs=False, config_path=None):
    """
    Plot the RRT* tree in configuration space.
    
    Args:
        config_tree: List of Node objects from RRT*
        joint_limits: Joint limits for the robot
        ax: matplotlib axis (if None, creates new figure)
        show_costs: Whether to display cost values for each node
        config_path: Optional path to highlight in purple
    """
    if ax is None:
        fig, ax = plt.subplots()
        ax.set_aspect('equal')
    
    # plot tree edges
    for node in config_tree:
        if node.parent:
            q_parent = node.parent.q
            q_node = node.q
            ax.plot([q_parent[0], q_node[0]], [q_parent[1], q_node[1]], 
                   'g-', linewidth=LINEWIDTH_TREE, alpha=0.9, zorder=1)
    
    # plot nodes
    for i, node in enumerate(config_tree):
        ax.plot(node.q[0], node.q[1], color=NODE_COLOR, marker='o', markersize=NODE_MARKER_SIZE, zorder=2)
        if show_costs:
            ax.text(node.q[0], node.q[1]+COST_LABEL_MARGIN, f'{node.cost:.1f}', 
                   fontsize=COST_LABEL_FONTSIZE, color=COST_LABEL_COLOR, ha='center', va='bottom', zorder=3)
    
    # highlight path
    if config_path is not None:
        path_array = np.array(config_path)
        ax.plot(path_array[:, 0], path_array[:, 1], 
               color=PATH_COLOR, linewidth=LINEWIDTH_PATH, zorder=4, label=r'Path')
        ax.legend()
    
    #
    tick_positions = [
        joint_limits[0],
        joint_limits[0] + (joint_limits[1] - joint_limits[0]) / 4,
        joint_limits[0] + (joint_limits[1] - joint_limits[0]) / 2,
        joint_limits[1] - (joint_limits[1] - joint_limits[0]) / 4,
        joint_limits[1]
    ]
    tick_labels = [pi_frac_label(tp) for tp in tick_positions]
    ax.set_xticks(tick_positions)
    ax.set_yticks(tick_positions)
    ax.set_xticklabels(tick_labels)
    ax.set_yticklabels(tick_labels)
    
    # set labels and limits
    ax.set_xlabel(r'$\theta_1$ (rad)')
    ax.set_ylabel(r'$\theta_2$ (rad)')
    ax.set_xlim(joint_limits[0], joint_limits[1])
    ax.set_ylim(joint_limits[0], joint_limits[1])
    ax.grid(True, alpha=0.3)
    
    return ax

def print_tree_metadata(config_tree, path=None, generate_path=False, show_costs=False):
    """Print tree statistics and metadata"""
    print("---  Tree Metadata ---")
    print(f"Total nodes in tree: {len(config_tree)}")
    if config_tree and show_costs:
        print(f"Maximum tree depth: {max([get_node_depth(node) for node in config_tree])}")
        total_cost = sum(node.cost for node in config_tree)
        print(f"Total tree cost: {total_cost:.3f}")
        avg_cost = total_cost / len(config_tree)
        print(f"Average node cost: {avg_cost:.3f}")

    if config_tree and not show_costs:
        print(f"Maximum tree depth: {max([get_node_depth(node) for node in config_tree])}")

    if path is not None and generate_path:
        print(f"Path found: Yes")
        print(f"Path length: {len(path)} waypoints")
        # calculate path cost
        if len(path) > 1:
            path_cost = sum(np.linalg.norm(np.array(path[i+1]) - np.array(path[i])) 
                           for i in range(len(path)-1))
            print(f"Path cost: {path_cost:.3f}")
    if path is not None and not generate_path:
        print(f"Collision-free path exists in tree: Yes")

    print("-" * 23)

def get_node_depth(node):
    """Get the depth of a node in the tree"""
    depth = 0
    current = node
    while current.parent is not None:
        depth += 1
        current = current.parent
    return depth

class RRTStar2R:
    """
    RRT* planner for a 2R planar robot arm.
    """
    class Node:
        def __init__(self, q):
            self.q = np.array(q, dtype=float)
            self.parent = None
            self.cost = 0.0
            self.path_q = []

    def __init__(self, start, goal, obstacles, expand_dist=0.3, path_resolution=0.1,
                 max_iter=500, connect_circle_dist=20, goal_sample_rate=0.3, check_collision=True, 
                 arm: TwoRArm=None, obstacle_type:Literal['circle', 'rect']='circle'):
        self.start = self.Node(start)
        self.end = self.Node(goal)
        self.dimension = len(start)
        self.expand_dist = expand_dist
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.obstacles = obstacles
        self.connect_circle_dist = connect_circle_dist
        self.config_tree = []
        self.check_collision = check_collision
        self.arm: TwoRArm = arm

        if self.arm is not None:
            # arm.joint_limits is [lower_bound, upper_bound] for all joints
            if len(self.arm.joint_limits) == 2:
                # Same limits for both joints
                self.joint_limits = [self.arm.joint_limits, self.arm.joint_limits]
            else:
                # different limits for each joint (should be 4 elements: [q1_min, q1_max, q2_min, q2_max])
                self.joint_limits = [(self.arm.joint_limits[0], self.arm.joint_limits[1]), 
                                   (self.arm.joint_limits[2], self.arm.joint_limits[3])]

        if obstacle_type not in ['circle', 'rect']:
            raise ValueError("obstacle_type must be either 'circle' or 'rect'")
        self.obstacle_type = obstacle_type
        if self.obstacle_type == 'circle':
            assert all(len(obs) == 3 for obs in obstacles), "Each circular obstacle must be defined by (x, y, r)"
        else:
            assert all(len(obs) == 4 for obs in obstacles), "Each rectangular obstacle must be defined by (x, y, w, h)"


    def sample_free(self):
        if random.random() > self.goal_sample_rate:
            samp_q = [np.random.uniform(l[0], l[1]) for l in self.joint_limits]
            return self.Node(samp_q)
        else:
            return self.Node(self.end.q)

    def calc_dist_to_goal(self, q):
        return np.linalg.norm(np.array(q) - np.array(self.end.q))

    def get_nearby_neighbors(self, x_new):
        curr_num_nodes = len(self.config_tree)
        if curr_num_nodes <= 1:
            near_radius = self.expand_dist
        else:
            near_radius = self.connect_circle_dist * (math.log(curr_num_nodes) / curr_num_nodes)**(1.0/self.dimension)
            near_radius = min(near_radius, self.expand_dist)
        dists = [np.sum((np.array(nd.q)-np.array(x_new.q)) ** 2) for nd in self.config_tree]
        near_inds = [idx for idx, dist in enumerate(dists) if dist <= near_radius ** 2]
        return near_inds

    def choose_best_parent(self, new_node, near_inds):
        if not near_inds:
            nearest_idx = self.get_nearest_node_index(self.config_tree, new_node)
            cand_new_node = self.steer(self.config_tree[nearest_idx], new_node)
            if cand_new_node and self.collision_free(cand_new_node):
                cand_new_node.parent = self.config_tree[nearest_idx]
                cand_new_node.cost = self.config_tree[nearest_idx].cost + self.calc_new_cost(self.config_tree[nearest_idx], cand_new_node)
                return cand_new_node
            return None
        costs = []
        for i in near_inds:
            near_node = self.config_tree[i]
            cand_new_node = self.steer(near_node, new_node)
            if cand_new_node and self.collision_free(cand_new_node):
                costs.append(self.calc_new_cost(near_node, cand_new_node))
            else:
                costs.append(float("inf"))
        min_cost = min(costs)
        if min_cost == float("inf"):
            return None
        min_ind = near_inds[costs.index(min_cost)]
        new_node = self.steer(self.config_tree[min_ind], new_node)
        if not new_node:
            return None
        new_node.parent = self.config_tree[min_ind]
        new_node.cost = min_cost
        return new_node

    def steer(self, x_nearest, x_random):
        extend_length = self.expand_dist
        start = np.array(x_nearest.q, dtype=float)
        goal = np.array(x_random.q, dtype=float)
        new_node = self.Node(start.copy())
        d = np.linalg.norm(goal - start)
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
        d_after = np.linalg.norm(goal - new_node.q)
        if d_after <= self.path_resolution:
            new_node.q = goal.copy()
            new_node.path_q.append(goal)
        new_node.parent = x_nearest
        return new_node

    def rewire(self, new_node, near_inds):
        for i in near_inds:
            near_node = self.config_tree[i]
            cand_new_node = self.steer(new_node, near_node)
            if not cand_new_node:
                continue
            no_collision = self.collision_free(cand_new_node)
            cost_is_improved = cand_new_node.cost + self.calc_new_cost(cand_new_node, near_node) < near_node.cost
            if no_collision and cost_is_improved:
                self.config_tree[i] = cand_new_node
                self.propagate_cost_to_leaves(new_node)

    def find_best_goal_node(self):
        dist_to_goal_list = [self.calc_dist_to_goal(nd.q) for nd in self.config_tree]
        goal_inds = [i for i, dist in enumerate(dist_to_goal_list) if dist <= self.expand_dist]
        collision_free_goal_inds = []
        for goal_ind in goal_inds:
            cand_new_node = self.steer(self.config_tree[goal_ind], self.end)
            if cand_new_node and self.collision_free(cand_new_node):
                collision_free_goal_inds.append(goal_ind)
        if not collision_free_goal_inds:
            return None
        min_cost = min([self.config_tree[i].cost for i in collision_free_goal_inds])
        for i in collision_free_goal_inds:
            if self.config_tree[i].cost == min_cost:
                return i
        return None

    def propagate_cost_to_leaves(self, parent_node):
        for node in self.config_tree:
            if node.parent == parent_node:
                node.cost = self.calc_new_cost(parent_node, node)
                self.propagate_cost_to_leaves(node)

    def calc_new_cost(self, from_node, to_node):
        return from_node.cost + np.linalg.norm(to_node.q - from_node.q)

    def generate_final_course(self, goal_ind):
        computed_path = [self.end.q]
        node = self.config_tree[goal_ind]
        while node.parent is not None:
            computed_path.append(node.q)
            node = node.parent
        computed_path.append(node.q)
        computed_path.reverse()
        return computed_path

    def build_tree(self):
        """Build the RRT* tree without necessarily finding a complete path to goal"""
        self.config_tree = [self.start]
        if not self.collision_free(self.start):
            print("Start configuration is in collision!")
            return
        if not self.collision_free(self.end):
            print("Goal configuration is in collision!")
            return
            
        for i in range(self.max_iter):
            rnd_node = self.sample_free()
            nearest_ind = self.get_nearest_node_index(self.config_tree, rnd_node)
            new_node = self.steer(self.config_tree[nearest_ind], rnd_node)
            if new_node and self.collision_free(new_node):
                near_inds = self.get_nearby_neighbors(new_node)
                new_node = self.choose_best_parent(new_node, near_inds)
                if new_node:
                    self.config_tree.append(new_node)
                    self.rewire(new_node, near_inds)

    def plan(self):
        """Plan a path from start to goal using RRT*"""
        self.config_tree = [self.start]
        if not self.collision_free(self.start) or not self.collision_free(self.end):
            return None
        for i in range(self.max_iter):
            rnd_node = self.sample_free()
            nearest_ind = self.get_nearest_node_index(self.config_tree, rnd_node)
            new_node = self.steer(self.config_tree[nearest_ind], rnd_node)
            if new_node and self.collision_free(new_node):
                near_inds = self.get_nearby_neighbors(new_node)
                new_node = self.choose_best_parent(new_node, near_inds)
                if new_node:
                    self.config_tree.append(new_node)
                    self.rewire(new_node, near_inds)
            last_index = self.find_best_goal_node()
            if last_index is not None:
                return self.generate_final_course(last_index)
        last_index = self.find_best_goal_node()
        if last_index is not None:
            return self.generate_final_course(last_index)
        return None

    def collision_free(self, candidate_node):
        if not self.check_collision:
            return True
        # check the node configuration and path configurations
        configs_to_check = [candidate_node.q] + candidate_node.path_q
        for q in configs_to_check:
            self.arm.update_joints(q)  # q is already a list [q1, q2]
            if self.obstacle_type == 'rect':
                for obstacle in self.obstacles:
                    if check_collision(self.arm, obstacle=obstacle, obstacle_type=self.obstacle_type):
                        return False
            elif self.obstacle_type == 'circle':
                for obstacle in self.obstacles:
                    is_colliding, _ = check_collision(self.arm, obstacle=obstacle, obstacle_type=self.obstacle_type)
                    if is_colliding:
                        return False
        return True

    @staticmethod
    def get_nearest_node_index(config_tree, rnd_node):
        dlist = [np.sum((np.array(node.q) - np.array(rnd_node.q))**2) for node in config_tree]
        min_idx = dlist.index(min(dlist))
        return min_idx
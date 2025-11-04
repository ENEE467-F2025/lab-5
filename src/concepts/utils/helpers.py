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
Utilities for configuration space and workspace visualization and collision checking in the plane.

References:
1. K. M. Lynch and F. C. Park, Modern Robotics. Cambridge University Press, 2019.

Author: Clinton Enwerem
Developed for the course ENEE467, Robotics Projects Laboratory, Fall 2025, University of Maryland, College Park, MD.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
from matplotlib.font_manager import FontProperties
from typing import Literal, Union, List, Tuple, Dict, overload
from numpy.typing import NDArray
import matplotlib.patches as patches
from fractions import Fraction
import shutil, os
from scipy.spatial import ConvexHull
import re
from utils.params import *
import argparse

# Plotting
plt.rcParams["font.size"] = FONTSIZE
plt.rcParams["axes.labelsize"] = FONTSIZE-4
plt.rcParams["xtick.labelsize"] = FONTSIZE-4
plt.rcParams["ytick.labelsize"] = FONTSIZE-4
plt.rcParams["legend.fontsize"] = FONTSIZE
plt.rcParams["axes.titlesize"] = FONTSIZE
plt.rcParams["text.usetex"] = False
rc = {"font.family" : "serif", 
      "mathtext.fontset" : "stix"}
plt.rcParams.update(rc)
plt.rcParams["font.serif"] = ["Times New Roman"] + plt.rcParams["font.serif"]

# 2R planar arm class
class TwoRArm:
    """
    Constructor for a 2R planar arm
    """
    def __init__(self, link_lengths: List, joint_limits:List):
        assert len(link_lengths) == 2
        self.link_lengths = np.array(link_lengths)
        self.n_links = len(self.link_lengths)
        self.points = [[0,0] for _ in range(3)]  
        self.joint_limits = joint_limits

    def update_joints(self, joint_angles):
        self.joint_angles = joint_angles
        self.update_points()

    def update_points(self):
        theta1, theta2 = self.joint_angles
        l1, l2 = self.link_lengths
        # joint 1
        x1 = l1 * np.cos(theta1)
        y1 = l1 * np.sin(theta1)
        # end-effector
        x2 = x1 + l2 * np.cos(theta1 + theta2)
        y2 = y1 + l2 * np.sin(theta1 + theta2)
        # transform config to Cartesian space
        self.points = [[0,0],[x1,y1],[x2,y2]]

    @overload
    def compute_fk(self, joint_angles, all_links: Literal[False]=False) -> Tuple[float, float]: ...

    @overload
    def compute_fk(self, joint_angles, all_links: Literal[True]=True) -> NDArray: ...

    def compute_fk(self, joint_angles, all_links=False) -> Tuple[float, float]:
        """
        Forward kinematics for 2R planar arm
        Returns (x, y) of end-effector
        """
        theta1, theta2 = joint_angles
        l1, l2 = self.link_lengths
        if all_links:
            # return all joint positions
            points = np.zeros((3,2))
            points[0,:] = [0,0]
            points[1,:] = [l1 * np.cos(theta1), l1 * np.sin(theta1)]
            points[2,:] = [l1 * np.cos(theta1) + l2 * np.cos(theta1 + theta2),
                           l1 * np.sin(theta1) + l2 * np.sin(theta1 + theta2)]
            return points
        # return end-effector position
        x = l1 * np.cos(theta1) + l2 * np.cos(theta1 + theta2)
        y = l1 * np.sin(theta1) + l2 * np.sin(theta1 + theta2)
        return (x, y)
    
    def compute_ik(self, x, y) -> List[Tuple[float, float]]:
        """
        Inverse kinematics for 2R planar arm
        Returns list of (theta1, theta2) tuples
        """
        l1, l2 = self.link_lengths
        D = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)
        if abs(D) > 1:
            return []  # No solutions

        theta2_options = [np.arctan2(np.sqrt(1 - D**2), D), np.arctan2(-np.sqrt(1 - D**2), D)]
        solutions = []
        for theta2 in theta2_options:
            k1 = l1 + l2 * np.cos(theta2)
            k2 = l2 * np.sin(theta2)
            theta1 = np.arctan2(y, x) - np.arctan2(k2, k1)
            # Normalize angles to be within [-pi, pi]
            theta1 = (theta1 + np.pi) % (2 * np.pi) - np.pi
            theta2 = (theta2 + np.pi) % (2 * np.pi) - np.pi
            # Check joint limits
            if (self.joint_limits[0] < theta1 < self.joint_limits[1]) and (self.joint_limits[0] < theta2 < self.joint_limits[1]):
                solutions.append((theta1, theta2))
        return solutions

# AStar Utilities
def astar_torus(grid: NDArray, start_node: Union[list, NDArray], goal_node: Union[list, NDArray], M) -> Tuple[List, float]:
    """
    Finds a path between an initial and goal joint configuration using
    the A* Algorithm on a toroidal grid.

    Args:
        grid: An occupancy grid (ndarray)
        start_node: Initial joint configuration (tuple)
        goal_node: Goal joint configuration (tuple)

    Returns:
        Obstacle-free route in joint space from start_node to goal_node
    
    Adapted from: https://github.com/AtsushiSakai/PythonRobotics/blob/master/ArmNavigation/arm_obstacle_navigation/arm_obstacle_navigation.py
    """

    # check if start or goal is in collision
    if grid[start_node] == OBSTACLE:
        print(f"Start cell {start_node} is in collision!")
        return [], float('inf')
    
    if grid[goal_node] == OBSTACLE:
        print(f"Goal cell {goal_node} is in collision!")
        return [], float('inf')

    grid[start_node] = START
    grid[goal_node] = GOAL

    parent_map = [[() for _ in range(M)] for _ in range(M)]

    heuristic_map = calc_heuristic_map(M, goal_node)

    explored_heuristic_map = np.full((M, M), np.inf)
    distance_map = np.full((M, M), np.inf)
    explored_heuristic_map[start_node] = heuristic_map[start_node]
    distance_map[start_node] = 0
    while True:
        grid[start_node] = START
        grid[goal_node] = GOAL 

        current_node = np.unravel_index(
            np.argmin(explored_heuristic_map, axis=None), explored_heuristic_map.shape)
        min_distance = np.min(explored_heuristic_map)
        if (current_node == goal_node) or np.isinf(min_distance):
            break

        grid[current_node] = CLOSED
        explored_heuristic_map[current_node] = np.inf

        i, j = current_node[0], current_node[1]

        neighbors = find_neighbors(i, j, M=M, wrap=False)

        for neighbor in neighbors:
            # skip if neighbor is an obstacle
            if grid[neighbor] == OBSTACLE:
                continue
            # proceed only if neighbor is free or the goal
            if grid[neighbor] == FREE or grid[neighbor] == GOAL:
                distance_map[neighbor] = distance_map[current_node] + 1
                # f(q) = g(q) + h(q); where g(q) is distance_map, h(q) is heuristic_map
                explored_heuristic_map[neighbor] = (distance_map[neighbor] + heuristic_map[neighbor])
                parent_map[neighbor[0]][neighbor[1]] = current_node
                grid[neighbor] = OPEN

    if np.isinf(explored_heuristic_map[goal_node]):
        route = []
    else:
        route = [goal_node]
        while parent_map[route[0][0]][route[0][1]] != ():
            route.insert(0, parent_map[route[0][0]][route[0][1]])

        print(f"The route found covers {len(route)} grid cells.")
    path_cost = distance_map[goal_node]
    return route, path_cost


def find_neighbors(i: int, j: int, M: int, wrap: bool = False) -> List[Tuple[int, int]]:
    """Finds the 4-connected neighbors of node (i, j)."""
    neighbors = []

    if wrap:
        neighbors = [
            ((i - 1) % M, j),
            ((i + 1) % M, j),
            (i, (j - 1) % M),
            (i, (j + 1) % M)
        ]
    else:
        if i > 0:
            neighbors.append((i - 1, j))
        if i < M - 1:
            neighbors.append((i + 1, j))
        if j > 0:
            neighbors.append((i, j - 1))
        if j < M - 1:
            neighbors.append((i, j + 1))
    return neighbors

# Manhattan distance heuristic
def calc_heuristic_map(M, goal_node):
    X, Y = np.meshgrid([i for i in range(M)], [i for i in range(M)])
    heuristic_map = np.abs(X - goal_node[1]) + np.abs(Y - goal_node[0])
    for i in range(heuristic_map.shape[0]):
        for j in range(heuristic_map.shape[1]):
            heuristic_map[i, j] = min(heuristic_map[i, j],
                                      M - i - 1 + heuristic_map[M - 1, j],
                                      i + heuristic_map[0, j],
                                      M - j - 1 + heuristic_map[i, M - 1],
                                      j + heuristic_map[i, 0]
                                      )

    return heuristic_map

def ensure_dir(path):
    os.makedirs(path, exist_ok=True)

def str2bool(v):
    if isinstance(v, bool):
        return v
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')

def parse_single_expression(expr):
    """
    Helper for parsing a string expression representing a multiple or fraction of np.pi,
    e.g. "pi/2", "-pi/3", "1.5*pi", "pi/1.2", "2.5", "-0.5*pi", etc.
    Returns the corresponding float value.
    """
    expr = expr.strip().replace('π', 'pi')
    expr = expr.replace('^', '**')
    expr = expr.replace('PI', 'pi').replace('Pi', 'pi')
    expr = expr.replace('np.pi', 'pi').replace('np.', '')
    expr = expr.replace(' ', '')

    # logic for replacing 'pi' with 'np.pi'
    # this should also handle cases like '5pi/12' with no operator between number and pi
    expr = re.sub(r'(\d+(\.\d+)?)(pi)', r'\1*np.pi', expr)
    expr = expr.replace('pi', 'np.pi')

    # Only allow np and float literals
    return eval(expr, {"np": np, "__builtins__": {}})

def pretty_print_array_pi(arr):
    """
    Pretty print a NumPy array with elements expressed as multiples of np.pi, rounded to 4 decimals.
    """
    arr = np.array(arr)
    def format_elem(x):
        frac = x / np.pi
        return f"{frac:.4f}π"
    if arr.ndim == 1:
        return [format_elem(x) for x in arr] 
    else:
        return [[format_elem(x) for x in row] for row in arr]

def check_latex_installation():
    """
    Checks if LaTeX is installed and accessible from the system's PATH.
    Also attempts to get the version information.
    """
    latex_executable = shutil.which('latex') or shutil.which('pdflatex')
    if latex_executable and latex_executable is not None:
        return True
    else:
        return False
    
# define linear mapping functions between grid indices and theta values
def i_to_theta(i, joint_limits): 
    return joint_limits[0] + (i / M) * (joint_limits[1] - joint_limits[0])

def theta_to_i(theta, joint_limits):  
    return (theta - joint_limits[0]) / (joint_limits[1] - joint_limits[0]) * M

# use true LaTeX rendering 
latex_installed = check_latex_installation()
if latex_installed is not None and latex_installed:
    plt.rcParams['text.usetex'] = True
    plt.rcParams['font.family'] = 'serif'
else:
    pass

# define style for arrows
style = "Fancy, tail_width=0.25, head_width=2.25, head_length=4"
kw = dict(arrowstyle=style, color="k", lw=0.55)

def add_angle_arcs(ax, x0: float, y0: float, x1: float, y1: float, theta1: float, theta2: float, l1: float, l2: float):
    """
    Draw curved arrows for theta1 and theta2 using only kinematic quantities
    """

    # arc for theta1 at base (x0,y0), assumed equal to the origin
    r1 = 0.45 * l1  # radius of arc

    # start and end points of small arc around (0,0)
    arc1_start = (x0 + r1 * np.cos(0), y0 + r1 * np.sin(0))               # along +x
    arc1_end   = (x0 + r1 * np.cos(theta1), y0 + r1 * np.sin(theta1))     # at theta1

    arc_1 = patches.FancyArrowPatch(
        arc1_start,
        arc1_end,
        connectionstyle=f"arc3,rad=0.2",
        zorder=2,
        **kw
    )
    ax.add_patch(arc_1)

    # label theta1 slightly away from the arc endpoint
    ax.text(
        r1 * np.cos(theta1/2) * 1.3,
        r1 * np.sin(theta1/2) * 1.05,
        r'$\theta_1$',
        
        color='black'
    )

    # arc for theta2 around joint 1

    r2 = 0.35 * l2
    base_angle2 = theta1     # note: The local reference for theta2 is the direction of link1
    arc2_start = (x1 + r2 * np.cos(base_angle2),
                  y1 + r2 * np.sin(base_angle2))
    arc2_end   = (x1 + r2 * np.cos(base_angle2 + theta2),
                  y1 + r2 * np.sin(base_angle2 + theta2))

    arc_2 = patches.FancyArrowPatch(
        arc2_start,
        arc2_end,
        connectionstyle=f"arc3,rad=0.2",
        zorder=2,
        **kw
    )
    ax.add_patch(arc_2)

    # label theta2 near arc midpoint
    mid_angle = base_angle2 + theta2 / 2
    ax.text(
        x1 + 1.3 * r2 * np.cos(mid_angle),
        y1 + 1.3 * r2 * np.sin(mid_angle),
        r'$\theta_2$',
        
        color='black'
    )

def pi_frac_label(val):
    """
    For latex axes labeling
    """
    frac = val / np.pi
    if np.isclose(frac, 0):
        return r"$0$"
    elif np.isclose(frac, 1):
        return r"$\pi$"
    elif np.isclose(frac, -1):
        return r"$-\pi$"
    elif np.isclose(frac, 0.5):
        return r"$\frac{\pi}{2}$"
    elif np.isclose(frac, -0.5):
        return r"$-\frac{\pi}{2}$"
    elif np.isclose(frac, 1/3):
        return r"$\frac{\pi}{3}$"
    elif np.isclose(frac, -1/3):
        return r"$-\frac{\pi}{3}$"
    elif np.isclose(frac, 1/4):
        return r"$\frac{\pi}{4}$"
    elif np.isclose(frac, -1/4):
        return r"$-\frac{\pi}{4}$"
    elif np.isclose(frac, 1/6):
        return r"$\frac{\pi}{6}$"
    elif np.isclose(frac, -1/6):
        return r"$-\frac{\pi}{6}$"
    else:
        # force fractional formatting for arbitrary multiples of pi
        frac = Fraction(val / np.pi).limit_denominator(25)
        num, denom = frac.numerator, frac.denominator
        # Case 1 - exactly zero
        if num == 0:
            return r"$0$"

        # Case 2 - integer multiples of pi, e.g. pi, 2pi
        if denom == 1:
            if abs(num) == 1:
                return r"$\pi$" if num > 0 else r"$-\pi$"
            else:
                return fr"${num}\pi$"

        # Case 3 - fractional multiples of pi
        # if numerator is \pm 1, omit 1 in the numerator
        if abs(num) == 1:
            sign = "-" if num < 0 else ""
            return fr"${sign}\frac{{\pi}}{{{denom}}}$"
        else:
            return fr"$\frac{{{num}\pi}}{{{denom}}}$"
    
        
# Plotting utilities
def plot_2r_with_rect(theta1: float, theta2: float, link_lengths: List[float], rect: List[float], embellish: Union[bool, None]=True):
    """
    theta1, theta2: joint angles in radians
    link_lengths: [l1, l2]
    rect: [x_corner, y_corner, width, height]
    """
    # forward kinematics
    l1, l2 = link_lengths
    x0, y0 = 0., 0.
    x1 = l1 * np.cos(theta1)
    y1 = l1 * np.sin(theta1)
    x2 = x1 + l2 * np.cos(theta1 + theta2)
    y2 = y1 + l2 * np.sin(theta1 + theta2)
    x3 = x1 + l2 * np.cos(theta1) 
    y3 = y1 + l2 * np.sin(theta1)

    # set up plot
    fig, ax = plt.subplots()
    ax.set_aspect('auto')

    # draw arm
    ax.plot([x0, x1], [y0, y1], 'k-', linewidth=2, zorder=2)
    ax.plot([x1, x2], [y1, y2], 'k-', linewidth=2, zorder=2)
    ax.plot([x0, x1], [y0, y1], 'ko', markersize=6, zorder=2)
    if embellish is True:
        ax.plot([0, l1+l2], [0, 0], color='gray', linestyle='--', linewidth=0.5, zorder=0)
        ax.plot([x1, x3], [y1, y3], color='gray', linestyle='--', linewidth=0.5, zorder=0)

    if embellish is True:
        # annotate link lengths l1 and l2 above the links
        ax.text(x0 + (x1 - x0) / 2, y0 + (y1 - y0) / 2 + 0.08 * l1, r'$\ell_1$',  color='k', ha='center', va='bottom')
        ax.text(x1 + (x2 - x1) / 2 -0.12, y1 + (y2 - y1) / 2 + 0.06 * l2, r'$\ell_2$',  color='k', ha='center', va='bottom')

        # arcs
        add_angle_arcs(ax, x0, y0, x1, y1, theta1, theta2, l1, l2)

    # rectangular obstacle
    x_corner, y_corner, width, height = rect
    x_min = x_corner 
    x_max = x_corner + width 
    y_min = y_corner
    y_max = y_corner + height
    rectangle = Rectangle((x_min, y_min), width, height,
                     facecolor='gray', alpha=0.7, edgecolor='black', linewidth=1.5, zorder=2)
    ax.add_patch(rectangle)
    ax.text(x_min + width/2, 
            y_min + height/2,
            r'$\mathcal{O}$', 
             
            ha='center', 
            va='center')

    #  axes
    total_length = l1 + l2
    ax.set_xlim(0, total_length)
    ax.set_xlabel(r'$x$')
    ax.set_ylabel(r'$y$')
    plt.tight_layout()
    plt.grid(False)

def plot_2r_path_with_rect(route: list, arm: TwoRArm, rect: List[float], M: int, embellish: Union[bool, None]=True):
    """
    Plot the path of the 2R arm through a sequence of joint configurations, with a rectangular obstacle.
    route: list of [theta1_idx, theta2_idx] pairs (indices in discretized joint space)
    arm: TwoRArm instance
    rect: [x_corner, y_corner, width, height]
    M: number of discretization steps for each joint
    embellish: whether to add labels/arcs
    """
    # forward kinematics
    l1, l2 = arm.link_lengths
    lower_limit, upper_limit = arm.joint_limits

    # set up plot
    fig, ax = plt.subplots()
    ax.set_aspect('auto')

    # iterate through the route to get the final arm configuration
    for node in route:
        node_theta1 = lower_limit + (upper_limit - lower_limit) * node[0] / (M - 1)
        node_theta2 = lower_limit + (upper_limit - lower_limit) * node[1] / (M - 1)
        arm.update_joints([node_theta1, node_theta2])
        points = arm.compute_fk([node_theta1, node_theta2], all_links=True)
        x0, y0 = points[0]
        x1, y1 = points[1]
        x2, y2 = points[2]

        # draw arm with increasing alpha for each step
        if route.index(node) == 0:
            color_arg = START_COLOR
            alpha_arg = 1.0
            z_order_arg = 3
        elif route.index(node) == len(route) - 1:
            color_arg = GOAL_COLOR
            alpha_arg = 1.0
            z_order_arg = 3
        else:
            color_arg = 'k'
            alpha_arg = 0.1# + 0.9 * (route.index(node) / len(route))
            z_order_arg = 2
        ax.plot([x0, x1], [y0, y1], linewidth=2, zorder=z_order_arg, color=color_arg, alpha=alpha_arg)
        ax.plot([x1, x2], [y1, y2], linewidth=2, zorder=z_order_arg, color=color_arg, alpha=alpha_arg)
        ax.plot([x0, x1], [y0, y1], 'o', markersize=6, zorder=z_order_arg, color=color_arg, alpha=alpha_arg)

        # start and goal annotations
        if route.index(node) == 0:
            ax.text(x2, y2 + 0.15 * l2, r'${q}_{\mathrm{start}}$',  color=START_LABEL_COLOR, ha='center', va='bottom')
        if route.index(node) == len(route) - 1:
            ax.text(x2, y2 + 0.15 * l2, r'${q}_{\mathrm{goal}}$',  color=GOAL_LABEL_COLOR, ha='center', va='bottom')

    # rectangular obstacle 
    x_corner, y_corner, width, height = rect
    x_min = x_corner 
    x_max = x_corner + width 
    y_min = y_corner
    y_max = y_corner + height
    rectangle = Rectangle((x_min, y_min), width, height,
                     facecolor='gray', alpha=0.7, edgecolor='black', linewidth=1.5, zorder=2)
    ax.add_patch(rectangle)
    ax.text(x_min + width/2, 
            y_min + height/2,
            r'$\mathcal{O}$', 
             
            ha='center', 
            va='center')

    #  axes
    total_length = l1 + l2
    ax.set_xlim(-total_length, total_length)
    ax.set_ylim(-total_length, total_length)
    ax.set_xlabel(r'$x$')
    ax.set_ylabel(r'$y$')
    plt.tight_layout()
    plt.grid(False)

def plot_2r_with_circle(
    theta1: float,
    theta2: float,
    link_lengths: List[float],
    circle: List[float] = [0.425, 1.625, 0.5303],
    embellish: Union[bool, None]=True
    ):
    """
    Plot the 2R robot with a circular obstacle in its workspace.
    circle: [x_center, y_center, radius]
    """
    l1, l2 = link_lengths
    x0, y0 = 0., 0.
    x1 = l1 * np.cos(theta1)
    y1 = l1 * np.sin(theta1)
    x2 = x1 + l2 * np.cos(theta1 + theta2)
    y2 = y1 + l2 * np.sin(theta1 + theta2)
    x3 = x1 + l2 * np.cos(theta1)
    y3 = y1 + l2 * np.sin(theta1)

    _, ax = plt.subplots()
    ax.set_aspect('auto')

    # draw arm
    ax.plot([x0, x1], [y0, y1], 'k-', linewidth=2, zorder=2)
    ax.plot([x1, x2], [y1, y2], 'k-', linewidth=2, zorder=2)
    ax.plot([x0, x1], [y0, y1], 'ko', markersize=6, zorder=2)
    if embellish is True:
        ax.plot([0, l1+l2], [0, 0], color='gray', linestyle='--', linewidth=0.5, zorder=0)
        ax.plot([x1, x3], [y1, y3], color='gray', linestyle='--', linewidth=0.5, zorder=0)

    if embellish is True:
        # annotate link lengths l1 and l2 above the links
        ax.text(x0 + (x1 - x0) / 2, y0 + (y1 - y0) / 2 + 0.08 * l1, r'$\ell_1$',  color='k', ha='center', va='bottom')
        ax.text(x1 + (x2 - x1) / 2 -0.12, y1 + (y2 - y1) / 2 + 0.06 * l2, r'$\ell_2$',  color='k', ha='center', va='bottom')

        # arcs
        add_angle_arcs(ax, x0, y0, x1, y1, theta1, theta2, l1, l2)

    # circular obstacle
    x_c, y_c, r_c = circle
    circ = Circle((x_c, y_c), r_c, facecolor='gray', alpha=0.7, edgecolor='black', linewidth=1.5, zorder=2)
    ax.add_patch(circ)
    ax.text(
        x_c, 
        y_c, 
        r'$\mathcal{O}$', 
         
        ha='center', 
        va='center')

    # axes
    total_length = l1 + l2
    ax.set_xlim(0, total_length)
    ax.set_xlabel(r'$x$')
    ax.set_ylabel(r'$y$')
    plt.tight_layout()
    plt.grid(False)

def plot_2r_path_with_circle(
    route: list,
    arm: TwoRArm,
    circle: List[float],
    M: int,
    embellish: Union[bool, None]=True
):
    """
    Plot the path of the 2R arm through a sequence of joint configurations, with a circular obstacle.
    route: list of [theta1_idx, theta2_idx] pairs (indices in discretized joint space)
    arm: TwoRArm instance
    circle: [x_center, y_center, radius]
    M: number of discretization steps for each joint
    embellish: whether to add labels/arcs
    """
    l1, l2 = arm.link_lengths
    lower_limit, upper_limit = arm.joint_limits

    # set up plot
    fig, ax = plt.subplots()
    ax.set_aspect('auto')

    # iterate through the route to get the final arm configuration
    for node in route:
        node_theta1 = lower_limit + (upper_limit - lower_limit) * node[0] / (M - 1)
        node_theta2 = lower_limit + (upper_limit - lower_limit) * node[1] / (M - 1)
        arm.update_joints([node_theta1, node_theta2])
        points = arm.compute_fk([node_theta1, node_theta2], all_links=True)
        x0, y0 = points[0]
        x1, y1 = points[1]
        x2, y2 = points[2]

        # draw arm with increasing alpha for each step
        if route.index(node) == 0:
            color_arg = START_COLOR
            alpha_arg = 1.0
            z_order_arg = 3
        elif route.index(node) == len(route) - 1:
            color_arg = GOAL_COLOR
            alpha_arg = 1.0
            z_order_arg = 3
        else:
            color_arg = 'k'
            alpha_arg = 0.1
            z_order_arg = 2
        ax.plot([x0, x1], [y0, y1], linewidth=2, zorder=z_order_arg, color=color_arg, alpha=alpha_arg)
        ax.plot([x1, x2], [y1, y2], linewidth=2, zorder=z_order_arg, color=color_arg, alpha=alpha_arg)
        ax.plot([x0, x1], [y0, y1], 'o', markersize=6, zorder=z_order_arg, color=color_arg, alpha=alpha_arg)

        # start and goal annotations
        if route.index(node) == 0:
            ax.text(x2, y2 + 0.15 * l2, r'${q}_{\mathrm{start}}$',  color=START_LABEL_COLOR, ha='center', va='bottom')
        if route.index(node) == len(route) - 1:
            ax.text(x2, y2 + 0.15 * l2, r'${q}_{\mathrm{goal}}$',  color=GOAL_LABEL_COLOR, ha='center', va='bottom')

    # circular obstacle
    x_c, y_c, r_c = circle
    circ = Circle((x_c, y_c), r_c, facecolor='gray', alpha=0.7, edgecolor='black', linewidth=1.5, zorder=2)
    ax.add_patch(circ)
    ax.text(
        x_c, 
        y_c, 
        r'$\mathcal{O}$', 
        ha='center', 
        va='center'
    )

    # axes
    total_length = l1 + l2
    ax.set_xlim(-total_length, total_length)
    ax.set_xlabel(r'$x$')
    ax.set_ylabel(r'$y$')
    plt.tight_layout()
    plt.grid(False)

def plot_workspace(workspace_ee: np.ndarray, all_points: np.ndarray, ax: plt.Axes, embellish: Union[bool, None]=True):
    """
    Plot the workspace of the 2R robot.
    - Filled orange polygon: all manipulator points (joints + links)
    - Blue scatter: end-effector points
    """
    # flatten all manipulator points (all joints including end-effector)
    flat_points = all_points.reshape(-1, 2)  # shape (N*3, 2)

    # convex hull of all points to represent the arm workspace
    hull = ConvexHull(flat_points)
    hull_vertices = flat_points[hull.vertices]

    # add filled polygon
    poly = patches.Polygon(
        hull_vertices, closed=True, facecolor="orange", edgecolor="black", alpha=0.3, lw=1.5
    )
    ax.add_patch(poly)

    # plot end-effector points
    if embellish is True:
        ax.scatter(workspace_ee[:, 0], workspace_ee[:, 1], s=2, color="#1f77b4", alpha=0.4)

    # Aspect ratio
    ax.set_aspect('equal', adjustable='box')

    # annotations
    # compute EE convex hull to place annotation outside boundary
    ee_hull = ConvexHull(workspace_ee)
    ee_top_point = workspace_ee[ee_hull.vertices, 1].max()
    ee_center_x = workspace_ee[:, 0].mean()

    # place W_EE annotation slightly above topmost EE point
    if embellish is True:
        ax.text(ee_center_x +0.2, ee_top_point - 0.1, r'$\mathcal{W}_{\mathrm{EE}}$', 
            color='#1f77b4', ha='center')

        arm_mean = flat_points.mean(axis=0)
        ax.text(arm_mean[0], arm_mean[1]-0.5, r'$\mathcal{W}$',  color='k', ha='center')

    ax.text(ee_center_x + 0.2, ee_top_point - 0.1, r'$\mathcal{W}$',  color='#d35400', ha='center')

    ax.set_xlabel(r'$x$')
    ax.set_ylabel(r'$y$')

# Rectangle collision detection
def detect_collision_rect(line_seg: List[List[float]], rect: List[float], eps: Union[float, None] = EPS_OBS) -> bool:
    """
    line_seg: [[x0,y0],[x1,y1]]
    rect: [x_corner, y_corner, width, height]

    Returns True if line intersects rectangle
    """
    x0, y0 = line_seg[0]
    x1, y1 = line_seg[1]
    assert len(rect) == 4, "The rectangle must be defined with four parameters: x,y, w, h"
    x_corner, y_corner, width, height = rect
    x_min = x_corner 
    x_max = x_corner + width 
    y_min = y_corner
    y_max = y_corner + height

    # Liang-Barsky algorithm for line-rectangle intersection
    dx = x1 - x0
    dy = y1 - y0
    p = [-dx, dx, -dy, dy]
    q = [x0 - x_min, x_max - x0, y0 - y_min, y_max - y0]
    u1, u2 = 0, 1
    for pi, qi in zip(p, q):
        if pi == 0:
            if qi < 0:
                return False
        else:
            # compute intersection t values
            t = qi / pi
            if pi < 0:
                u1 = max(u1, t)
            else:
                u2 = min(u2, t)
    # require more than epsilon overlap to count as intersection
    return (u1-eps) <= (u2+eps)

# Circle collision detection
def detect_collision_circle(line_seg: List[List[float]], circle: List[float], eps: Union[float, None] = EPS_OBS) -> Tuple[bool, float]:
    """
    Method to determine if a line intersects with a circle

    Modified from Python Robotics Arm Navigation Submodule: https://github.com/AtsushiSakai/PythonRobotics/blob/master/ArmNavigation/arm_obstacle_navigation/arm_obstacle_navigation.py

    line_seg: [[x0,y0], [x1,y1]]
    circle: [center_x, center_y, radius]

    Returns True if the line intersects the circle.
    """
    a_vec = np.array([line_seg[0][0], line_seg[0][1]])
    b_vec = np.array([line_seg[1][0], line_seg[1][1]])
    assert len(circle) == 3, "The circle must be defined with three parameters: x, y, radius"
    c_vec = np.array([circle[0], circle[1]])
    radius = circle[2]
    line_vec = b_vec - a_vec
    line_mag = np.linalg.norm(line_vec)
    circle_vec = c_vec - a_vec
    proj = circle_vec.dot(line_vec / line_mag)
    if proj <= 0:
        closest_point = a_vec
    elif proj >= line_mag:
        closest_point = b_vec
    else:
        closest_point = a_vec + line_vec * proj / line_mag
    min_dist = np.linalg.norm(closest_point - c_vec)
    collision = min_dist < (radius + eps)
    return collision, min_dist

def safe_joint_limits_circle(L1: float, L2: float, circle: List[float], 
                             theta1_range:Tuple[float, float]=(-np.pi, np.pi), 
                             theta2_range:Tuple[float, float]=(-np.pi, np.pi), 
                             num_samples:int=100):
    """
    Compute safe (collision-free) joint limits for a 2R planar robot given a circular obstacle.

    Returns conservative joint limits (theta1_min, theta1_max), (theta2_min, theta2_max)
    such that all configurations inside are collision-free.
    """

    c_x, c_y, r = circle
    theta1_vals = np.linspace(*theta1_range, num_samples)
    theta2_vals = np.linspace(*theta2_range, num_samples)

    collision_grid = np.zeros((num_samples, num_samples), dtype=bool)

    for i, th1 in enumerate(theta1_vals):
        for j, th2 in enumerate(theta2_vals):
            # forward kinematics
            p0 = np.array([0.0, 0.0])
            p1 = np.array([L1*np.cos(th1), L1*np.sin(th1)])
            p2 = p1 + np.array([L2*np.cos(th1+th2), L2*np.sin(th1+th2)])

            # check both links
            for seg in [(p0, p1), (p1, p2)]:
                collides, _ = detect_collision_circle(seg, [c_x, c_y, r])
                if collides:
                    collision_grid[i, j] = True
                    break

    # identify the largest connected rectangle of collision-free samples
    free_idxs = np.argwhere(~collision_grid)
    if free_idxs.size == 0:
        raise RuntimeError("No collision-free region found.")
    
    th1_min = theta1_vals[free_idxs[:,0]].min()
    th1_max = theta1_vals[free_idxs[:,0]].max()
    th2_min = theta2_vals[free_idxs[:,1]].min()
    th2_max = theta2_vals[free_idxs[:,1]].max()

    return (th1_min, th1_max), (th2_min, th2_max)

# C-space occupancy grid
# overloaded function signatures for type hinting
@overload
def get_cspace_grid(arm: TwoRArm, 
                    obstacle: List[float], 
                    M:int=100, 
                    obstacle_type: Union[Literal['rect', 'circle'], None]='rect',
                    check_collision: Union[bool, None]=True) -> Tuple[NDArray, Dict[Tuple[int, int], List[float]]]: ...
@overload
def get_cspace_grid(arm: TwoRArm, 
                    obstacle: List[float], 
                    M:int=100, 
                    obstacle_type: Union[Literal['rect', 'circle'], None]='circle',
                    check_collision: Union[bool, None]=True) -> Tuple[NDArray, NDArray, Dict[Tuple[int, int], List[float]]]: ...  

def get_cspace_grid(
    arm,
    obstacle,
    M=100,
    obstacle_type="rect",
    check_collision=True
):
    """
    Compute the configuration-space occupancy grid for the 2R arm.
    Each cell corresponds to a discrete joint configuration.
    """
    grid = np.zeros((M, M), dtype=int)
    grid_to_config_map: Dict[Tuple[int, int], List[float]] = {}
    if obstacle_type == "circle":
        min_dists = np.zeros((M, M))
    
    theta_vals = np.linspace(arm.joint_limits[0], arm.joint_limits[1], M)

    # check obstacle definition
    if obstacle_type == "rect":
        assert len(obstacle) == 4, \
            "Rectangle obstacle must be defined as [x, y, w, h]."
    elif obstacle_type == "circle":
        assert len(obstacle) == 3, \
            "Circle obstacle must be defined as [x, y, radius]."
    else:
        raise ValueError(f"Unknown obstacle_type '{obstacle_type}'")

    for i, th1 in enumerate(theta_vals):
        for j, th2 in enumerate(theta_vals):
            config = [th1, th2]
            grid_to_config_map[(i, j)] = config

            if not check_collision:
                continue

            arm.update_joints(config)
            points = arm.points
            collision = False
            min_dist_arr = []

            for k in range(len(points) - 1):
                segment = [points[k], points[k + 1]]
                if obstacle_type == "rect":
                    if detect_collision_rect(segment, obstacle):
                        collision = True
                        break
                elif obstacle_type == "circle":
                    is_colliding, min_dist = detect_collision_circle(segment, obstacle)
                    min_dist_arr.append(min_dist)
                    if is_colliding:
                        collision = True
                        break

            grid[i, j] = 1 if collision else 0
            if obstacle_type == "circle":
                min_dists[i, j] = min(min_dist_arr) if min_dist_arr else np.inf

    if obstacle_type == "circle":
        return grid, min_dists, grid_to_config_map
    return grid, grid_to_config_map

def config_to_index(theta: np.ndarray, arm: TwoRArm, M: int):
    """
    Map continuous joint angles (theta1, theta2)
    to discrete grid indices (i, j).
    """
    lower, upper = arm.joint_limits
    theta1, theta2 = theta

    # scale to [0, M-1]
    i = int(np.round((theta1 - lower) / (upper - lower) * (M - 1)))
    j = int(np.round((theta2 - lower) / (upper - lower) * (M - 1)))

    # wrap around toroidal C-space
    i %= M
    j %= M
    return (i, j)

@overload
def check_collision(arm: TwoRArm, obstacle: List[float], obstacle_type: Union[Literal['rect', 'circle'], None]='rect') -> bool: ...

@overload
def check_collision(arm: TwoRArm, obstacle: List[float], obstacle_type: Union[Literal['rect', 'circle'], None]='rect') -> Tuple[bool, float]: ...

def check_collision(arm: TwoRArm, obstacle: List[float], obstacle_type: Union[Literal['rect', 'circle'], None]='rect') -> bool:
    """
    Check if the current configuration of the arm is in collision with the obstacle.
    """
    points = arm.points
    collision = False
    for k in range(len(points)-1):
        if obstacle is not None and obstacle_type == 'rect':
            assert len(obstacle) == 4, "The rectangle must be defined with four parameters: x,y, w, h"
            is_colliding = detect_collision_rect([points[k], points[k+1]], obstacle)
            if is_colliding:
                collision = True
                break
        elif obstacle is not None and obstacle_type == 'circle':
            min_dist_arr = []
            assert len(obstacle) == 3, "The circle must be defined with three parameters: x, y, radius"
            is_colliding, min_dist_k = detect_collision_circle([points[k], points[k+1]], obstacle)
            min_dist_arr.append(min_dist_k)
            if is_colliding:
                collision = True
                break
        else:
            print(f"Obstacle of type '{obstacle_type}' not defined.")
            print("You must specify one of either 'rect' or 'circle'.")
    min_dist = min(min_dist_arr) if obstacle_type == 'circle' and len(min_dist_arr) > 0 else np.inf
    if obstacle_type == 'circle':
        return collision, min_dist
    return collision

# get workspace
def get_workspace(arm: TwoRArm, M_wksp:int=100) -> Tuple[NDArray, NDArray]:
    """
    Compute all points on the robot at each configuration and also the end-effector points.
    Returns:
        all_points: (M*M, 3, 2) array of all robot points for each configuration
            The sizing is chosen to be M*M, 3, 2 because there are:
            - M choices for theta1 and M choices for theta2, 
            - Three key points along the arm:
                * Base (x0, y0)
                * Joint 1 (x1, y1)
                * End-effector (x2, y2), and
            - 2 coordinates (x, y) for each point
        workspace_ee: (M*M, 2) array of end-effector positions for each configuration
    """
    theta_vals = np.linspace(arm.joint_limits[0], arm.joint_limits[1], M_wksp)
    all_points = np.zeros((M_wksp * M_wksp, 3, 2))
    workspace_ee = np.zeros((M_wksp * M_wksp, 2))

    idx = 0
    for th1 in theta_vals:
        for th2 in theta_vals:
            arm.update_joints([th1, th2])
            all_points[idx, :, :] = np.array(arm.points)
            workspace_ee[idx, :] = arm.points[-1]
            idx += 1

    return all_points, workspace_ee

# for plotting link traces
def plot_link_traces(arm:TwoRArm, obstacle: List[float], M:int=4):
    """
    Samples joint space and plots the traces of both links
    """
    theta_vals = np.linspace(arm.joint_limits[0], arm.joint_limits[1],  M)

    plt.figure()
    for th1 in theta_vals:
        for th2 in theta_vals:
            arm.update_joints([th1, th2])
            pts = np.array(arm.points)  

            # Plot link 1 trace (joint 0 -- link 1 -- joint 1)
            plt.plot([pts[0,0], pts[1,0]], [pts[0,1], pts[1,1]], color='k', alpha=0.05)

            # Plot link 2 trace (joint 1 -- link 2 -- end-effector)
            plt.plot([pts[1,0], pts[2,0]], [pts[1,1], pts[2,1]], color='k', alpha=0.05)

    # Draw obstacle according to type
    ax = plt.gca()
    if len(obstacle) == 4:
        x_corner, y_corner, width, height = obstacle
        x_min = x_corner 
        x_max = x_corner + width 
        y_min = y_corner
        y_max = y_corner + height
        rectangle = Rectangle((x_min, y_min), width, height, color='gray', alpha=0.6)
        ax.add_patch(rectangle)
    if len(obstacle) == 3:
        center_x, center_y, radius = obstacle
        circle = Circle(xy=[center_x, center_y], radius=radius, color='gray', alpha=0.6)
        ax.add_patch(circle)

    plt.gca().set_aspect("equal")
    ax.set_xlabel(r'$x$')
    ax.set_ylabel(r'$y$')

def plot_cspace(grid: np.ndarray, 
                joint_limits: List[float], 
                route: Union[List[Tuple[int, int]], None] = None, 
                use_theta: Union[bool, None]=True,
                grid_to_config_map: Union[Dict[Tuple[int, int], List[float]], None]=None):
    """
    Plot a 2D C-space (binary grid) using rectangle patches.
    If use_theta=True, axes show joint angles; otherwise, show grid indices.
    """
    M = grid.shape[0]
    _, ax = plt.subplots()
    ax.set_aspect('equal')

    if use_theta:
        if not grid_to_config_map:
            theta_vals = np.linspace(joint_limits[0], joint_limits[1], M)
        else:
            # If grid_to_config_map is provided, extract theta_vals from its values
            # grid_to_config_map is a dict mapping (i,j) -> [theta1, theta2]
            # We want to get sorted unique theta values for each joint
            theta_vals = sorted(set([v[0] for v in grid_to_config_map.values()]))
        cell_w = (theta_vals[-1] - theta_vals[0]) / M
        cell_h = cell_w
        to_coords = lambda i, j: (theta_vals[i], theta_vals[j])
    else:
        cell_w = cell_h = 1
        to_coords = lambda i, j: (i, j)

    # Simplify route logic
    for i in range(M):
        for j in range(M):
            x, y = to_coords(i, j)
            color = 'white'
            alpha = 0.3
            if grid[i, j] == 1:
                color = 'black'
                alpha = 1.0
            if route and (i, j) in route:
                if (i, j) == route[0]:
                    color = START_COLOR
                    alpha = 0.9
                    ax.text(x + cell_w / 2, y + cell_h / 2, r'${q}_{\mathrm{start}}$',  color=START_LABEL_COLOR, ha='center', va='center')
                elif (i, j) == route[-1]:
                    color = GOAL_COLOR
                    alpha = 0.9
                    ax.text(x + cell_w / 2, y + cell_h / 2, r'${q}_{\mathrm{goal}}$',  color=GOAL_LABEL_COLOR, ha='center', va='center')
                else:
                    if grid[i, j] == 1:
                        color = INVALID_PATH_COLOR
                    color = VALID_PATH_COLOR
                    alpha = 1.
            ax.add_patch(Rectangle((x, y), cell_w, cell_h, facecolor=color, edgecolor='none', alpha=alpha))

    # Labels
    free_indices = np.argwhere(grid == 0)
    occ_indices = np.argwhere(grid > 0)

    if free_indices.size > 0:
        mean_i, mean_j = free_indices.mean(axis=0)
        x, y = to_coords(int(mean_i)-20, int(mean_j)-40)
        ax.text(x, y, r'$\mathcal{Q}_{\mathrm{free}}$',  color='k', ha='center', va='center')

    if occ_indices.size > 0:
        mean_i, mean_j = occ_indices.mean(axis=0)
        x, y = to_coords(int(mean_i), int(mean_j))
        ax.text(x, y, r'$\mathcal{Q}_{\mathrm{obs}}$',  color="#FF7575", ha='center', va='center')

    # Outer label
    # Set ticks to show -pi, -pi/2, 0, pi/2, pi
    tick_positions = [
        joint_limits[0],
        joint_limits[0] + (joint_limits[1] - joint_limits[0]) / 4,
        joint_limits[0] + (joint_limits[1] - joint_limits[0]) / 2,
        joint_limits[1] - (joint_limits[1] - joint_limits[0]) / 4,
        joint_limits[1]
    ]
    tick_labels = [pi_frac_label(tp) for tp in tick_positions]

    if use_theta:
        # Add label for configuration space outside the figure
        ax.text(
            (joint_limits[0] + joint_limits[1]) / 2,
            joint_limits[1] + 0.02 * (joint_limits[1] - joint_limits[0]),
            r'$\mathcal{Q}$',
            
            ha='center',
            va='bottom'
        )
        
        ax.set_xticks(tick_positions)
        ax.set_yticks(tick_positions)
        ax.set_xticklabels(tick_labels)
        ax.set_yticklabels(tick_labels)
        ax.set_xlabel(r'$\theta_1$ (rad)')
        ax.set_ylabel(r'$\theta_2$ (rad)')
    else:
        # ax.text(
        # M / 2,
        # M + 0.02 * M,
        # r'$\mathcal{Q}$',
        # ha='center',
        # va='bottom'
        # )
        ax.set_xlim(0, M)
        ax.set_ylim(0, M)
        ax.set_xlabel(r'$i$')
        ax.set_ylabel(r'$j$')

        # Primary grid
        ax.grid(True, which='major', linestyle='--', linewidth=0.5, alpha=0.7)
        ax.grid(True, which='minor', linestyle=':', linewidth=0.3, alpha=0.6)
        ax.minorticks_on()

        # Secondary axes for theta values
        # Create a twin x and y axis
        ax_top = ax.secondary_xaxis('top')
        ax_right = ax.secondary_yaxis('right')

        # Set the same tick positions
        theta_ticks = np.linspace(joint_limits[0], joint_limits[1], 5)
        i_ticks = [theta_to_i(t, joint_limits) for t in theta_ticks]
        theta_labels = [pi_frac_label(t) for t in theta_ticks]

        ax_top.set_xticks(i_ticks)
        ax_top.set_xticklabels(theta_labels)
        ax_top.set_xlabel(r'$\theta_1$ (rad)', color=THETA_LABEL)

        ax_top.set_xticks(i_ticks)
        ax_top.set_xticklabels(theta_labels, color=THETA_LABEL)

        ax_right.set_yticks(i_ticks)
        ax_right.set_yticklabels(theta_labels, color=THETA_LABEL)
        ax_right.set_ylabel(r'$\theta_2$ (rad)', color=THETA_LABEL)

    plt.tight_layout()


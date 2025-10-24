#!/usr/bin/env python3
"""
Parameters file for robotics planning algorithms.

Author: Clinton Enwerem
Developed for the course ENEE467, Robotics Projects Laboratory, Fall 2025, University of Maryland, College Park, MD.
"""
import numpy as np

# Obstacle parameters
RECT_OBS = [0.05, 1.25, 0.75, 0.75]  # bottom-left x, bottom-left y, w, h
RECT_OBS_ASTAR = [1.5, 0.0, 0.75, 0.75]  # bottom-left x, bottom-left y, w, h

# Compute CIRC_OBS from RECT_OBS
rect_x, rect_y, rect_w, rect_h = RECT_OBS
circ_x = rect_x + rect_w / 2
circ_y = rect_y + rect_h / 2
circ_r = min(rect_w, rect_h) / 2
CIRC_OBS = [circ_x, circ_y, circ_r]

# compute CIRC_OBS_ASTAR from RECT_OBS_ASTAR
rect_x_astar, rect_y_astar, rect_w_astar, rect_h_astar = RECT_OBS_ASTAR
circ_x_astar = rect_x_astar + rect_w_astar / 2
circ_y_astar = rect_y_astar + rect_h_astar / 2
circ_r_astar = min(rect_w_astar, rect_h_astar) / 2
CIRC_OBS_ASTAR = [circ_x_astar, circ_y_astar, circ_r_astar]

# Grid and workspace parameters
M_WKSP = 50   # number of samples along each dimension of the workspace
M = 100       # number of samples along each dimension of the occupancy map (grid, G)

# Cell state constants
FREE = 0
OBSTACLE = 1
CLOSED = 2
OPEN = 3
START = 4
GOAL = 5
PATH = 6
UNUSED = 7

# obstacle detection tolerance
EPS_OBS = 5.5e-2

# Robot arm parameters
LINK_LENGTHS = [1.0, 1.0]  # lengths of the
JOINT_LIMITS = [-np.pi/2, np.pi/2]  # joint limits in radians

# Figure parameters
SHOW_FIG = True

# Plotting
GOAL_COLOR = "#000"
START_COLOR = "#000"
START_LABEL_COLOR = "#035189"
GOAL_LABEL_COLOR = "#2A6500"
VALID_PATH_COLOR = "#4616a0"
INVALID_PATH_COLOR = "#ff0000"
THETA_LABEL = "#9B5500" 
FONTSIZE = 28

# C-space tree plotting
NODE_COLOR = "#000"
FONTSIZE_TREE = 35
LINEWIDTH_TREE = 4
NODE_MARKER_SIZE = 12
LINEWIDTH_PATH = LINEWIDTH_TREE + 0.5
COST_LABEL_FONTSIZE = 14
OBS_COLOR = "gray"  # gray
PATH_COLOR = "#FD3DB5"# "#E400F9" #B39EB5"  # light purple
COST_LABEL_MARGIN = 0.1  # margin for cost label placement
COST_LABEL_COLOR = "#4A4A4A"  # bright orange
RANDOM_SEED = 42  # for reproducibility

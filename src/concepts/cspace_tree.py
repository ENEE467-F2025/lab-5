#!/usr/bin/env python3
"""
Simple script for generating a C-space tree for a 2-link planar robot arm in an environment with a rectangular/circular obstacle.

Author: Clinton Enwerem
Developed for the course ENEE467, Robotics Projects Laboratory, Fall 2025, University of Maryland, College Park, MD.
"""
import argparse
import random
import numpy as np
import matplotlib.pyplot as plt
from utils.rrt_star import RRTStar2R, plot_tree_in_cspace, print_tree_metadata
from utils.params import (
    LINK_LENGTHS, JOINT_LIMITS, RECT_OBS, CIRC_OBS,
    LINEWIDTH_TREE, LINEWIDTH_PATH, NODE_MARKER_SIZE,
    FONTSIZE_TREE, OBS_COLOR, PATH_COLOR, RANDOM_SEED
)
from utils.helpers import (
    TwoRArm, parse_single_expression, get_cspace_grid, ensure_dir
)
import os, pathlib

# plotting style
plt.rcParams["font.size"] = FONTSIZE_TREE
plt.rcParams["axes.labelsize"] = FONTSIZE_TREE
plt.rcParams["xtick.labelsize"] = FONTSIZE_TREE
plt.rcParams["ytick.labelsize"] = FONTSIZE_TREE
plt.rcParams["legend.fontsize"] = FONTSIZE_TREE
plt.rcParams["axes.titlesize"] = FONTSIZE_TREE
plt.rcParams["text.usetex"] = False
rc = {"font.family" : "serif", 
      "mathtext.fontset" : "stix"}
plt.rcParams.update(rc)
plt.rcParams["font.serif"] = ["Times New Roman"] + plt.rcParams["font.serif"]

def main():
    parser = argparse.ArgumentParser(description="RRT* planner for 2R robot")
    parser.add_argument('--qstart', type=parse_single_expression, nargs=2, 
                       default=[0.0, 0.0], help='Start joint angles [q1, q2]')
    parser.add_argument('--qgoal', type=parse_single_expression, nargs=2, 
                       default=[np.pi/6, np.pi/4], help='Goal joint angles [q1, q2]')
    parser.add_argument('--joint_limits', type=parse_single_expression, nargs=2, 
                       default=JOINT_LIMITS, help='Joint limits [q_min, q_max]')
    parser.add_argument('--obstacle_type', type=str, default='rect', 
                       choices=['circle', 'rect'], help='Type of obstacle: circle or rect')
    parser.add_argument('--obstacle_params', type=float, nargs='+', 
                       help='Obstacle parameters (rect: [x,y,w,h], circle: [x,y,r])')
    parser.add_argument('--expand_dist', type=float, default=0.3, 
                       help='Distance to expand tree at each step')
    parser.add_argument('--path_resolution', type=float, default=0.1, 
                       help='Resolution of the path, measured in radians')
    parser.add_argument('--max_iter', type=int, default=100, 
                       help='Maximum number of iterations to build the tree')
    parser.add_argument('--connect_circle_dist', type=float, default=20, help='Radius to connect nodes, measured in node count')
    parser.add_argument('--goal_sample_rate', type=float, default=0.3, help='Probability of sampling the goal')
    parser.add_argument('--show_costs', action='store_true', help='Show cost values for nodes')
    parser.add_argument('--generate_path', action='store_true', help='Generate and show path')
    parser.add_argument('--seed', type=int, default=42, help='Random seed for reproducibility')
    parser.add_argument('--verbose', action='store_true', default=True)

    args = parser.parse_args()

    # Set random seeds for reproducibility
    np.random.seed(args.seed)
    random.seed(args.seed)

    # Set up obstacle parameters
    if args.obstacle_params:
        obstacle = args.obstacle_params
    else:
        obstacle = RECT_OBS if args.obstacle_type == 'rect' else CIRC_OBS
    
    # Create robot arm
    arm = TwoRArm(link_lengths=LINK_LENGTHS, joint_limits=args.joint_limits)
    
    # Create RRT* planner
    planner = RRTStar2R(
        start=args.qstart,
        goal=args.qgoal,
        obstacles=[obstacle],  
        expand_dist=args.expand_dist,
        path_resolution=args.path_resolution,
        max_iter=args.max_iter,
        connect_circle_dist=args.connect_circle_dist,
        goal_sample_rate=args.goal_sample_rate,
        check_collision=True,
        arm=arm,
        obstacle_type=args.obstacle_type,
    )
    
    # run planning
    path = None
    if args.generate_path:
        path = planner.plan()
    else:
        # build the tree without finding complete path to goal
        planner.build_tree()
    
    # Print tree metadata
    print_tree_metadata(planner.config_tree, path, generate_path=args.generate_path, show_costs=args.show_costs)
    
    if path is None and args.generate_path:
        print("No path found.")
    elif path is not None:
        print("Path found:")
        for i, q in enumerate(path):
            print(f"Waypoint {i}: [{q[0]:.3f}, {q[1]:.3f}]")
    
    # create C-space visualization
    fig, ax = plt.subplots(figsize=(12, 11))
    ax.set_aspect('equal')
    
    # plot the C-space with obstacles
    if args.obstacle_type=="circle":
        grid, _, _ = get_cspace_grid(arm, obstacle, M=50, 
                                                obstacle_type=args.obstacle_type, 
                                                check_collision=True)
    elif args.obstacle_type=="rect":
        grid, _ = get_cspace_grid(arm, obstacle, M=50, 
                                                obstacle_type=args.obstacle_type, 
                                                check_collision=True)
    else:
        raise ValueError(f"Unknown obstacle type: {args.obstacle_type}")
    
    # plot obstacle regions in C-space
    M = grid.shape[0]
    theta_vals = np.linspace(args.joint_limits[0], args.joint_limits[1], M)
    cell_w = (theta_vals[-1] - theta_vals[0]) / M
    cell_h = cell_w
    
    # draw obstacle cells
    for i in range(M):
        for j in range(M):
            if grid[i, j] == 1:  
                x, y = theta_vals[i], theta_vals[j]
                from matplotlib.patches import Rectangle
                ax.add_patch(Rectangle((x, y), cell_w, cell_h, 
                                     facecolor=OBS_COLOR, alpha=0.9, 
                                     edgecolor='none', zorder=0))
    
    # plot the tree on top
    plot_tree_in_cspace(planner.config_tree, args.joint_limits, ax, 
                       show_costs=args.show_costs, config_path=path)
    
    # mark start and goal
    ax.plot(args.qstart[0], args.qstart[1], 'ro', markersize=NODE_MARKER_SIZE, zorder=5)
    ax.plot(args.qgoal[0], args.qgoal[1], 'bs', markersize=NODE_MARKER_SIZE, zorder=5)

    # add obstacle region label in the plot area
    if np.any(grid == 1): 
        obs_y, obs_x = np.where(grid == 1)
        if len(obs_y) > 0:
            mid_obs_x = theta_vals[obs_x[len(obs_x)//2]]
            mid_obs_y = theta_vals[obs_y[len(obs_y)//2]]
            ax.text(mid_obs_x, mid_obs_y, r'$\mathcal{Q}_{\mathrm{obs}}$', 
                    fontsize=FONTSIZE_TREE, color='black', fontweight='bold',
                    ha='center', va='center')
    
    # add legend entries only for tree and path
    legend_elements = [
        plt.Line2D([0], [0], color='g', linewidth=LINEWIDTH_TREE, marker='o', markerfacecolor='k', markeredgecolor='k', markersize=NODE_MARKER_SIZE, label='C-space Tree'),
        plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='red', markersize=NODE_MARKER_SIZE+4, label=r'$q_{\mathrm{start}}$'),
        plt.Line2D([0], [0], marker='s', color='w', markerfacecolor='blue', markersize=NODE_MARKER_SIZE+4, label=r'$q_{\mathrm{goal}}$')
    ]
    
    ncols = 3
    anchor_xy = (0.5, 0.9)
    padding = 2
    pad_val = 1.0
    hpad_val = 0.5
    w_pad_val = 0.5

    if path is not None:
        legend_elements.append(plt.Line2D([0], [0], color=PATH_COLOR, 
                                        linewidth=LINEWIDTH_PATH, label='Path'))
        ncols = 2

    ax.legend(handles=legend_elements, loc='lower center', bbox_to_anchor=anchor_xy,
                  fancybox=False, shadow=False, ncol=ncols, borderaxespad=padding)

    plt.tight_layout(pad=pad_val, h_pad=hpad_val, w_pad=w_pad_val)
    plt.show()

if __name__ == "__main__":
    main()
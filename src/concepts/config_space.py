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
Simple script to compute and visualize the configuration space (C-space) of a 2R planar robot arm
in the presence of either a rectangular or circular obstacle in its workspace.

Author: Clinton Enwerem
Developed for the course ENEE467, Robotics Projects Laboratory, Fall 2025, University of Maryland, College Park, MD.
"""
#!/usr/bin/env python3
from utils.helpers import *
import os, pathlib
import argparse

#
if __name__ == '__main__':

    # create argparse object
    parser = argparse.ArgumentParser(description="A simple CLI program for visualizing the C-space and workspace of a 2R planar kinematic chain.")
    parser.add_argument(
        '--link_lengths',
        type=float,
        nargs='+',
        default=LINK_LENGTHS,
        help='Space-separated link lengths of the planar 2R robot arm in meters. Default: [1.0, 1.0]'
    )
    parser.add_argument(
        '--thetas',
        type=parse_single_expression,
        nargs=2,
        default=[np.pi/6, np.pi/4],
        help="Space-separated NumPy expressions, e.g., np.pi/6 np.pi/4 of joint angles to set. \
             To pass negative argument values, wrap the negative argument value in double quotes, prepending a space before the minus sign."
    )
    parser.add_argument(
        '--joint_limits',
        type=parse_single_expression,
        nargs=2,  
        default=JOINT_LIMITS,
        help="Space-separated NumPy expressions of joint limits in radians, e.g. -np.pi/2 np.pi/2. \
             To pass negative argument values, wrap the negative argument value in double quotes, prepending a space before the minus sign."
    )
    parser.add_argument(
        '--M',
        type=int,
        default=100,
        help='Number of joint variables to sample along each dimension of the occupancy map (grid, G)'
    )
    parser.add_argument(
        '--rect_params',
        type=float,
        nargs='+',
        default=RECT_OBS,
        help='Bottom-left xy coordinates, width, and height of rectangular obstacle.'
    )
    parser.add_argument(
        '--circ_params',
        type=float,
        nargs='+',
        default=CIRC_OBS,
        help="Center xy coordinates and radius of circular obstacle to spawn in robot's workspace."
    )
    parser.add_argument(
        '--obstacle_type',
        type=str,
        default='rect',
        choices=['rect', 'circle'],
        help="The obstacle type to spawn in the robot's workspace. "
    )
    parser.add_argument(
        '--save_fig',
        type=str2bool,
nargs='?',
        default=True,
        help="Whether to save the generated figure to disk. "
    )
    parser.add_argument(
        '--show_fig',
        type=str2bool,
nargs='?',
        default=SHOW_FIG,
        help="Whether to show the figure. "
    )

    # get params
    args = parser.parse_args()
    link_lengths = args.link_lengths
    thetas = args.thetas
    joint_limits = args.joint_limits
    theta1 = thetas[0]
    theta2 = thetas[1]
    joint_limits = joint_limits
    M = args.M
    obstacle_type = args.obstacle_type
    save_fig = args.save_fig
    show_fig = args.show_fig

    # construct a TwoRArm object using default parameters or argparse
    assert (joint_limits[0] < theta1 < joint_limits[1]) and (joint_limits[0] < theta2 < joint_limits[1]), f"Joint angles out of bounds: [{joint_limits[0]:.2f}, {joint_limits[1]:.2f}]"
    arm = TwoRArm(link_lengths=link_lengths, joint_limits=joint_limits)

    # Rectangular obstacle [x_corner, y_corner, width, height]
    if obstacle_type == 'rect':
        x_corner, y_corner, W_obs, H_obs = args.rect_params
        rect_obstacle = [x_corner, y_corner, W_obs, H_obs]
        grid_rect, _ = get_cspace_grid(arm, rect_obstacle, M=M, obstacle_type=obstacle_type)

    # Circular obstacle [center_x, center_y, radius]
    if obstacle_type == 'circle':
        circular_obstacle =  args.circ_params
        grid_circ, min_dists, _ = get_cspace_grid(arm, circular_obstacle, M=M, obstacle_type=obstacle_type)

    # Get points in workspace
    all_points, workspace_ee = get_workspace(arm, M_wksp = M_WKSP)

    ###### plots #########
    if obstacle_type == 'rect':
        plot_2r_with_rect(theta1, theta2, link_lengths, rect_obstacle)
    elif obstacle_type == 'circle':
        plot_2r_with_circle(theta1, theta2, link_lengths, circular_obstacle)
    else:
        print(f"Obstacle of type '{obstacle_type}' not defined. Please specify either \
              'rect' or 'circle' with appropriate obstacle sizes. ")

    # workspace (rectangle)
    ax = plt.gca()
    plot_workspace(workspace_ee=workspace_ee, all_points=all_points, ax=ax)
    if obstacle_type == 'rect':
        if save_fig:
            wksp_rect_base_dir = os.path.abspath(os.path.join(pathlib.Path(os.path.join(os.path.dirname(__file__))).parent.parent, "docs", "figures"))
            if os.path.isdir(wksp_rect_base_dir):
                wksp_rect_sp = os.path.join(wksp_rect_base_dir, "wksp_rect")
            else:
                wksp_rect_sp = os.path.join(os.path.dirname(os.path.abspath(__file__)), "wksp_rect")
            ensure_dir(os.path.dirname(wksp_rect_sp))
            # plt.savefig(wksp_rect_sp+'.png', bbox_inches='tight')

    if obstacle_type == 'circle':
        plot_workspace(workspace_ee=workspace_ee, all_points=all_points, ax=ax)
        if save_fig:
            wksp_circle_base_dir = os.path.abspath(os.path.join(pathlib.Path(os.path.join(os.path.dirname(__file__))).parent.parent, "docs", "figures"))
            if os.path.isdir(wksp_circle_base_dir):
                wksp_circle_sp = os.path.join(wksp_circle_base_dir, "wksp_circle")
            else:
                wksp_circle_sp = os.path.join(os.path.dirname(os.path.abspath(__file__)), "wksp_circle")
            ensure_dir(os.path.dirname(wksp_circle_sp))
            # plt.savefig(wksp_circle_sp+'.png', bbox_inches='tight')

    # config space obstacle (rect)
    plt.close('all')
    if obstacle_type == 'rect':
        plot_cspace(grid=grid_rect, joint_limits=joint_limits)
        if save_fig:
            cspace_obs_rect_base_dir = os.path.abspath(os.path.join(pathlib.Path(os.path.join(os.path.dirname(__file__))).parent.parent, "docs", "figures"))
            if os.path.isdir(cspace_obs_rect_base_dir):
                cspace_obs_rect_sp = os.path.join(cspace_obs_rect_base_dir, "cspace_obs_rect")
            else:
                cspace_obs_rect_sp = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cspace_obs_rect")
            ensure_dir(os.path.dirname(cspace_obs_rect_sp))
            plt.savefig(cspace_obs_rect_sp+'.png', bbox_inches='tight')

    # config space obstacle (circle)
    if obstacle_type == 'circle':
        plot_cspace(grid=grid_circ, joint_limits=joint_limits)
        if save_fig:
            cspace_obs_circ_base_dir = os.path.abspath(os.path.join(pathlib.Path(os.path.join(os.path.dirname(__file__))).parent.parent, "docs", "figures"))
            if os.path.isdir(cspace_obs_circ_base_dir):
                cspace_obs_circ_sp = os.path.join(cspace_obs_circ_base_dir, "cspace_obs_circle")
            else:
                cspace_obs_circ_sp = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cspace_obs_circle")
            ensure_dir(os.path.dirname(cspace_obs_circ_sp))
            plt.savefig(cspace_obs_circ_sp+'.png', bbox_inches='tight')

    if show_fig:
        plt.show()

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
Simple script implementing basic graph algorithms and data structures.

Adapted from: https://github.com/AtsushiSakai/PythonRobotics/blob/master/ArmNavigation/arm_obstacle_navigation/arm_obstacle_navigation.py

Author: Clinton Enwerem
Developed for the course ENEE467, Robotics Projects Laboratory, Fall 2025, University of Maryland, College Park, MD.
"""
from utils.helpers import *
from utils.params import *
import os, pathlib

M_copy  = M

#
def main():

    # create argparse object
    parser = argparse.ArgumentParser(description="A simple CLI program for A* path planning.")
    parser.add_argument(
        '--link_lengths',
        type=float,
        nargs='+',
        default=LINK_LENGTHS,
        help='Space-separated link lengths of the planar 2R robot arm in meters. Default: [1.0, 1.0]'
    )
    parser.add_argument(
        '--start_idxs',
        type=int,
        nargs=2,
        default=[20, 50],
        help="Grid indices of the start joint configuration."
    )
    parser.add_argument(
        '--goal_idxs',
        type=int,
        nargs=2,
        default=[78, 50],
        help="Grid indices of the goal joint configuration."
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
        default=M_copy,
        help='Number of joint variables to sample along each dimension of the occupancy map (grid, G)'
    )
    parser.add_argument(
        '--rect_params',
        type=float,
        nargs='+',
        default=RECT_OBS_ASTAR,
        help='Bottom-left xy coordinates, width, and height of rectangular obstacle.'
    )
    parser.add_argument(
        '--circ_params',
        type=float,
        nargs='+',
        default=CIRC_OBS_ASTAR,
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
        '--verbose',
        type=str2bool,
        nargs='?',
        default=True,
        help="Whether to print verbose output."
    )
    parser.add_argument(
        '--save_fig',
        type=str2bool,
        nargs='?',
        default=False,
        help="Whether to save the generated figure to disk."
    )
    parser.add_argument(
        '--show_fig',
        type=str2bool,
        nargs='?',
        default=True,
        help="Whether to show the generated figure."
    )
    parser.add_argument(
        '--check_collision',
        type=str2bool,
        nargs='?',
        default=True,
        help="If true, the grid is a proper binary occupancy grid; if false, then all configurations are treated as belonging to Q_free."
    )

    # get params
    args = parser.parse_args()
    link_lengths = args.link_lengths
    start_idxs = tuple(args.start_idxs)
    goal_idxs = tuple(args.goal_idxs)
    joint_limits = args.joint_limits
    joint_limits = joint_limits
    M_grid = args.M
    obstacle_type = args.obstacle_type
    verbose = args.verbose
    save_fig = args.save_fig
    show_fig = args.show_fig
    check_collision = args.check_collision

    # Construct the arm
    arm = TwoRArm(link_lengths=link_lengths, joint_limits=joint_limits)

    # Rectangular obstacle [x_corner, y_corner, width, height]
    if obstacle_type == 'rect':
        x_corner, y_corner, W_obs, H_obs = args.rect_params
        rect_obstacle = [x_corner, y_corner, W_obs, H_obs]
        grid, grid_to_config_map = get_cspace_grid(arm, rect_obstacle, M=M_grid, obstacle_type=obstacle_type, check_collision=check_collision)
        occupancy_grid = grid.copy()

    # Circular obstacle [center_x, center_y, radius]
    if obstacle_type == 'circle':
        circular_obstacle =  args.circ_params
        grid, _, grid_to_config_map = get_cspace_grid(arm, circular_obstacle, M=M_grid, obstacle_type=obstacle_type, check_collision=check_collision)
        occupancy_grid = grid.copy()


    # Get grid
    all_points, workspace_ee = get_workspace(arm, M_wksp = M_WKSP)
    # run A* on a working copy so astar_torus can annotate its grid for visualization
    route, min_cost = astar_torus(grid.copy(), start_idxs, goal_idxs, M_grid)
    if verbose:
        if route:
            node_thetas = []
            lower_limit, upper_limit = joint_limits
            # print("Route (joint space):")
            for node in route:
                # Convert grid indices to actual joint angles using the joint limits
                node_theta1, node_theta2 = grid_to_config_map[node]
                node_thetas.append((node_theta1, node_theta2))
                # report the true occupancy (0/1) from the preserved occupancy_grid
                is_colliding_flag = int(occupancy_grid[node]) if 'occupancy_grid' in locals() else int(grid[node])
                print(f"  Cell: (i, j) = ({node[0]}, {node[1]}) -> (theta1, theta2) = ({node_theta1:.2f}, {node_theta2:.2f}) rad")
                # print(f"  Cell: (i, j) = ({node[0]}, {node[1]}) -> (theta1, theta2) = ({node_theta1:.2f}, {node_theta2:.2f}) rad, is_colliding = {is_colliding_flag}")
            print(f"Minimum cost from start to goal: {int(min_cost)}")
        else:
            print("No path found!")

    # Get points in workspace
    all_points, workspace_ee = get_workspace(arm, M_wksp = M_WKSP)

    ###### plots #########
    if obstacle_type == 'rect':
        if len(route) == 0:
            route = [start_idxs, goal_idxs]
        plot_2r_path_with_rect(route=route, arm=arm, rect=rect_obstacle, M=M_grid, embellish=False)
    elif obstacle_type == 'circle':
        plot_2r_path_with_circle(route=route, arm=arm, circle=circular_obstacle, M=M_grid, embellish=False)
    else:
        print(f"Obstacle of type '{obstacle_type}' not defined. Please specify either \
              'rect' or 'circle' with appropriate obstacle sizes. ")

    # workspace (rectangle)
    ax = plt.gca()
    if obstacle_type == 'rect':
        plot_workspace(workspace_ee=workspace_ee, all_points=all_points, ax=ax, embellish=False)
        if save_fig:
            # 2r in workspace
            astar_path_rect_base_dir = os.path.abspath(os.path.join(pathlib.Path(os.path.join(os.path.dirname(__file__))).parent.parent, "docs", "figures"))
            if os.path.isdir(astar_path_rect_base_dir):
                astar_path_rect_sp = os.path.join(astar_path_rect_base_dir, "astar_path_rect")
            else:
                astar_path_rect_sp = os.path.join(os.path.dirname(os.path.abspath(__file__)), "astar_path_rect")
            ensure_dir(os.path.dirname(astar_path_rect_sp))
            plt.savefig(astar_path_rect_sp+'.png', bbox_inches='tight')
            plt.savefig(astar_path_rect_sp+'.pdf', bbox_inches='tight')
        if show_fig:
            plt.show(block=True)
        if save_fig:
            plt.close()

            # C-space with route
            plt.clf()
            plot_cspace(grid=grid, joint_limits=joint_limits, route=route, grid_to_config_map=grid_to_config_map)
            cspace_with_route_rect_base_dir = os.path.abspath(os.path.join(pathlib.Path(os.path.join(os.path.dirname(__file__))).parent.parent, "docs", "figures"))
            if os.path.isdir(cspace_with_route_rect_base_dir):
                cspace_with_route_rect_sp = os.path.join(cspace_with_route_rect_base_dir, "cspace_with_route_rect")
            else:
                cspace_with_route_rect_sp = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cspace_with_route_rect")
            ensure_dir(os.path.dirname(cspace_with_route_rect_sp))
            plt.savefig(cspace_with_route_rect_sp+'.png', bbox_inches='tight')
            plt.savefig(cspace_with_route_rect_sp+'.pdf', bbox_inches='tight')
            plt.close()

            # Grid with route
            plt.clf()
            plot_cspace(grid=grid, joint_limits=joint_limits, route=route, use_theta=False, grid_to_config_map=grid_to_config_map)
            grid_with_route_rect_base_dir = os.path.abspath(os.path.join(pathlib.Path(os.path.join(os.path.dirname(__file__))).parent.parent, "docs", "figures"))
            if os.path.isdir(grid_with_route_rect_base_dir):
                grid_with_route_rect_sp = os.path.join(grid_with_route_rect_base_dir, "grid_with_route_rect")
            else:
                grid_with_route_rect_sp = os.path.join(os.path.dirname(os.path.abspath(__file__)), "grid_with_route_rect")
            ensure_dir(os.path.dirname(grid_with_route_rect_sp))
            plt.savefig(grid_with_route_rect_sp+'.png', bbox_inches='tight')
            plt.savefig(grid_with_route_rect_sp+'.pdf', bbox_inches='tight')
            plt.close()


    if obstacle_type == 'circle':
        plot_workspace(workspace_ee=workspace_ee, all_points=all_points, ax=ax, embellish=False)
        if save_fig:
            # 2r in workspace
            astar_path_circle_base_dir = os.path.abspath(os.path.join(pathlib.Path(os.path.join(os.path.dirname(__file__))).parent.parent, "docs", "figures"))
            if os.path.isdir(astar_path_circle_base_dir):
                astar_path_circle_sp = os.path.join(astar_path_circle_base_dir, "astar_path_circle")
            else:
                astar_path_circle_sp = os.path.join(os.path.dirname(os.path.abspath(__file__)), "astar_path_circle")
            ensure_dir(os.path.dirname(astar_path_circle_sp))
            plt.savefig(astar_path_circle_sp+'.png', bbox_inches='tight')
            plt.savefig(astar_path_circle_sp+'.pdf', bbox_inches='tight')
        if show_fig:
            plt.show(block=True)
        if save_fig:
            
            # C-space with route
            plt.clf()
            plot_cspace(grid=grid, joint_limits=joint_limits, route=route)
            cspace_with_route_circle_base_dir = os.path.abspath(os.path.join(pathlib.Path(os.path.join(os.path.dirname(__file__))).parent.parent, "docs", "figures"))
            if os.path.isdir(cspace_with_route_circle_base_dir):
                cspace_with_route_circle_sp = os.path.join(cspace_with_route_circle_base_dir, "cspace_with_route_circle")
            else:
                cspace_with_route_circle_sp = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cspace_with_route_circle")
            ensure_dir(os.path.dirname(cspace_with_route_circle_sp))
            plt.savefig(cspace_with_route_circle_sp+'.png', bbox_inches='tight')
            plt.savefig(cspace_with_route_circle_sp+'.pdf', bbox_inches='tight')
            plt.close()

            # Grid with route
            plt.clf()
            plot_cspace(grid=grid, joint_limits=joint_limits, route=route, use_theta=False)
            grid_with_route_circle_base_dir = os.path.abspath(os.path.join(pathlib.Path(os.path.join(os.path.dirname(__file__))).parent.parent, "docs", "figures"))
            if os.path.isdir(grid_with_route_circle_base_dir):
                grid_with_route_circle_sp = os.path.join(grid_with_route_circle_base_dir, "grid_with_route_circle")     
            else:
                grid_with_route_circle_sp = os.path.join(os.path.dirname(os.path.abspath(__file__)), "grid_with_route_circle")
            ensure_dir(os.path.dirname(grid_with_route_circle_sp))
            plt.savefig(grid_with_route_circle_sp+'.png', bbox_inches='tight')
            plt.savefig(grid_with_route_circle_sp+'.pdf', bbox_inches='tight')
            plt.close()


if __name__ == '__main__':
    main()
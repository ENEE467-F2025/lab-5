#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch_ros.actions import Node
import os

def generate_launch_description():
    robot_3r_gz_dir = FindPackageShare('robot_3r_gz_sim')
    launch_planning_nodes = LaunchConfiguration("launch_planning_nodes")
    pkg_share = FindPackageShare("robot_3r_description").find("robot_3r_description")
    urdf_file = os.path.join(pkg_share, "urdf", "robot_3r.urdf.xacro")
    declared_arguments = [
        DeclareLaunchArgument(
            "launch_planning_nodes",
            default_value="true",
            description="Launch planning nodes.",
        ),
        DeclareLaunchArgument(
            "run_headless",
            default_value="true",
            description="Whether to start the simulator in headless mode.",
        ),
        DeclareLaunchArgument(
            "is_dense",
            default_value="false",
            choices=["true", "false"],
            description="Whether to use dense obstacle configuration. \n" \
            "If false, only two obstacles are spawned, else ten.", 
        )
    ]
    # Include the Gazebo launch directly and pass launch args
    view_gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([robot_3r_gz_dir, 'launch', 'robot_3r_gz.launch.py'])
        ),
        launch_arguments=[
            ('launch_rviz', 'true'),
            ('run_headless', LaunchConfiguration('run_headless'))
        ]
    )
    obstacle_publisher_node = Node(
        package="robot_3r_planners",
        condition=IfCondition(launch_planning_nodes),
        executable="simple_obstacle_publisher",
        parameters=[{
            "is_dense": LaunchConfiguration("is_dense")}],
        output="screen"
    )
    robot_geom_publisher_node = Node(
        package="robot_3r_planners",
        condition=IfCondition(launch_planning_nodes),
        parameters=[{
                "robot_description": Command(["xacro ", urdf_file])
            }],
        executable="robot_geom_publisher",
        output="screen"
    )

    return LaunchDescription(declared_arguments + [
        view_gz_launch,
        obstacle_publisher_node,
        robot_geom_publisher_node
    ])

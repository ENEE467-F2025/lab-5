#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    robot_3r_gz_dir = FindPackageShare('robot_3r_gz_sim')

    view_gz_launch = ExecuteProcess(
        cmd=[
            'ros2', 'launch',
            PathJoinSubstitution([robot_3r_gz_dir, 'launch', 'robot_3r_gz.launch.py']),
            'launch_rviz:=true'
        ],
        output='screen'
    )

    return LaunchDescription([
        view_gz_launch,
    ])

#!/usr/bin/env python3

# Lab 5: Collision-Free Kinematic Motion Planning in ROS 2 - Part II
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

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ur3e_hande_gz_dir = FindPackageShare('ur3e_hande_gz')
    ur3e_hande_moveit_config_dir = FindPackageShare('ur3e_hande_moveit_config')

    view_gz_launch = ExecuteProcess(
        cmd=[
            'ros2', 'launch',
            PathJoinSubstitution([ur3e_hande_gz_dir, 'launch', 'view_gz.launch.py']),
            'launch_rviz:=false', "world_to_spawn:=smallbox"  
        ],
        output='screen'
    )

    demo_launch = ExecuteProcess(
        cmd=[
            'ros2', 'launch',
            PathJoinSubstitution([ur3e_hande_moveit_config_dir, 'launch', 'demo.launch.py']),
        ],
        output='screen'
    )

    return LaunchDescription([
        view_gz_launch,
        demo_launch
    ])

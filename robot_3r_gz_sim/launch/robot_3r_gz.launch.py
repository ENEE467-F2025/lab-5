#!/usr/bin/env python3
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.substitutions import Command, PathJoinSubstitution, TextSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from ament_index_python import get_package_share_directory


def generate_launch_description():
    pkg_share = FindPackageShare("robot_3r_description").find("robot_3r_description")
    urdf_file = os.path.join(pkg_share, "urdf", "robot_3r.urdf.xacro")
    launch_rviz = LaunchConfiguration("launch_rviz")
    log_level = LaunchConfiguration("log_level")
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    config = os.path.join(
        pkg_share,
        "config",
        "controllers.yaml"
    )

    declared_arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulated time.",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="warn",
            description="ROS 2 log level.",
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz2 with the robot model and TFs."
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=os.path.join(
                get_package_share_directory("robot_3r_description"),
                "rviz",
                "view_3r.rviz",
            ),
            description="Path to configuration for RViz2.",
        )]

    return LaunchDescription(declared_arguments + [
        # Start Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                ])
            ),
            launch_arguments=[("gz_args", 
                               [
                                "default.sdf",
                                TextSubstitution(text=" -v "),
                                "3",
                                TextSubstitution(text=" -r "),
                                TextSubstitution(text=" --physics-engine gz-physics-bullet-plugin")
                                # DART: gz-physics-dartsim-plugin
                                # TPE: gz-physics-tpe-plugin
                                # Bullet: gz-physics-bullet-plugin
                            ]
                            )]
                            ),
        # Robot State Publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{
                "robot_description": Command(["xacro ", urdf_file])
            },
            {"use_sim_time": use_sim_time}
            ],
            output="screen"
        ),
    
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--param-file", config, "--controller-manager", "/controller_manager"],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["threedofbot_joint_trajectory_controller", "--param-file", config, "--controller-manager", "/controller_manager"],
        ),
        # Spawn robot into Gazebo
        Node(
            package="ros_gz_sim",
            executable="create",
            output="screen",
            arguments=[
            "-topic", "robot_description",
            "-name", "robot_3r",
            "-pose", "-0.52", "0.01", "0.1", "0.0", "0.0", "-0.06",
            "--ros-args", "--log-level", "debug"
            ],
            parameters=[{"use_sim_time": use_sim_time}],
        ),

        # rviz2
        Node(
            package="rviz2",
            condition=IfCondition(launch_rviz),
            executable="rviz2",
            output="log",
            arguments=[
                "--display-config",
                rviz_config,
                "--ros-args",
                "--log-level",
                log_level,
            ],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        # ros_gz_bridge
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="ros_gz_bridge",
            output="log",
            arguments=[
                # GzSim->ROS2 Bridge
                "/clock" + "@rosgraph_msgs/msg/Clock" + "[gz.msgs.Clock",
                "--ros-args",
                "--log-level",
                log_level,
            ],
            parameters=[{"use_sim_time": use_sim_time}],
        )
    ])

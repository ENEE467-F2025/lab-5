#!/usr/bin/env python3

"""
Test script to verify Docker container setup for Lab 5, Part II.

Author: Clinton Enwerem
Developed for the course ENEE467: Robotics Projects Laboratory, Fall 2025, University of Maryland, College Park, MD.
"""

import rclpy
import pathlib, os
from rclpy.node import Node
from ament_index_python import get_package_share_directory

# Lab 5 packages
lab5_packages = [
    "pymoveit2",
    "ur3e_hande_moveit_scripts",
    "ur3e_hande_gz",
    "ur3e_hande_moveit_config",
    "ur3e_hande_description",
    "robotiq_hande_description",
    "ur3e_hande_planning_interfaces",
    "ur3e_hande_scene_manager",
    "hande_action_client",
]
lab5_pkg_share_dirs = [get_package_share_directory(pkg) for pkg in lab5_packages]

class TestDockerNode(Node):
    def __init__(self):
        super().__init__('test_docker_node')
        self.docker_timer = self.create_timer(2.0, self.test_docker_cb)

    def test_docker_cb(self):

        # Check if lab 5 packages are accessible
        for pkg, share_dir in zip(lab5_packages, lab5_pkg_share_dirs):
            if not pathlib.Path(share_dir).exists():
                self.get_logger().error(f"Package '{pkg}' not found in lab 5 workspace at {share_dir}. Docker setup may be incorrect.")
                return
            # else:
            #     self.get_logger().info(f"Package '{pkg}' found in lab 5 workspace at {share_dir}.")

        self.get_logger().info("\x1b[92mAll packages for Lab 5, Part II found. Docker setup is correct.\x1b[0m")
        self.docker_timer.cancel()

        # emulate Ctrl-C by shutting down rclpy, causing rclpy.spin() to exit
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = TestDockerNode()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()
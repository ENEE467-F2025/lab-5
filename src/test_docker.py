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
Test script to verify Docker container setup
"""

import rclpy
import pathlib, os
from rclpy.node import Node
from ament_index_python import get_package_share_directory

# Lab 5 packages
COMMON_WS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'common_ws'))

common_ws_packages = [
    "pymoveit2",
]

lab5_packages = [
    "robot_3r_bringup",
    "robot_3r_description",
    "robot_3r_moveit_config",
    "robot_3r_planners",
    "robot_3r_interfaces",
    "robot_3r_trajectory"
]
common_ws_pkg_share_dirs = [get_package_share_directory(common_pkg) for common_pkg in common_ws_packages]
lab5_pkg_share_dirs = [get_package_share_directory(pkg) for pkg in lab5_packages]

class TestDockerNode(Node):
    def __init__(self):
        super().__init__('test_docker_node')
        self.docker_timer = self.create_timer(2.0, self.test_docker_cb)

    def test_docker_cb(self):
        # Check if common workspace packages are accessible
        for pkg, share_dir in zip(common_ws_packages, common_ws_pkg_share_dirs):
            if not pathlib.Path(share_dir).exists():
                self.get_logger().error(f"Package '{pkg}' not found in common workspace at {share_dir}. Docker setup may be incorrect.")
                return

        # Check if lab 5 packages are accessible
        for pkg, share_dir in zip(lab5_packages, lab5_pkg_share_dirs):
            if not pathlib.Path(share_dir).exists():
                self.get_logger().error(f"Package '{pkg}' not found in lab 5 workspace at {share_dir}. Docker setup may be incorrect.")
                return

        self.get_logger().info("\x1b[92mAll packages for Lab 5 found. Docker setup is correct.\x1b[0m")
        self.docker_timer.cancel()

        # emulate Ctrl-C by shutting down rclpy
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = TestDockerNode()
    rclpy.spin(node)
    node.destroy_node()
    # rclpy.shutdown()

if __name__ == '__main__':
    main()
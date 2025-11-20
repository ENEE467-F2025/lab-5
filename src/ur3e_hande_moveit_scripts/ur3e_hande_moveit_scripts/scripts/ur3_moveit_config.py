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

"""
ROS 2 node to instantiate MoveIt2 interface for the UR3e-Hand-E robot.
"""

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2
from pymoveit2.robots import ur3e_hande as robot


class Ur3MoveItConfigNode(Node):
    def __init__(self):
        super().__init__("ur3_moveit_config_node")
        self.robot = robot
        # Callback group for MoveIt2
        self._cb_group = ReentrantCallbackGroup()

        # Create an instance of MoveIt2 interface
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=self.robot.joint_names(), # UR3e-Hand-E joint names
            base_link_name=self.robot.base_link_name(), # UR3e-Hand-E base link name
            end_effector_name=self.robot.end_effector_name(),   # UR3e-Hand-E EE name
            group_name=self.robot.MOVE_GROUP_ARM,            # UR3e-Hand-E group name for the arm
            callback_group=self._cb_group,         # callback group for concurrency
        )

        # set planner ID and scale down speed
        self.declare_parameter("planner_id", "RRTConnectkConfigDefault")
        self.moveit2.planner_id = (
            self.get_parameter("planner_id").get_parameter_value().string_value
        )
        self.moveit2.max_velocity = 0.2
        self.moveit2.max_acceleration = 0.2


def main(args=None):
    rclpy.init(args=args)
    node = Ur3MoveItConfigNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

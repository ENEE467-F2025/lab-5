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
Script to update the planning scene of the UR3e HandE robot using PyMoveIt2.
This version only focuses on loading, clearing, and removing objects via parameters.

Author: Clinton Enwerem
Developed for the course ENEE467: Robotics Projects Laboratory, Fall 2025, University of Maryland, College Park, MD.
"""

import os
import yaml
from threading import Thread
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from pymoveit2 import MoveIt2
from pymoveit2.robots import ur3e_hande as robot

import ament_index_python.packages as pkg

def get_config_path():
    package_share_path = pkg.get_package_share_directory('ur3e_hande_scene_manager')
    config_path = os.path.join(package_share_path, 'config', 'scene.yaml')
    return config_path

class UR3MoveItScene(Node):
    def __init__(self):
        super().__init__("scene_manager")
        # Declare parameters
        self.declare_parameter(
            "scene_config",
            value=get_config_path(),
            descriptor=ParameterDescriptor(
                description="Path to YAML file describing the scene objects",
                type=ParameterType.PARAMETER_STRING,
            ),
        )

        # Declare parameters
        self.declare_parameter("load", False, ParameterDescriptor(description="Set to True to load the scene from the config"))
        self.declare_parameter("clear", False, ParameterDescriptor(description="Set to True to clear the planning scene"))
        self.declare_parameter(
            "cancel_after",
            -1.0,
            ParameterDescriptor(
                name="cancel_after",
                type=ParameterType.PARAMETER_DOUBLE,
                description=(
                    "Seconds after which a pending clear_all_collision_objects call is cancelled."
                ),
            ),
        )
        self.declare_parameter("remove_id", "", ParameterDescriptor(description="ID of object to remove from the scene"))

        # PyMoveIt2 model
        self.robot = robot

        # Callback group for concurrency
        self.cb_group = ReentrantCallbackGroup()

        # MoveIt2 interface
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=self.robot.joint_names(),
            base_link_name=self.robot.base_link_name(),
            end_effector_name=self.robot.end_effector_name(),
            group_name=self.robot.MOVE_GROUP_ARM,
            callback_group=self.cb_group,
        )

        # Track objects
        self.objects = {}

        self.get_logger().info("Scene Manager Node ready.")

    def load_scene(self):
        config_path = self.get_parameter("scene_config").value
        if not config_path or not os.path.exists(config_path):
            self.get_logger().error(f"Scene config file not found: {config_path}")
            return

        self.get_logger().info(f"Loading scene from {config_path}")
        with open(config_path, "r") as f:
            scene = yaml.safe_load(f)

        for obj in scene.get("scene_objects", []):
            self.add_collision_object(obj)
            # Small pause so MoveIt has time to process the published CollisionObject
            time.sleep(0.05)

        self.get_logger().info(f"Loaded {len(scene.get('scene_objects', []))} objects.")

    def clear_scene(self):
        cancel_after = self.get_parameter("cancel_after").get_parameter_value().double_value
        self.get_logger().info("Clearing planning scene...")
        
        # Call clear_all_collision_objects service
        future = self.moveit2.clear_all_collision_objects()
        start_time = self.get_clock().now()
        
        # Check if future is None 
        if not future:
            self.get_logger().error("Failed to clear planning scene")
            return

        # Handle future completion or cancellation
        rate = self.create_rate(10)
        while rclpy.ok() and not future.done():
            if cancel_after >= 0.0 and (self.get_clock().now() - start_time).nanoseconds / 1e9 >= cancel_after:
                self.moveit2.cancel_clear_all_collision_objects_future(future)
                self.get_logger().info("Cancelled clear planning scene service call")
                break
            rate.sleep()

        # Process result if the future is done
        if future.done():
            success = self.moveit2.process_clear_all_collision_objects_future(future)
            if success:
                self.get_logger().info("Successfully cleared planning scene")
            else:
                self.get_logger().error("Failed to clear planning scene")
        elif future.cancelled():
            self.get_logger().info("Cancelled clear planning scene service call")
        else:
            self.get_logger().warn("Clear scene request was neither successful nor cancelled.")

    def remove_object(self, obj_id):
        if obj_id in self.objects:
            self.moveit2.remove_collision_object(id=obj_id)
            del self.objects[obj_id]
            self.get_logger().info(f"Removed '{obj_id}'")
        else:
            self.get_logger().warn(f"Tried to remove unknown object '{obj_id}'")

    # Core helpers
    def add_collision_object(self, obj):
        shape = obj["shape"]
        obj_id = obj["id"]
        pos = obj.get("position", [0, 0, 0])
        quat = obj.get("quat_xyzw", [0, 0, 0, 1])
        dims = obj.get("dimensions", [0.1, 0.1, 0.1])

        if shape == "box":
            self.moveit2.add_collision_box(id=obj_id, position=pos, quat_xyzw=quat, size=dims)
        elif shape == "sphere":
            self.moveit2.add_collision_sphere(id=obj_id, position=pos, radius=dims[0])
        elif shape == "cylinder":
            self.moveit2.add_collision_cylinder(
                id=obj_id, position=pos, quat_xyzw=quat, height=dims[0], radius=dims[1]
            )
        elif shape == "cone":
            self.moveit2.add_collision_cone(
                id=obj_id, position=pos, quat_xyzw=quat, height=dims[0], radius=dims[1]
            )
        else:
            raise ValueError(f"Unknown shape '{shape}'")
        self.objects[obj_id] = obj
        self.get_logger().info(f"Added {shape} '{obj_id}' at {pos}")


def main(args=None):
    rclpy.init(args=args)
    node = UR3MoveItScene()
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    thread = Thread(target=executor.spin, daemon=True)
    thread.start()
    timeout = 5.0
    start = time.time()
    got_service = False
    try:
        while time.time() - start < timeout:
            try:
                if node.moveit2._apply_planning_scene_service.wait_for_service(timeout_sec=0.5):
                    got_service = True
                    break
            except Exception:
                pass
        if not got_service:
            node.get_logger().warn(
                "MoveIt services not ready after timeout; objects may not appear until MoveIt is up."
            )
    except Exception as e:
        node.get_logger().warn(f"Error waiting for MoveIt services: {e}")

    # Check parameters and act on them
    if node.get_parameter("load").get_parameter_value().bool_value:
        node.load_scene()
    if node.get_parameter("clear").get_parameter_value().bool_value:
        node.clear_scene()
    remove_id = node.get_parameter("remove_id").get_parameter_value().string_value
    if remove_id:
        node.remove_object(remove_id)

    try:
        thread.join()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Scene Manager.")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Script to update the planning scene of the UR3e HandE robot using PyMoveIt2.
"""

import os
import yaml
from threading import Thread

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from std_srvs.srv import Trigger
from geometry_msgs.msg import Pose
from ur3e_hande_planning_interfaces.srv import AddObject, MoveObject, RemoveObject

from pymoveit2 import MoveIt2
from pymoveit2.robots import ur3e_hande as robot

import ament_index_python.packages as pkg

def get_config_path():
    package_share_path = pkg.get_package_share_directory('ur3e_hande_scene_manager')
    config_path = os.path.join(package_share_path, 'config', 'scene.yaml')
    return config_path

class SceneManager(Node):
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

        # Core services; ~/ indicates relative to node name
        self.create_service(Trigger, "~/load_scene", self.load_scene_cb)
        self.create_service(Trigger, "~/clear_scene", self.clear_scene_cb)

        # Dynamic object management
        ####################################################################
        # TODO (Ex 1b): instantiate add_object service
        ####################################################################


        ####################################################################
        # TODO (Ex 1d): instantiate move_object service
        ####################################################################

        ####################################################################

        # LEAVE UNCHANGED
        self.create_service(RemoveObject, "~/remove_object", self.remove_object_cb)
        self.get_logger().info("Dynamic Scene Manager Node ready.")

    
    # Scene loading and clearing
    def load_scene_cb(self, request, response):
        config_path = self.get_parameter("scene_config").value
        if not config_path or not os.path.exists(config_path):
            response.success = False
            response.message = f"Scene config file not found: {config_path}"
            self.get_logger().error(response.message)
            return response

        self.get_logger().info(f"Loading scene from {config_path}")
        with open(config_path, "r") as f:
            scene = yaml.safe_load(f)

        for obj in scene.get("scene_objects", []):
            self.add_collision_object(obj)

        response.success = True
        response.message = f"Loaded {len(scene.get('scene_objects', []))} objects."
        return response

    def clear_scene_cb(self, request, response):
        self.get_logger().info("Clearing planning scene...")
        fut = self.moveit2.clear_all_collision_objects()
        if fut:
            self.moveit2.process_clear_all_collision_objects_future(fut)
            self.objects.clear()
            response.success = True
            response.message = "Scene cleared."
        else:
            response.success = False
            response.message = "Failed to clear scene."
        return response

    
    # Dynamic object operations
    ####################################################################
       # TODO (Ex 1a): Create add_object_cb class method
    ####################################################################
       


    ####################################################################
    ####################################################################


    ####################################################################
    # TODO (Ex 1c): Modify move_object_cb class method
    ####################################################################
    def move_object_cb(self, request, response):
        if request.id not in self.objects:
            response.success = False
            response.message = f"Object '{request.id}' not found."
            return response
        try:
            # TODO: call class method move_collision_object
            # passing in the arguments from the request argument
            # id, position, quat_xyzw
            ####################################################
            response.success = True
            response.message = f"Moved '{request.id}'"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def remove_object_cb(self, request, response):
        try:
            self.remove_collision_object(request.id)
            response.success = True
            response.message = f"Removed '{request.id}'"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    
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

    def move_collision_object(self, obj_id, new_pos, new_quat):
        self.moveit2.move_collision(id=obj_id, position=new_pos, quat_xyzw=new_quat)
        self.objects[obj_id]["position"] = new_pos
        self.objects[obj_id]["quat_xyzw"] = new_quat

    def remove_collision_object(self, obj_id):
        if obj_id in self.objects:
            self.moveit2.remove_collision_object(id=obj_id)
            del self.objects[obj_id]
            self.get_logger().info(f"Removed '{obj_id}'")
        else:
            self.get_logger().warn(f"Tried to remove unknown object '{obj_id}'")


def main(args=None):
    rclpy.init(args=args)
    node = SceneManager()
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    thread = Thread(target=executor.spin, daemon=True)
    thread.start()
    node.get_logger().info("Dynamic Scene Manager running.")
    try:
        thread.join()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Scene Manager.")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()


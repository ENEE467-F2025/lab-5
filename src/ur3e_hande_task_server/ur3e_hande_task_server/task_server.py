#!/usr/bin/env python3
"""
ROS 2 node implementing a task server for the UR3e HandE robot using PyMoveIt2.

Named tasks:
- Home: Move the robot to the SRDF "home" configuration (or report already homed)
- Pick-and-Place: Move from a preset "pick" configuration to a preset "place" configuration,
  operating the gripper in between.
"""

from threading import Thread
from typing import List, Optional
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2, MoveIt2State, GripperInterface
from pymoveit2.robots import ur3e_hande as robot

class UR3eHandETaskServer(Node):
    def __init__(self):
        super().__init__("ur3e_hande_task_server")

        # Create callback group that allows execution of callbacks in parallel without restrictions
        self._cb_group = ReentrantCallbackGroup()

        # Parameters
        # Which task to run: "Home" or "Pick-and-Place"
        self.declare_parameter("task", "Home")
        # Plan/execution options
        self.declare_parameter("synchronous", True)
        self.declare_parameter("planner_id", "RRTConnectkConfigDefault")
        self.declare_parameter("gripper_cmd_action", "gripper_action_controller/gripper_cmd")

        # Named states (from SRDF) if available
        self._named_states = robot.get_named_group_states("")
        # Default names for the states used by the tasks
        self.declare_parameter("home_state", "home")
        self.declare_parameter("pick_state", "pick")
        self.declare_parameter("place_state", "place")

        # Fallback joint arrays if the named states are not available in SRDF
        # Note: Only used if the corresponding named state does not exist
        self.declare_parameter("home_joints", [0.0, -1.5707, 0.0, -1.5707, 0.0, 0.0])
        self.declare_parameter("pick_joints", [0.0, -1.57, 0.0, -1.57, 0.0, 0.0])
        self.declare_parameter("place_joints", [0.5, -1.2, 0.8, -1.3, 0.7, 0.2])

        # Create MoveIt2 interface for UR3e HandE
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=robot.joint_names(),              # UR3e joint names
            base_link_name=robot.base_link_name(),        # UR3e base link name
            end_effector_name=robot.end_effector_name(),  # UR3e end effector name
            group_name=robot.MOVE_GROUP_ARM,              # UR3e group name for the arm
            callback_group=self._cb_group,
        )
        # Planner and scaling defaults
        self.moveit2.planner_id = self.get_parameter("planner_id").get_parameter_value().string_value
        self.moveit2.max_velocity = 0.2
        self.moveit2.max_acceleration = 0.2

        # Gripper interface
        self._gripper = GripperInterface(
            node=self,
            gripper_joint_names=robot.gripper_joint_names(),
            open_gripper_joint_positions=robot.OPEN_GRIPPER_JOINT_POSITIONS,
            closed_gripper_joint_positions=robot.CLOSED_GRIPPER_JOINT_POSITIONS,
            gripper_group_name=robot.MOVE_GROUP_GRIPPER,
            callback_group=self._cb_group,
            gripper_command_action_name=self.get_parameter("gripper_cmd_action").get_parameter_value().string_value,
        )

        # Start shortly after construction to allow MoveIt to initialize
        self.create_timer(0.5, self._start, callback_group=self._cb_group)

    # ===== Helpers =====
    def _get_named_or_param(self, name_key: str, joints_param: str) -> List[float]:
        """Return joint positions for a named SRDF state if present; otherwise parameter fallback."""
        state_name = self.get_parameter(name_key).get_parameter_value().string_value
        if state_name in self._named_states:
            return self._named_states[state_name]
        # Fallback to param joints
        return list(self.get_parameter(joints_param).get_parameter_value().double_array_value)

    def _current_arm_positions(self) -> Optional[List[float]]:
        js = self.moveit2.joint_state
        if js is None:
            return None
        name_to_pos = {n: p for n, p in zip(js.name, js.position)}
        return [name_to_pos.get(jn, 0.0) for jn in self.moveit2.joint_names]

    def _move_to_joints(self, joints: List[float], synchronous: bool = True):
        self.get_logger().info(f"Planning to joints: {joints}")
        self.moveit2.move_to_configuration(joints)
        if synchronous:
            self.moveit2.wait_until_executed()

    # ===== Tasks =====
    def _task_home(self):
        synchronous = self.get_parameter("synchronous").get_parameter_value().bool_value
        home_joints = self._get_named_or_param("home_state", "home_joints")
        cur = self._current_arm_positions()
        if cur is not None and len(cur) == len(home_joints):
            if max(abs(c - h) for c, h in zip(cur, home_joints)) < 1e-2:
                self.get_logger().info("Already at home configuration.")
                return
        self.get_logger().info("Moving to home configuration...")
        self._move_to_joints(home_joints, synchronous=synchronous)
        self.get_logger().info("Home task completed.")

    def _task_pick_and_place(self):
        synchronous = True  # enforce sequencing
        pick_joints = self._get_named_or_param("pick_state", "pick_joints")
        place_joints = self._get_named_or_param("place_state", "place_joints")

        # Approach pick
        self.get_logger().info("Moving to pick configuration...")
        self._move_to_joints(pick_joints, synchronous=synchronous)
        # Close gripper
        self.get_logger().info("Closing gripper...")
        self._gripper.close(); self._gripper.wait_until_executed()
        # Move to place
        self.get_logger().info("Moving to place configuration...")
        self._move_to_joints(place_joints, synchronous=synchronous)
        # Open gripper
        self.get_logger().info("Opening gripper...")
        self._gripper.open(); self._gripper.wait_until_executed()
        self.get_logger().info("Pick-and-Place task completed.")

    # ===== Startup/User handler =====
    def _start(self):
        # One-shot timer body
        task = self.get_parameter("task").get_parameter_value().string_value.lower().strip()
        self.get_logger().info(f"Selected task: {task}")
        if task in ("home",):
            self._task_home()
        elif task in ("pick-and-place", "pick_and_place", "pickplace"):
            self._task_pick_and_place()
        else:
            self.get_logger().warn("Unknown task. Use 'Home' or 'Pick-and-Place'.")
        # prevent re-triggering
        self._start = lambda: None

def main(args=None):
    rclpy.init(args=args)

    ur3e_hande_task_server = UR3eHandETaskServer()

    # Spin in a separate thread to allow for concurrent task execution
    spin_thread = Thread(target=rclpy.spin, args=(ur3e_hande_task_server,), daemon=True)
    spin_thread.start()

    ur3e_hande_task_server.get_logger().info("UR3e HandE Task Server is running.")

    try:
        spin_thread.join()
    except KeyboardInterrupt:
        pass

    ur3e_hande_task_server.destroy_node()
    rclpy.shutdown()
#!/usr/bin/env python3

# Lab 5: Collision-Free Kinematic Motion Planning in ROS 2
# Copyright (C) 2025 Clinton Enwerem
#
# Licensed under the Apache License, Version 2.0 (the "License").

"""
Benchmark several OMPL planners on the same start-to-goal arm motion using
PyMoveIt2. This node only PLANS: it calls plan() and inspects the returned
trajectory. It never executes a motion, so it is safe to run in simulation and
on hardware.

Usage:
    ros2 run ur3e_hande_moveit_scripts ur3_planner_benchmark_node
    ros2 run ur3e_hande_moveit_scripts ur3_planner_benchmark_node \
        --ros-args -p add_obstacle:=true

Developed for the course ENEE467: Robotics Project Laboratory, University of Maryland, College Park, MD.
"""

import time
from math import sqrt

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2
from pymoveit2.robots import ur as robot

# (label, OMPL planner id) pairs to compare. These mirror the planners from Part 1.
PLANNERS = [
    ("RRTConnect", "RRTConnectkConfigDefault"),
    ("RRT*",       "RRTstarkConfigDefault"),
    ("PRM",        "PRMkConfigDefault"),
]

START = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
GOAL = [1.2, -1.0, 0.8, -1.5, -1.57, 0.0]
TRIALS = 5


def path_length(trajectory):
    """Return the C-space path length: the sum of joint-space distances between
    consecutive trajectory points."""
    pts = trajectory.points
    total = 0.0
    for i in range(1, len(pts)):
        p1 = pts[i - 1].positions
        p2 = pts[i].positions
        ############################################################
        # TODO (Exercise): add the distance between p1 and p2 to total.
        # p1 and p2 are lists of six joint angles. The distance is the
        # Euclidean norm of their difference.
        # Hint: sqrt(sum((a - b) ** 2 for a, b in zip(p2, p1)))
        ############################################################
        pass  # <--- MODIFY: replace with the line that updates total
    return total


class PlannerBenchmark(Node):
    def __init__(self):
        super().__init__("ur3_planner_benchmark")
        self.declare_parameter("add_obstacle", False)
        self._cb = ReentrantCallbackGroup()
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=robot.joint_names(),
            base_link_name=robot.base_link_name(),
            end_effector_name=robot.end_effector_name(),
            group_name=robot.MOVE_GROUP_ARM,
            callback_group=self._cb,
        )
        self.moveit2.allowed_planning_time = 5.0
        self._timer = self.create_timer(2.0, self._run, callback_group=self._cb)

    def _run(self):
        self._timer.cancel()

        if self.get_parameter("add_obstacle").value:
            # A box obstacle placed between the start and goal.
            self.moveit2.add_collision_box(
                id="obstacle",
                size=[0.1, 0.4, 0.4],
                position=[0.4, 0.2, 0.4],
                quat_xyzw=[0.0, 0.0, 0.0, 1.0],
            )
            time.sleep(1.0)
            self.get_logger().info("Added obstacle to the planning scene.")

        self.get_logger().info(
            f"{'planner':<12}{'success':<10}{'plan_time[s]':<14}{'path_len[rad]'}")
        for label, planner_id in PLANNERS:
            self.moveit2.planner_id = planner_id
            n_ok, t_sum, len_sum = 0, 0.0, 0.0
            for _ in range(TRIALS):
                t0 = time.time()
                traj = self.moveit2.plan(joint_positions=GOAL, start_joint_state=START)
                dt = time.time() - t0
                if traj is not None and len(traj.points) > 0:
                    n_ok += 1
                    t_sum += dt
                    len_sum += path_length(traj)
            if n_ok > 0:
                self.get_logger().info(
                    f"{label:<12}{f'{n_ok}/{TRIALS}':<10}"
                    f"{t_sum / n_ok:<14.4f}{len_sum / n_ok:.3f}")
            else:
                self.get_logger().info(f"{label:<12}{'0/' + str(TRIALS):<10}(no solution)")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = PlannerBenchmark()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

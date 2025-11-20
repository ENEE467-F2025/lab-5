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
# 

"""
Simple script to listen and print the static transform between the base_link and camera_depth_optical_frame.

Author: Clinton Enwerem
Developed for the course ENEE467: Robotics Projects Laboratory, Fall 2025, University of Maryland, College Park, MD.
"""

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped

rclpy.init()
node = Node("tf_listener")
tf_buffer = Buffer()
tf_listener = TransformListener(tf_buffer, node)

try:
    t = tf_buffer.lookup_transform(
        "base_link",                  # target frame
        "camera_depth_optical_frame", # source frame
        rclpy.time.Time(),            # at latest available time
        timeout=rclpy.duration.Duration(seconds=1.0)
    )
    print(t.transform.translation)
    print(t.transform.rotation)
except Exception as e:
    print("TF lookup failed:", e)

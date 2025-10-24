#!/usr/bin/env python3

"""
Simple obstacle publisher for RViz and collision checking

Author: Clinton Enwerem
Developed for the course ENEE467: Robotics Projects Laboratory, Fall 2025, University of Maryland, College Park, MD.
"""
import rclpy
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped, Pose
from robot_3r_msgs.msg import SceneObstacles
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA
from pymoveit2.robots import robot_3r as robot
import math
import itertools

# robot data
BASE_LINK_NAME = robot.base_link_name("")
WORLD_FRAME = robot.world_frame("")

class SimpleObstaclePublisher(Node):
    def __init__(self):
        super().__init__('simple_obstacle_publisher')
        self.declare_parameter("obstacle_type", "box")
        self.declare_parameter("is_dense", False)
        self.first_publish = True
        self.first_marker_publish = True

        self.is_dense = self.get_parameter("is_dense").get_parameter_value().bool_value
        if self.is_dense:
            # cage the robot in a ring of obstacles
            num_obstacles = 10
            radius = 0.45
            center_z = 0.5
            platform_height = 0.755
            obs_z = [1.0, 0.8]
            obstacle_positions = []
            obstacle_sizes = []
            for i in range(num_obstacles):
                angle = 2 * math.pi * i / num_obstacles
                x = radius * math.cos(angle)
                y = radius * math.sin(angle)
                
                if i % 2 == 0:
                    obstacle_sizes.append([0.1, 0.1, obs_z[0]])
                    obstacle_positions.extend([x, y, center_z * obs_z[i % 2]])    
                else:
                    obstacle_sizes.append([0.15, 0.15, obs_z[1]])
                    obstacle_positions.extend([x, y, platform_height + center_z * obs_z[i % 2]])
            self.declare_parameter("num_obstacles", num_obstacles)
            self.declare_parameter("obstacle_positions", obstacle_positions)
            self.declare_parameter("obstacle_sizes", list(itertools.chain.from_iterable(obstacle_sizes)))
        else:
            self.declare_parameter("num_obstacles", 2)
            self.declare_parameter("obstacle_sizes", [0.1, 0.1, 1.0])  # for box: [x, y, z]
            self.declare_parameter("obstacle_positions", [0.0, 0.6, 0.5, 0.6, 0.0, 0.5]) # 0.755+(0.5*z) # no 2D array support

        self.num_obstacles = self.get_parameter("num_obstacles").get_parameter_value().integer_value
        self.obstacle_type = self.get_parameter("obstacle_type").get_parameter_value().string_value
        self.obstacle_sizes = self.get_parameter("obstacle_sizes").get_parameter_value().double_array_value
        self.obstacle_positions = self.get_parameter("obstacle_positions").get_parameter_value().double_array_value
        self.scene_obstacle_publisher_ = self.create_publisher(SceneObstacles, 'scene_obstacles', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

        # obstacle marker publisher
        self.obstacle_marker_pub = self.create_publisher(MarkerArray, "obstacle_markers", 10)
        self.obstacle_marker_pub_timer = self.create_timer(0.1, self.publish_obstacle_markers)
        self.obstacle_dict = {}

    def timer_callback(self):
        self.publish_obstacles()

    def publish_obstacles(self):
        msg = SceneObstacles()
        poses = [] 
        obstacles = [] 
        obstacle_ids = []
        for i in range(self.num_obstacles):
            pose = PoseStamped()
            pose.header.frame_id = WORLD_FRAME
            pose.pose.position.x = self.obstacle_positions[3*i]
            pose.pose.position.y = self.obstacle_positions[3*i+1]
            pose.pose.position.z = self.obstacle_positions[3*i+2]
            pose.pose.orientation.w = 1.0
            poses.append(pose)
            obstacle_ids.append(i+1)  
            if self.obstacle_type == "box":
                obstacle = SolidPrimitive()
                obstacle.type = SolidPrimitive.BOX
                obstacle.dimensions = self.obstacle_sizes[3*i:3*i+3] if self.is_dense else self.obstacle_sizes
                obstacles.append(obstacle)
            self.obstacle_dict[i+1] = (obstacle, pose)

        msg.scene_obstacles = obstacles
        msg.obstacle_poses = poses
        msg.obstacle_ids = obstacle_ids
        self.scene_obstacle_publisher_.publish(msg)

        # show detailed info only on first publish
        if self.first_publish:
            self.get_logger().info(f"Published {len(obstacles)} obstacles.")
            self.first_publish = False

    def publish_obstacle_markers(self):
        if self.obstacle_dict == {} or self.obstacle_dict is None:
            self.get_logger().warning("No obstacles to publish")
            return
        else:
            obstacles = []
            obstacle_poses = []
            for obstacle, pose in self.obstacle_dict.values():
                obstacles.append(obstacle)
                obstacle_poses.append(pose)

            marker_array = MarkerArray()
            for i, (obstacle, pose) in enumerate(zip(obstacles, obstacle_poses)):
                marker = Marker()
                marker_pose = Pose()
                marker_pose.position = pose.pose.position
                marker_pose.orientation = pose.pose.orientation
                marker.header.frame_id = WORLD_FRAME
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "obstacles"
                marker.id = i+1
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.pose = marker_pose
                marker.scale = Vector3()
                if self.is_dense:
                    marker.scale.x = self.obstacle_sizes[3*i]
                    marker.scale.y = self.obstacle_sizes[3*i+1]
                    marker.scale.z = self.obstacle_sizes[3*i+2]
                else:
                    marker.scale.x = self.obstacle_sizes[0]
                    marker.scale.y = self.obstacle_sizes[1]
                    marker.scale.z = self.obstacle_sizes[2]
                marker.color = ColorRGBA()
                marker.color.r = 1.0
                marker.color.g = 0.64
                marker.color.b = 0.0
                marker.color.a = 1.0
                marker_array.markers.append(marker)
            self.obstacle_marker_pub.publish(marker_array)
            if self.first_marker_publish:
                self.get_logger().info(f"Published {len(marker_array.markers)} obstacle markers")
                self.first_marker_publish = False

def main(args=None):
    rclpy.init(args=args)
    obstacle_pub_node = SimpleObstaclePublisher()
    rclpy.spin(obstacle_pub_node)
    obstacle_pub_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
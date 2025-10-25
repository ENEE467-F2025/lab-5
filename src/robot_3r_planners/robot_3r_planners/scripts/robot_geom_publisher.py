#!/usr/bin/env python3

"""
Robot geometry publisher using URDF and xml.etree.ElementTree

Author: Clinton Enwerem
Developed for the course ENEE467: Robotics Projects Laboratory, Fall 2025, University of Maryland, College Park, MD.
"""

import rclpy
from rclpy.node import Node
import xml.etree.ElementTree as ET
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, PoseStamped
import transforms3d as t3d
from robot_3r_interfaces.msg import RigidBodyGeom
from rclpy.duration import Duration
from pymoveit2.robots import robot_3r as robot
from robot_3r_planners.utils.collision_utils import get_link_transform
import spatialmath as sm
import numpy as np

# robot data
JOINT_NAMES = robot.joint_names("")
JOINT_LIMITS = robot.get_joint_limits()
GROUP_STATES = robot.get_named_group_states("")
DISABLED_COLLISION_PAIRS = robot.get_disabled_collision_pairs("")
BASE_LINK_NAME = robot.base_link_name("")
LINK_NAMES = robot.get_link_names("")
WORLD_FRAME = robot.world_frame("")

class URDFCollisionPublisher(Node):
    def __init__(self):
        super().__init__("robot_geom_publisher")
        self.cache_time = Duration(seconds=1)
        # load from parameter
        if not self.has_parameter("robot_description"):
            self.declare_parameter("robot_description", "")
        urdf_str = self.get_parameter("robot_description").get_parameter_value().string_value
        if not urdf_str:
            raise RuntimeError("robot_description parameter is empty!")

        self.tree = ET.fromstring(urdf_str) # root element
        if self.tree.attrib.get("name") is not None:
            self.robot_name =  self.tree.attrib.get("name").lower().strip().replace(" ", "_")
        # self.get_logger().info("URDF parsed successfully.")
        self.robot_geom = None
        self.pub = self.create_publisher(RigidBodyGeom, "robot_geometry", 10)
        self.timer = self.create_timer(6.0, self.publish_robot_geometry_once)
        self.first_publish = True  # Flag to show detailed info only once

    def publish_robot_geometry_once(self):
        self.publish_robot_geometry()
        self.create_timer(1.0, self.publish_robot_geometry)

    def publish_robot_geometry(self):
        robot_geom = RigidBodyGeom()
        link_poses = []
        link_geometries = []
        link_names = []
        link_geom_origins = []

        # build a joint map to apply joint transformations
        joint_map = {}
        for joint in self.tree.findall("joint"):
            parent = joint.find("parent").attrib.get("link")
            child = joint.find("child").attrib.get("link")
            joint_origin = joint.find("origin")
            if joint_origin is not None:
                xyz_origin = [float(x) for x in joint_origin.attrib.get("xyz", "0 0 0").split()]
                rpy_origin = [float(x) for x in joint_origin.attrib.get("rpy", "0 0 0").split()]
            else:
                xyz_origin = [0, 0, 0]
                rpy_origin = [0, 0, 0]
            T_joint = sm.SE3(xyz_origin) * sm.SE3.RPY(rpy_origin, order="xyz", unit="rad")
            joint_map[child] = (parent, T_joint)

        for link in self.tree.findall("link"):
            link_name = link.attrib.get("name")

            for col in link.findall("collision"):
                geom = col.find("geometry")
                origin = col.find("origin")
                if origin is not None:
                    xyz = [float(x) for x in origin.attrib.get("xyz", "0 0 0").split()]
                    rpy = [float(x) for x in origin.attrib.get("rpy", "0 0 0").split()]
                else:
                    xyz = [0, 0, 0]
                    rpy = [0, 0, 0]

                quat_offset = t3d.euler.euler2quat(rpy[0], rpy[1], rpy[2])  # [w,x,y,z]
                
                # save original geometry pose relative to link frame
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = xyz
                pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z = quat_offset
                link_geom_origins.append(pose)

                try:
                    T_link = get_link_transform(link_name, joint_map).A
                except Exception as e:
                    self.get_logger().error(f"Error getting transform for link '{link_name}': {e}")
                    continue

                try:
                    T_final = T_link @ t3d.affines.compose(
                        T=xyz, R=t3d.quaternions.quat2mat(quat_offset), Z=[1, 1, 1]
                    )
                except Exception as e:
                    self.get_logger().error(f"Error composing transform for link '{link_name}': {e}")
                    continue

                # extract pose
                trans_final, rot_final, _, _ = t3d.affines.decompose44(T_final)
                quat_final = t3d.quaternions.mat2quat(rot_final)

                # create solid primitive for collision geometry
                prim = SolidPrimitive()
                if geom.find("box") is not None:
                    size = [float(x) for x in geom.find("box").attrib.get("size").split()]
                    prim.type = SolidPrimitive.BOX
                    prim.dimensions = size
                elif geom.find("sphere") is not None:
                    r = float(geom.find("sphere").attrib.get("radius"))
                    prim.type = SolidPrimitive.SPHERE
                    prim.dimensions = [r]
                elif geom.find("cylinder") is not None:
                    r = float(geom.find("cylinder").attrib.get("radius"))
                    l = float(geom.find("cylinder").attrib.get("length"))
                    prim.type = SolidPrimitive.CYLINDER
                    prim.dimensions = [r, l]
                else:
                    continue

                link_geometries.append(prim)

                link_pose = PoseStamped()
                link_pose.header.frame_id = BASE_LINK_NAME
                link_pose.header.stamp = self.get_clock().now().to_msg()
                link_pose.pose.position.x = trans_final[0]
                link_pose.pose.position.y = trans_final[1]
                link_pose.pose.position.z = trans_final[2]
                link_pose.pose.orientation.w = quat_final[0]
                link_pose.pose.orientation.x = quat_final[1]
                link_pose.pose.orientation.y = quat_final[2]
                link_pose.pose.orientation.z = quat_final[3]
                link_poses.append(link_pose)
                link_names.append(link_name)

        robot_geom.link_poses = link_poses
        robot_geom.link_geometries = link_geometries
        robot_geom.link_names = link_names
        robot_geom.link_geom_origins = link_geom_origins
        
        # show detailed info only on first publish
        if self.first_publish:
            self.get_logger().info(
                f"Publishing robot geometry for {self.robot_name} with {len(link_names)} links: {link_names}"
            )
            self.first_publish = False            
        self.pub.publish(robot_geom)

def main():
    rclpy.init()
    urdf_coll_pub_node = URDFCollisionPublisher()
    rclpy.spin(urdf_coll_pub_node)
    urdf_coll_pub_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

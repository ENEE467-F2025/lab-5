from typing import List, Tuple, Optional

import numpy as np
import transforms3d as t3d
import spatialmath as sm
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, Pose, Point
from rclpy.duration import Duration
from rclpy.node import Node
from robot_3r_interfaces.msg import RigidBodyGeom
from shape_msgs.msg import SolidPrimitive
from pymoveit2.robots import robot_3r as robot


def build_ee_path_line_strip(
    computed_path: List[np.ndarray],
    rtb_model,
    base_link_name: str,
    node: Node,
) -> Tuple[Optional[MarkerArray], Optional[PoseStamped]]:
    """
    Build a LINE_STRIP marker and last end-effector PoseStamped for the given path.
    """
    if not computed_path:
        return None, None

    points = []
    last_pose_stamped: Optional[PoseStamped] = None

    for q in computed_path:
        try:
            rtb_model.q = q
            T = rtb_model.fkine(q).A
        except Exception:
            continue
        trans = T[:3, 3].tolist()
        R = T[:3, :3]
        quat = t3d.quaternions.mat2quat(R)  # (w,x,y,z)
        ps = PoseStamped()
        ps.header.frame_id = base_link_name
        ps.header.stamp = node.get_clock().now().to_msg()
        ps.pose.position.x = trans[0]
        ps.pose.position.y = trans[1]
        ps.pose.position.z = trans[2]
        ps.pose.orientation.w = float(quat[0])
        ps.pose.orientation.x = float(quat[1])
        ps.pose.orientation.y = float(quat[2])
        ps.pose.orientation.z = float(quat[3])
        last_pose_stamped = ps
        points.append(Point(x=trans[0], y=trans[1], z=trans[2]))

    if not points:
        return None, last_pose_stamped

    marker = Marker()
    marker.header.frame_id = base_link_name
    marker.header.stamp = node.get_clock().now().to_msg()
    marker.ns = "ee_path"
    marker.id = 0
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = 0.015
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.points = points
    marker.lifetime = Duration(seconds=0).to_msg()

    return MarkerArray(markers=[marker]), last_pose_stamped


def build_jsp_waypoint_markers(
    computed_path: List[np.ndarray],
    robot_geom: RigidBodyGeom,
    rtb_model,
    base_link_name: str,
    joint_limits: List[tuple],
    node: Node,
) -> Optional[MarkerArray]:
    """
    Build a MarkerArray containing simplified link primitives for each waypoint.
    """
    if robot_geom is None or not computed_path:
        return None

    markers = MarkerArray()
    n_waypoints = len(computed_path)

    for idx, q in enumerate(computed_path):
        marker_id = 0
        try:
            clamped_q = [
                max(joint_limits[i][0], min(q[i], joint_limits[i][1]))
                for i in range(len(q))
            ]
            rtb_link_poses = {}
            for link in rtb_model.links:
                link_pose = rtb_model.fkine(q, end=link.name, include_base=True)
                rtb_link_poses[link.name] = robot.se3_to_pose_stamped(
                    link_pose, node_obj=node, frame_id=base_link_name
                )
        except Exception as e:
            node.get_logger().warning(f"FK failed for waypoint {idx}: {e}")
            continue

        colors = {
            "base_link": (0.2, 0.2, 0.2),
            "link0": (0.0, 0.0, 0.0),
            "link1": (0.122, 0.467, 0.706),
            "link2": (0.122, 0.467, 0.706),
            "link3": (0.122, 0.467, 0.706),
        }

        scale_offset = 0.0
        for i, link_name in enumerate(robot_geom.link_names):
            try:
                link_geom = robot_geom.link_geometries[i]
                link_geom_orig = robot_geom.link_geom_origins[i]
            except IndexError:
                continue

            p = Pose()
            link_pose = rtb_link_poses[link_name].pose
            link_T = sm.SE3(
                link_pose.position.x,
                link_pose.position.y,
                link_pose.position.z,
            ) * sm.SE3.RPY(
                *t3d.euler.quat2euler(
                    [
                        link_pose.orientation.w,
                        link_pose.orientation.x,
                        link_pose.orientation.y,
                        link_pose.orientation.z,
                    ],
                    axes="sxyz",
                )
            )
            origin_T = sm.SE3(
                link_geom_orig.position.x,
                link_geom_orig.position.y,
                link_geom_orig.position.z,
            ) * sm.SE3.RPY(
                *t3d.euler.quat2euler(
                    [
                        link_geom_orig.orientation.w,
                        link_geom_orig.orientation.x,
                        link_geom_orig.orientation.y,
                        link_geom_orig.orientation.z,
                    ],
                    axes="sxyz",
                )
            )
            geom_T = link_T * origin_T
            ps = robot.se3_to_pose_stamped(geom_T, node_obj=node, frame_id=base_link_name)
            p.position = ps.pose.position
            p.orientation = ps.pose.orientation

            m = Marker()
            m.header.frame_id = base_link_name
            m.header.stamp = node.get_clock().now().to_msg()
            m.ns = f"path_wp_{idx}"
            m.id = marker_id
            marker_id += 1

            if link_geom.type == SolidPrimitive.BOX:
                m.type = Marker.CUBE
                sx, sy, sz = link_geom.dimensions
                m.scale.x = sx + scale_offset
                m.scale.y = sy + scale_offset
                m.scale.z = sz + scale_offset
            elif link_geom.type == SolidPrimitive.SPHERE:
                m.type = Marker.SPHERE
                rads = link_geom.dimensions[0]
                m.scale.x = m.scale.y = m.scale.z = 2 * rads
            elif link_geom.type == SolidPrimitive.CYLINDER:
                m.type = Marker.CYLINDER
                radius = link_geom.dimensions[0]
                length = link_geom.dimensions[1]
                m.scale.x = m.scale.y = 2 * radius
                m.scale.z = length
            else:
                m.type = Marker.SPHERE
                m.scale.x = m.scale.y = m.scale.z = 0.02
            m.pose = p

            if idx == 0 and link_name in ["link1", "link2", "link3"]:
                m.color.r = 0.61624
                m.color.g = 0.53967
                m.color.b = 0.99264
                m.color.a = 1.0
            elif idx == n_waypoints - 1 and link_name in ["link1", "link2", "link3"]:
                m.color.r = 0.122
                m.color.g = 0.467
                m.color.b = 0.706
                m.color.a = 1.0
            elif link_name in ["link2", "link3"]:
                m.color.r = 0.0
                m.color.g = 0.0
                m.color.b = 0.0
                m.color.a = 0.16
            else:
                m.color.r = colors[link_name][0]
                m.color.g = colors[link_name][1]
                m.color.b = colors[link_name][2]
                m.color.a = 1.0

            m.lifetime = Duration(seconds=0).to_msg()
            markers.markers.append(m)

    return markers

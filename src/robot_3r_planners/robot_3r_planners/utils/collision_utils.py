#!/usr/bin/env python3

from typing import List
from rclpy.node import Node

from robot_3r_interfaces.msg import RigidBodyGeom, SceneObstacles
from shape_msgs.msg import SolidPrimitive
import fcl
from geometry_msgs.msg import PoseStamped
import transforms3d as t3d
import spatialmath as sm
import numpy as np
from pymoveit2.robots import robot_3r as robot

def robotgeom_to_fclobj(robot_geom: RigidBodyGeom):
    # Convert robot geometry to FCL collision objects
    fcl_objects = []
    for i in range(len(robot_geom.link_names)):
        link_name = robot_geom.link_names[i]
        link_pose = robot_geom.link_poses[i]
        link_geometry = robot_geom.link_geometries[i]
        link_geometry_type = link_geometry.type
        if link_geometry_type == SolidPrimitive.BOX:
            fcl_geom = fcl.Box(*link_geometry.dimensions)
        elif link_geometry_type == SolidPrimitive.SPHERE:
            fcl_geom = fcl.Sphere(link_geometry.radius)
        elif link_geometry_type == SolidPrimitive.CYLINDER:
            fcl_geom = fcl.Cylinder(link_geometry.radius, link_geometry.height)
        else:
            print(f"Unsupported geometry type: {link_geometry_type}")
            continue
        fcl_obj = create_fcl_object(link_pose, fcl_geom)
        fcl_objects.append(fcl_obj)
    return fcl_objects

def obstacle_to_fclobj(obstacles: SceneObstacles):
    # Convert obstacle geometry to FCL collision objects
    fcl_objects = []
    for i in range(len(obstacles.obstacle_ids)):
        obstacle_pose = obstacles.obstacle_poses[i]
        obstacle_geometry = obstacles.scene_obstacles[i]
        obstacle_geometry_type = obstacle_geometry.type
        if obstacle_geometry_type == SolidPrimitive.BOX:
            fcl_geom = fcl.Box(*obstacle_geometry.dimensions)
            obs_name_str = "box"
        elif obstacle_geometry_type == SolidPrimitive.SPHERE:
            fcl_geom = fcl.Sphere(obstacle_geometry.radius)
            obs_name_str = "sphere"
        elif obstacle_geometry_type == SolidPrimitive.CYLINDER:
            fcl_geom = fcl.Cylinder(obstacle_geometry.radius, obstacle_geometry.height)
            obs_name_str = "cylinder"
        else:
            print(f"Unsupported geometry type: {obstacle_geometry_type}")
            continue
        # obstacle_name = f"{obs_name_str.title()}_{obstacles.obstacle_ids[i]}"
        fcl_obj = create_fcl_object(obstacle_pose, fcl_geom)
        fcl_objects.append(fcl_obj)
    return fcl_objects

def create_fcl_object(pose: PoseStamped, geometry: fcl.CollisionGeometry) -> fcl.CollisionObject:
    q = pose.pose.orientation
    t = pose.pose.position
    R = t3d.quaternions.quat2mat([q.w, q.x, q.y, q.z])
    x, y, z = t.x, t.y, t.z
    translation = [x, y, z]
    fcl_obj = fcl.CollisionObject(geometry, fcl.Transform(R, translation))
    return fcl_obj

def check_collision(fcl_obj1, fcl_obj2):
    request = fcl.CollisionRequest()
    result = fcl.CollisionResult()
    dist_request = fcl.DistanceRequest()
    dist_result = fcl.DistanceResult()

    ret_dist = fcl.distance(fcl_obj1, fcl_obj2, dist_request, dist_result)
    ret = fcl.collide(fcl_obj1, fcl_obj2, request, result)     # check for collision between two FCL objects
    # print(f"FCL Collision: {ret_dist < 0}")
    # print(f"Min dist between objects: {ret_dist}")
    return ret_dist < 0 # if True, objects are in collision

def get_link_transform(link_name, joint_map) -> sm.SE3:
    """Return transform from base to this link by chaining all parent joints."""
    T = sm.SE3()
    current = link_name
    chain = []
    while current in joint_map:
        parent, T_joint = joint_map[current]
        chain.append((parent, current))
        T = T_joint * T
        current = parent
    # print(
    #     f"\033[92mTransform chain for '{link_name}': {chain} --> {T}\033[0m"
    # )
    return T


def is_path_segment_collision_free(
    path_q: List[np.ndarray],
    robot_geom: RigidBodyGeom,
    obstacles: SceneObstacles,
    node: Node,
    rtb_model,
    base_link_name: str,
    world_frame_name: str,
) -> bool:
    """
    Check all intermediate joint configs (path_q) for collisions against the obstacle set.

    Args:
        path_q: list of numpy arrays representing joint configs along a local path segment
        robot_geom: robot link geometries and origins
        obstacles: scene obstacles message
        node: ROS node for params/logging
        rtb_model: Robotics Toolbox model with .fkine and .links
        base_link_name: base link name used for frames
        world_frame_name: world frame name to skip pseudo-links

    Returns:
        True if no collisions along the entire segment, else False.
    """
    if not node.get_parameter("check_collision").get_parameter_value().bool_value:
        node.get_logger().warning("check_collision set to False. Path will be invalid.")
        return False

    try:
        obs_fcl_objects = list(obstacle_to_fclobj(obstacles=obstacles))
    except Exception as e:
        node.get_logger().error(f"Failed to build FCL obstacles: {e}")
        return False

    checker = node.get_parameter("collision_checker").get_parameter_value().string_value
    verbose = node.get_parameter("verbose").get_parameter_value().bool_value
    min_obs_dist = node.get_parameter("min_obs_dist").get_parameter_value().double_value

    for q in path_q:
        # set robot joints on the shared model
        try:
            rtb_model.q = q
        except Exception as e:
            node.get_logger().error(f"Failed to set model configuration: {e}")
            return False

        for link in rtb_model.links:
            link_name = link.name
            # skip base/world pseudo-links
            if link_name in [base_link_name, "link0", world_frame_name]:
                continue

            # Find mapping for geometry
            try:
                idx = robot_geom.link_names.index(link_name)
            except ValueError:
                # no geometry -> skip
                continue

            # Compute FK in world frame
            try:
                T_fk_se3 = rtb_model.fkine(q, end=link.name, include_base=True)
            except Exception as e:
                node.get_logger().error(f"FK failed for {link_name}: {e}")
                return False

            # Build geometry for this link
            link_geometry = robot_geom.link_geometries[idx]
            try:
                if link_geometry.type == SolidPrimitive.BOX:
                    x, y, z = link_geometry.dimensions
                    geom = fcl.Box(x, y, z)
                elif link_geometry.type == SolidPrimitive.SPHERE:
                    geom = fcl.Sphere(link_geometry.dimensions[0])
                elif link_geometry.type == SolidPrimitive.CYLINDER:
                    radius = link_geometry.dimensions[0]
                    height = link_geometry.dimensions[1]
                    geom = fcl.Cylinder(radius, height)
                else:
                    # unsupported -> skip this link
                    continue
            except Exception as e:
                node.get_logger().error(f"Invalid geometry for link {link_name}: {e}")
                return False

            rob_obj = create_fcl_object(robot.se3_to_pose_stamped(T_fk_se3, node), geom)

            # Check collisions/proximity against all obstacles
            for obs_obj in obs_fcl_objects:
                if checker == "bvol":
                    creq = fcl.CollisionRequest()
                    creq.enable_contact = True
                    cres = fcl.CollisionResult()
                    ret = fcl.collide(rob_obj, obs_obj, creq, cres)
                    if cres.is_collision or ret > 0:
                        if verbose:
                            node.get_logger().info(f"Bvol collision on {link_name} at q={np.round(q,3)}")
                        return False

                elif checker == "proximity":
                    dreq = fcl.DistanceRequest(enable_signed_distance=True)
                    dres = fcl.DistanceResult()
                    _ = fcl.distance(rob_obj, obs_obj, dreq, dres)
                    min_dist = dres.min_distance

                    if min_dist < 0:
                        if verbose:
                            node.get_logger().info(
                                f"Proximity collision on {link_name} at q={np.round(q,3)}"
                            )
                        node.set_parameters_by_dict({"proximity_alert": True})
                        return False
                    if min_dist < min_obs_dist:
                        node.set_parameters_by_dict({"proximity_alert": True})
                        # Close to either start or goal? treat as collision to prune
                        try:
                            target_q = np.asarray(node.get_parameter("goal_config").get_parameter_value().double_array_value)
                        except Exception:
                            target_q = None
                        try:
                            start_q = np.asarray(getattr(node, "start_config", None))
                        except Exception:
                            start_q = None
                        if start_q is not None and np.linalg.norm(q - start_q) < min_obs_dist:
                            return False
                        if target_q is not None and np.linalg.norm(q - target_q) < min_obs_dist:
                            return False
                        # Else soft proximity -> keep exploring but flag; treat as collision to simplify
                        if verbose:
                            node.get_logger().info(
                                f"Proximity near-miss on {link_name} at q={np.round(q,3)}"
                            )
                        return False
                else:
                    # Unknown checker type -> assume conservative
                    return False
    # No collisions found
    return True
#!/usr/bin/env python3

from robot_3r_interfaces.msg import RigidBodyGeom, SceneObstacles
from shape_msgs.msg import SolidPrimitive
import fcl
from geometry_msgs.msg import PoseStamped
import transforms3d as t3d
import spatialmath as sm
import numpy as np

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
    return T
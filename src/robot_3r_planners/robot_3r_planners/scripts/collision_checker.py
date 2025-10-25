#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_3r_interfaces.srv import CheckCollision
from robot_3r_interfaces.msg import RigidBodyGeom, SceneObstacles
from robot_3r_planners.utils.collision_utils import collision_free, robotgeom_to_fclobj, obstacle_to_fclobj

class CollisionChecker(Node):
    def __init__(self):
        super().__init__('collision_checker')
        self.robot_geom = None
        self.obstacle_geom = None

        # Subscriptions
        self.create_subscription(RigidBodyGeom, "robot_geometry", self.robot_geom_cb, 10)
        self.create_subscription(SceneObstacles, "scene_obstacles", self.scene_obs_cb, 10)

        # Service
        self.srv = self.create_service(CheckCollision, 'check_collision', self.check_collision_cb)

    def robot_geom_cb(self, msg):
        self.robot_geom = msg

    def scene_obs_cb(self, msg):
        self.obstacle_geom = msg

    def check_collision_cb(self, request, response):
        if self.robot_geom is None or self.obstacle_geom is None:
            self.get_logger().warn("No robot or obstacle geometry received yet")
            response.in_collision = True
            return response

        # Wrap your collision check utility
        response.in_collision = not collision_free(
            candidate_node=request.joint_positions,  
            robot_geom=self.robot_geom,
            obstacles=self.obstacle_geom,
            check_collision=True
        )
        return response

def main(args=None):
    rclpy.init(args=args)
    node = CollisionChecker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Point

from pymoveit2 import MoveIt2
from pymoveit2.robots import panda

class XArmMover(Node):

    def __init__(self):
        super().__init__('xarm_movement_node')
        
        # Create callback group that allows execution of callbacks in parallel without restrictions
        self.callback_group = ReentrantCallbackGroup()

        # Create MoveIt2 interface
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=panda.joint_names(),
            base_link_name=panda.base_link_name(),
            end_effector_name=panda.end_effector_name(),
            group_name=panda.MOVE_GROUP_ARM,
            callback_group=self.callback_group,
        )

        # Subscribe to the transformed_coordinates topic
        self.subscription = self.create_subscription(
            Point,
            '/transformed_coordinates',
            self.coordinates_callback,
            10
        )

        # Ensure the robot moves only once
        self.has_moved = False

    def coordinates_callback(self, msg):
        # If the robot has already moved, return
        if self.has_moved:
            return

        # Use the coordinates from the message to move the robot
        position = [msg.x, msg.y, msg.z + 0.172]
        quat_xyzw = [0.0, 1.0, 0.0, 0.0]  # Assuming a default orientation

        self.get_logger().info(f"Moving to {{position: {list(position)}, quat_xyzw: {list(quat_xyzw)}}}")
        self.moveit2.move_to_pose(position=position, quat_xyzw=quat_xyzw, cartesian=False)
        self.moveit2.wait_until_executed()

        # Set flag to indicate the robot has moved
        self.has_moved = True

def main(args=None):
    rclpy.init(args=args)
    node = XArmMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

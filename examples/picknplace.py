#!/usr/bin/env python3

import rclpy
import subprocess
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Point
from gb_visual_detection_3d_msgs.msg import LabeledCentroid

from pymoveit2 import MoveIt2, MoveIt2Gripper
from pymoveit2.robots import panda
import argparse
import time
import datetime

class TimeLog(object):
    def __init__(self, logger_method, subject):
        self.logger_method = logger_method
        self.subject = subject
    def __enter__(self):
        self.starttime = datetime.datetime.now()
    def __exit__(self, type, value, traceback):
        delta = (datetime.datetime.now() - self.starttime) / datetime.timedelta(seconds=1)
        msg = f"Timing result {self.subject}: {delta}"
        self.logger_method(msg)

def close_gripper():
    try:
        subprocess.run(["ros2", "run", "pymoveit2", "ex_gripper.py", "--ros-args", "-p", "action:=close"], check=True)
    except subprocess.CalledProcessError as e:
        print(f"An error occurred: {str(e)}")

def open_gripper():
    try:
        subprocess.run(["ros2", "run", "pymoveit2", "ex_gripper.py", "--ros-args", "-p", "action:=open"], check=True)
    except subprocess.CalledProcessError as e:
        print(f"An error occurred: {str(e)}")


class XArmMover(Node):

    def __init__(self, object_to_pick, drop_location):
        super().__init__('xarm_movement_node')
        self.object_to_pick = object_to_pick
        self.drop_location = drop_location
        
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

        #move to an initial start position and give some time to detect objects
        #small delay to allow for RViz to launch before running moveit2 command
        time.sleep(3)
        with TimeLog(self.get_logger().info, "Initial pose"):
            self.moveit2.move_to_pose(position=[-0.381, -0.257, 0.493], quat_xyzw=[0.0, 1.0, 0.0, 0.0], cartesian=False)
        time.sleep(3)

        # Create MoveIt2Gripper interface
        self.moveit2_gripper = MoveIt2Gripper(
            node=self,
            gripper_joint_names=panda.gripper_joint_names(),
            open_gripper_joint_positions=panda.OPEN_GRIPPER_JOINT_POSITIONS,
            closed_gripper_joint_positions=panda.CLOSED_GRIPPER_JOINT_POSITIONS,
            gripper_group_name=panda.MOVE_GROUP_GRIPPER,
            callback_group=self.callback_group,
        )

        self.detected_objects = set()
        self.object_coordinates = {}
        self.is_waiting_for_user = False

        # Subscribe to the transformed_coordinates topic
        self.subscription = self.create_subscription(
            LabeledCentroid,
            '/transformed_coordinates',
            self.coordinates_callback,
            10
        )

        self.create_timer(1.0, self.user_prompt_timer_callback)

    def coordinates_callback(self, msg):
        self.get_logger().info(f"Objects detected: {msg.label}")

        self.detected_objects.add(msg.label)
        self.object_coordinates[msg.label] = (msg.x, msg.y, msg.z)

    def user_prompt_timer_callback(self):
        if self.detected_objects and not self.is_waiting_for_user:
            self.is_waiting_for_user = True
            self.prompt_user()
            self.is_waiting_for_user = False

    def prompt_user(self):

        if self.detected_objects:
            objects_list = ', '.join(self.detected_objects)
            self.get_logger().info(f"Objects detected: {objects_list}.")

            selected_object = self.object_to_pick

            if selected_object in self.detected_objects:
                self.get_logger().info(f"Moving to pick up {selected_object}...")
                x, y, z = self.object_coordinates[selected_object]

                position = [x, y, z + 0.172]
                quat_xyzw = [0.0, 1.0, 0.0, 0.0]  # Assuming a default orientation

                self.get_logger().info("Opening gripper...") #Open gripper in case it is not already open
                with TimeLog(self.get_logger().info, "Initial opening gripper"):
                    open_gripper()
                
                self.get_logger().info(f"Moving to {{position: {list(position)}, quat_xyzw: {list(quat_xyzw)}}}")
                with TimeLog(self.get_logger().info, "Object position"):
                    self.moveit2.move_to_pose(position=position, quat_xyzw=quat_xyzw, cartesian=False)

                self.get_logger().info("Closing gripper...")
                with TimeLog(self.get_logger().info, "Closing gripper"):
                    close_gripper()

                if self.drop_location == "bin1":
                    with TimeLog(self.get_logger().info, "Above bin"):
                        self.moveit2.move_to_pose(position=[-0.533, -0.056, 0.450], quat_xyzw=quat_xyzw, cartesian=False)
                    with TimeLog(self.get_logger().info, "Inside bin"):
                        self.moveit2.move_to_pose(position=[-0.533, -0.056, 0.260], quat_xyzw=quat_xyzw, cartesian=False)

                if self.drop_location =="bin2":
                    self.moveit2.move_to_pose(position=[-0.609, -0.157, 0.450], quat_xyzw=quat_xyzw, cartesian=False)
                    self.moveit2.move_to_pose(position=[-0.609, -0.157, 0.260], quat_xyzw=quat_xyzw, cartesian=False)
                
                self.get_logger().info("Opening gripper...")
                open_gripper()

                self.subscription.destroy()

            else:
                self.get_logger().info("f{selected_object} not detected among available objects.")
        
        else:
            self.get_logger().info("No objects detected.")

        self.moveit2.move_to_pose(position=[-0.381, -0.257, 0.493], quat_xyzw=quat_xyzw, cartesian=False)
        
        self.detected_objects.clear()
        self.object_coordinates.clear()

    def timer_callback(self):
        if self.is_waiting_for_user:
            self.get_logger().info("User did not respond. Continuing to detect objects...")
            self.is_waiting_for_user = False


def main(args=None):
    parser = argparse.ArgumentParser(description='Pick and place robot movement')
    parser.add_argument('--object', dest='object_to_pick', type=str, required=True, 
                        help='Specify the name of the object to pick')
    parser.add_argument('--location', dest='drop_location', type=str, choices=['bin1', 'bin2'], required=True, help='Specify the drop location of the object')

    args, ros_args = parser.parse_known_args()  # Parse known arguments

    rclpy.init(args=ros_args)
    node = XArmMover(object_to_pick=args.object_to_pick, drop_location=args.drop_location)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

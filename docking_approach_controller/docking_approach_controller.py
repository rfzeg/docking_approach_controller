#!/usr/bin/env python3
"""
Minimal docking approach controller node, ROS2
Implents alternating turn and forward movement sequences 
Author: Roberto Zegers
Date: December 2022
License: BSD-3-Clause
"""

from math import pow, atan2, sqrt, pi, asin, hypot

import rclpy
import tf2_ros
from tf_transformations import euler_from_quaternion
from rclpy.node import Node
from geometry_msgs.msg import Vector3, Twist, Pose, Point, Quaternion


class ApproachController(Node):
    def __init__(self):
        super().__init__("docking_approach_controller")
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer, self)

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.cmd_vel_msg = Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(
            x=0.0, y=0.0, z=0.0))
        self.create_timer(0.1, self.timer_cb)
        self.get_logger().info("Docking approach controller started")
        # hard coded proportinal gain for angular velocity
        self.angular_kp = 1.0
        # hard coded heading tolerance (pi/180 = 1 degree)
        self.goal_yaw_tolerance = 2 * 3.14159/180
        # hard coded linear tolerance distance to goal, in meters
        self.tolerance_distance_to_goal = 0.1
        # initialize robot behaviour by aligning the robot's heading to the goal
        self.controller_state = "turn_to_goal"

    def timer_cb(self):
        out = Twist()
        try:
            # update current goal
            dock_transform = self.tf_buffer.lookup_transform(
                "odom", "docking_station_center", rclpy.duration.Duration())
            self.current_goal_pose = self.tf_transform_to_ros_pose(
                dock_transform)

            # update current robot pose
            robot_transform = self.tf_buffer.lookup_transform(
                "odom", "lidar_1_link", rclpy.duration.Duration())  # lidar_1_link, base_link

            self.current_robot_pose = self.tf_transform_to_ros_pose(
                robot_transform)

            self.get_logger().debug("TF lookup success.")
            self.command_robot()
        except tf2_ros.TransformException:
            self.get_logger().warning("TF lookup failed. Stopping robot.")
            # stop robot
            out = Twist()
            self.cmd_pub.publish(out)

    def command_robot(self):
        # Check 1: Goal reached do nothing
        if self.controller_state == "goal_reached":
            pass
        yaw_error = self.err_heading_to_goal(
            self.current_robot_pose, self.current_goal_pose)
        # Check 2: heading to goal is larger than tolerance limits
        if (abs(yaw_error) > self.goal_yaw_tolerance):
            self.turn_to_goal()
        # Check 3: goal has not been reached
        elif self.controller_state != "goal_reached":
            self.go_to_goal()

    def tf_transform_to_ros_pose(self, tf_transform):
        """
        Convert a tf transform to a ROS pose object
        tf_transform: a ROS transform object
        return: a ROS pose (geometry_msgs.msg.Pose)
        """
        # create a new Pose object
        ros_pose = Pose()
        #ros_pose.header = tf_transform.header

        ros_pose.position.x = tf_transform.transform.translation.x
        ros_pose.position.y = tf_transform.transform.translation.y
        ros_pose.position.x = tf_transform.transform.translation.z
        ros_pose.orientation.x = tf_transform.transform.rotation.x
        ros_pose.orientation.y = tf_transform.transform.rotation.y
        ros_pose.orientation.z = tf_transform.transform.rotation.z
        ros_pose.orientation.w = tf_transform.transform.rotation.w

        return ros_pose

    @staticmethod
    def normalize_angle(angle_radians):
        """
        Normalize angles, in radians, to the interval [−pi, pi]
        angle_randians: any angle expressed in radians
        """
        while angle_radians > pi:
            angle_radians -= 2.0 * pi

        while angle_radians < -pi:
            angle_radians += 2.0 * pi

        return angle_radians

    @staticmethod
    def distance_between_poses(pose1, pose2):
        """
        Calculate the distance between two ROS Pose objects in 2D space
        pose1: current robot pose as an ROS pose object
        pose2: current goal pose as an ROS pose object
        atan2 returns distance in meters
        """
        x1 = pose1.position.x
        y1 = pose1.position.y
        x2 = pose2.position.x
        y2 = pose2.position.y

        distance = hypot(x2 - x1, y2 - y1)

        return distance

    @staticmethod
    def get_goal_heading(pose1, pose2):
        """
        find angle in radians from one pose to another in 2D space
        angle in the frame of reference of pose1 and pose2
        atan2 returns angle in rad in the range of [-pi, pi)
        pose1: current robot pose as an ROS pose object
        pose2: current goal pose as an ROS pose object
        """
        delta_y = pose2.position.y - pose1.position.y
        delta_x = pose2.position.x - pose1.position.x
        return atan2(delta_y, delta_x)

    @staticmethod
    def get_angle_error(pose1, pose2):
        delta_y = pose2.position.y - pose1.position.y
        delta_x = pose2.position.x - pose1.position.x

        distance = sqrt(delta_x ** 2 + delta_y ** 2)
        norm_x = delta_x / distance
        norm_y = delta_y / distance
        return atan2(norm_y, norm_x)

    def err_heading_to_goal(self, robot_pose, goal_pose):
        desired_yaw = self.get_angle_error(robot_pose, goal_pose)
        _, _, robot_pose_yaw = euler_from_quaternion(
            [robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w])

        yaw_error = desired_yaw - robot_pose_yaw
        self.get_logger().debug(
            f"yaw_error (not normalized): {yaw_error} = {desired_yaw} (desired yaw) - {robot_pose_yaw} current robot yaw")
        return self.normalize_angle(yaw_error)

    def turn_to_goal(self):
        self.get_logger().info("Turn to goal.", throttle_duration_sec=1)
        yaw_error = self.err_heading_to_goal(
            self.current_robot_pose, self.current_goal_pose)
        # heading to goal is within tolerance limits
        if (abs(yaw_error) > self.goal_yaw_tolerance):
            self.get_logger().info(
                f"Current robot yaw error: {yaw_error}, allowed yaw error: {self.goal_yaw_tolerance}")
            self.controller_state = "turn_to_goal"
            # if controls are flipped prefix a - sign below
            rotational_vel = self.angular_kp * yaw_error
            self.cmd_vel_msg.linear.x = 0.0
            self.cmd_vel_msg.angular.z = rotational_vel
            self.cmd_pub.publish(self.cmd_vel_msg)

        # heading to goal is not within tolerance limits
        else:
            self.controller_state = "go_to_goal"

    def go_to_goal(self):
        self.get_logger().debug("Go to goal.", throttle_duration_sec=1)
        # set x velocity according to distance to goal
        current_distance = self.distance_between_poses(
            self.current_robot_pose, self.current_goal_pose)
        # case 1: far away from goal pose
        if current_distance > self.tolerance_distance_to_goal + 0.8:
            self.get_logger().debug("Go to goal, far away from goal pose")
            self.cmd_vel_msg.linear.x = 1.0
        # case 2: near goal pose
        elif current_distance > self.tolerance_distance_to_goal + 0.2:
            self.get_logger().debug("Go to goal, near goal pose")
            self.cmd_vel_msg.linear.x = 0.5
        # case 3: very close to goal pose
        elif current_distance > self.tolerance_distance_to_goal:
            self.get_logger().debug("Go to goal, very close to goal pose")
            self.cmd_vel_msg.linear.x = 0.1
        # case 4: goal reached or withing tolerance distance to goal
        else:
            self.get_logger().info("Goal reached within tolerance distance to goal")
            self.cmd_vel_msg.linear.x = 0.0
            self.cmd_vel_msg.angular.z = 0.0
            self.controller_state = "goal_reached"
        self.cmd_pub.publish(self.cmd_vel_msg)


def main():
    rclpy.init()
    controller = ApproachController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

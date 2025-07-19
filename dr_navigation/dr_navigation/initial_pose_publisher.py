#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from math import cos, sin
import yaml
import os
import time


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

    def publish_initial_pose(self, yaml_path):
        if not yaml_path or not os.path.exists(yaml_path):
            self.get_logger().error(f"Invalid or missing config_path: {yaml_path}")
            return False

        self.get_logger().info(f"Reading initial pose from: {yaml_path}")
        with open(yaml_path, 'r') as file:
            data = yaml.safe_load(file)

        pose = data.get('initial_pose', {})
        x = float(pose.get('x', 0.0))
        y = float(pose.get('y', 0.0))
        yaw = float(pose.get('yaw', 0.0))

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = sin(yaw / 2.0)
        msg.pose.pose.orientation.w = cos(yaw / 2.0)
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685

        time.sleep(1.0)  # Wait for AMCL to be ready
        self.publisher.publish(msg)
        self.get_logger().info(f"Published initial pose: x={x}, y={y}, yaw={yaw}")
        return True


def main():
    rclpy.init()
    node = InitialPosePublisher()

    config_path = node.declare_parameter('config_path', '').get_parameter_value().string_value
    success = node.publish_initial_pose(config_path)

    rclpy.shutdown()


if __name__ == '__main__':
    main()

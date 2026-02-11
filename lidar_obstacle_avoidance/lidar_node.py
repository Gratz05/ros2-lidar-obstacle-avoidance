#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math


class ObstacleAvoidanceNode(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        # Subscriber to LIDAR
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

        # Publisher to control robot
        self.cmd_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)

        self.get_logger().info("Obstacle Avoidance Node Started")

    def scan_callback(self, msg: LaserScan):
        # Get ranges from LIDAR
        ranges = msg.ranges

        # Front region of LIDAR (center 20 degrees)
        front_ranges = ranges[350:360] + ranges[0:10]

        # Remove invalid values
        front_ranges = [r for r in front_ranges if not math.isinf(r)]

        if len(front_ranges) == 0:
            return

        min_distance = min(front_ranges)
        self.get_logger().info(f"Front Distance: {min_distance:.2f}")

        twist = Twist()

        # Decision making
        if min_distance < 0.5:
            # Obstacle detected → Turn
            twist.linear.x = 0.0
            twist.angular.z = 0.5
            self.get_logger().info("Obstacle! Turning...")
        else:
            # Path clear → Move forward
            twist.linear.x = 0.2
            twist.angular.z = 0.0
            self.get_logger().info("Path Clear. Moving forward.")

        self.cmd_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

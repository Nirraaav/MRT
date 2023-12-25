#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import random

class RoverNode(Node):
    def __init__(self):
        super().__init__('d_rover2')
        self.publisher = self.create_publisher(Point, 'location', 10)
        self.timer = self.create_timer(2, self.publish_location)

    def publish_location(self):
        location = Point()
        location.x = random.uniform(0, 200)
        location.y = random.uniform(0, 200)
        location.z = random.uniform(0, 200)
        self.publisher.publish(location)
        self.get_logger().info(f"Published location: ({location.x}, {location.y}, {location.z})")

def main(args=None):
    rclpy.init(args=args)
    rover_node = RoverNode()
    rclpy.spin(rover_node)
    rover_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    

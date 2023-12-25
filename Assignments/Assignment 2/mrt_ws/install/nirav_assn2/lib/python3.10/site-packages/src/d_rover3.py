#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random

class RoverNode(Node):
    def __init__(self):
        super().__init__('d_rover3')
        self.publisher = self.create_publisher(String, 'task', 10)
        self.timer = self.create_timer(1, self.publish_task)

    def publish_task(self):
        generated_integer = random.randint(1, 10)
        msg = String()
        if generated_integer >= 5:
            msg.data = f"Task accomplished"
            self.publisher.publish(msg)
            self.get_logger().info(f"Published: Random Generated Integer is {generated_integer} --> Task accomplished")
        else:
            msg.data = "Mission Failed"
            self.publisher.publish(msg)
            self.get_logger().info(f"Published: Random Generated Integer is {generated_integer} --> Mission Failed")

def main(args=None):
    rclpy.init(args=args)
    rover_node = RoverNode()
    rclpy.spin(rover_node)
    rover_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

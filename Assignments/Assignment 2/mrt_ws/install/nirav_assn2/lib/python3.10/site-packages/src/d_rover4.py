#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nirav_assn2.msg import RoverInfo
from geometry_msgs.msg import Pose
from std_msgs.msg import String
import random

class DaughterRover4(Node):
    def __init__(self):
        super().__init__('d_rover4')
        self.publisher = self.create_publisher(RoverInfo, 'topic4', 10)
        self.timer = self.create_timer(1, self.publish_rover_info)

    def publish_rover_info(self):
        msg = RoverInfo()
        msg.rover_id = random.randint(1, 100)
        msg.battery_level = random.uniform(0, 100)
        msg.current_location = Pose()  
        msg.health_status = random.choice(["Good", "Bad"])
        self.publisher.publish(msg)
        self.get_logger().info("Published rover information")

def main(args=None):
    rclpy.init(args=args)
    rover = DaughterRover4()
    rclpy.spin(rover)
    rover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

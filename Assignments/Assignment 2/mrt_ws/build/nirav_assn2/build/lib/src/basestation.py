#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from nirav_assn2.msg import RoverInfo
from geometry_msgs.msg import Point

class BaseStation(Node):
    def __init__(self):
        super().__init__('basestation')
        self.subscription1 = self.create_subscription(
            Float32,
            'altitude',
            self.listener_callback1,
            10)

        self.subscription2 = self.create_subscription(
            Point,
            'location',
            self.listener_callback2,
            10)

        self.subscription3 = self.create_subscription(
            String,
            'task',
            self.listener_callback3,
            10)
        
        self.subscription4 = self.create_subscription(
            RoverInfo,
            'topic4',
            self.listener_callback4,
            10)

    def listener_callback1(self, msg):
        self.get_logger().info(f"Received altitude: {msg.data}")

    def listener_callback2(self, msg):
        self.get_logger().info(f"Received location: ({msg.x}, {msg.y}, {msg.z})")

    def listener_callback3(self, msg):
        self.get_logger().info(f"Received mission status: {msg.data}")
    
    def listener_callback4(self, msg):
        self.get_logger().info(
            f"Received rover info: ID - {msg.rover_id}, Battery - {msg.battery_level}, "
            f"Location - ({msg.current_location}), Health - {msg.health_status}"
        )

def main(args=None):
    rclpy.init(args=args)
    basestation = BaseStation()
    rclpy.spin(basestation)
    basestation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

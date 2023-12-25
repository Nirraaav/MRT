import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random

class RoverNode(Node):
    def __init__(self):
        super().__init__('rover_node')
        self.publisher = self.create_publisher(Float32, 'altitude', 10)
        self.timer = self.create_timer(1, self.publish_altitude)

    def publish_altitude(self):
        msg = Float32()
        msg.data = random.uniform(0, 100)
        self.publisher.publish(msg)
        self.get_logger().info(f"Daughter 1 altitude: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    rover_node = RoverNode()
    rclpy.spin(rover_node)
    rover_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

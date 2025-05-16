import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class StandoffAngleDetector(Node):
    def __init__(self):
        super().__init__('standoff_detector')
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.callback, 10)

    def callback(self, msg):
        for i, r in enumerate(msg.ranges):
            if 0.05 < r < 0.3:  # likely a standoff hit
                angle = msg.angle_min + i * msg.angle_increment
                self.get_logger().info(f"Standoff detected at angle: {math.degrees(angle):.2f}°")

def main(args=None):
    rclpy.init(args=args)
    node = StandoffAngleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class LaserScanFilter(Node):
    def __init__(self):
        super().__init__('laser_scan_filter')
        
        # self.declare_parameter('standoff_angles', [15.03, 33.58, 48.61, 62.14, 71.66,
        #                                            90.20, 107.74, 118.27, 130.80, 146.33, 164.87])

        self.declare_parameter('standoff_angles', [1.00, 2.00, 3.01, 4.01, 5.01, 6.01, 7.02, 8.02, 9.02, 10.02, 11.03, 12.03, 13.03, 14.03, 
                                                   15.03, 16.04, 17.04, 18.04, 19.04, 20.05, 21.05, 22.05, 23.05, 24.05, 25.06, 26.06, 27.06, 
                                                   28.06, 29.07, 32.07, 33.08, 34.08, 35.08, 42.10, 43.10, 44.10, 45.10, 46.10, 47.11, 48.11, 
                                                   49.11, 50.11, 51.12, 52.12, 53.12, 54.12, 55.13, 59.13, 60.14, 61.14, 62.14, 63.14, 64.15, 
                                                   65.15, 67.15, 68.15, 69.16, 70.16, 71.16, 72.16, 73.17, 74.17, 75.17, 76.17, 89.20, 90.20, 
                                                   91.21, 103.23, 104.24, 105.24, 106.24, 107.24, 108.25, 109.25, 110.25, 111.25, 112.26, 
                                                   115.26, 116.26, 117.27, 118.27, 119.27, 120.27, 121.28, 124.28, 125.28, 126.29, 127.29, 
                                                   128.29, 129.29, 130.30, 131.30, 132.30, 133.30, 134.31, 135.31, 136.31, 137.31, 144.33, 145.33, 
                                                   146.33, 147.33, 148.34, 150.34, 151.34, 152.35, 153.35, 154.35, 155.35, 156.36, 157.36, 158.36, 
                                                   159.36, 160.36, 161.37, 162.37, 163.37, 164.37, 165.38, 166.38, 167.38, 168.38, 169.38, 170.39, 
                                                   171.39, 172.39, 173.39, 174.40, 175.40, 176.40, 177.40, 178.41, 179.41, 190.43, 349.79])
        self.declare_parameter('tolerance_deg', 2.0)
        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_filtered')

        self.standoff_angles = self.get_parameter('standoff_angles').get_parameter_value().double_array_value
        self.tolerance_deg = self.get_parameter('tolerance_deg').value
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value

        self.sub = self.create_subscription(
            LaserScan,
            self.input_topic,
            self.scan_callback,
            10
        )

        self.pub = self.create_publisher(
            LaserScan,
            self.output_topic,
            10
        )

    def scan_callback(self, msg: LaserScan):
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        ranges = np.array(msg.ranges)
        
        num_readings = len(ranges)
        angles_deg = np.degrees(angle_min + np.arange(num_readings) * angle_increment)
        
        for standoff_angle in self.standoff_angles:
            mask = np.abs(angles_deg - standoff_angle) <= self.tolerance_deg
            ranges[mask] = float('inf')

        filtered_msg = LaserScan()
        filtered_msg.header = msg.header
        filtered_msg.angle_min = msg.angle_min
        filtered_msg.angle_max = msg.angle_max
        filtered_msg.angle_increment = msg.angle_increment
        filtered_msg.time_increment = msg.time_increment
        filtered_msg.scan_time = msg.scan_time
        filtered_msg.range_min = msg.range_min
        filtered_msg.range_max = msg.range_max
        filtered_msg.ranges = ranges.tolist()
        filtered_msg.intensities = msg.intensities

        self.pub.publish(filtered_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LaserScanFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

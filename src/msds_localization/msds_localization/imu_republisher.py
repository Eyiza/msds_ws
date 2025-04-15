#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import Imu

'''
    This node is used to republish the IMU data from the "imu/out" topic to the "imu_ekf" topic.
    It is used to change the frame_id of the IMU data to "base_footprint_ekf" so that it can be used in the Kalman filter.
'''

imu_pub = None

def imuCallback(imu):
    global imu_pub # To use the global variable imu_pub
    imu.header.frame_id = "base_footprint_ekf"
    imu_pub.publish(imu)


def main(args=None):
    global imu_pub
    rclpy.init(args=args)
    node = Node('imu_republisher_node')
    time.sleep(1) # Wait for the IMU data to be available
    imu_pub = node.create_publisher(Imu, "imu_ekf", 10)
    imu_sub = node.create_subscription(Imu, "imu/out", imuCallback, 10)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

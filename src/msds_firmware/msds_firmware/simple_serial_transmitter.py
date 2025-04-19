#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

'''
This is a simple serial transmitter node for ROS2.
It subscribes to the serial_transmitter topic and sends the messages to the serial port.
Arduino will receive messages from the serial port defined in the parameters.
This script will send messages to the arduino board using the serial port defined in the parameters.
The parameters are:
- port: The serial port to use. Default is /dev/ttyACM0
- baudrate: The baudrate to use. Default is 115200
'''

class SimpleSerialTransmitter(Node):
    def __init__(self):
        super().__init__("simple_serial_transmitter")

        # Declare parameters
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)

        # Get parameters
        self.port_ = self.get_parameter("port").value
        self.baudrate_ = self.get_parameter("baudrate").value

        # Create a subscription to the serial_transmitter topic
        # The message type is String
        self.sub_ = self.create_subscription(String, "serial_transmitter", self.msgCallback, 10)
        self.sub_

        # Create and open the serial port for communication
        self.arduino_ = serial.Serial(port=self.port_, baudrate=self.baudrate_, timeout=0.1)

    def msgCallback(self, msg):
        self.get_logger().info("New message received, publishing on serial: %s" % self.arduino_.name)

        # Write the message to the serial port
        # The message is a string, so we need to encode it to bytes
        # The encoding is utf-8
        self.arduino_.write(msg.data.encode("utf-8"))


def main(): 
    rclpy.init()

    simple_serial_transmitter = SimpleSerialTransmitter()
    rclpy.spin(simple_serial_transmitter)
    
    simple_serial_transmitter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
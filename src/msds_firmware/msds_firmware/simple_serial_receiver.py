#!/usr/bin/env python3
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

'''
This is a simple serial receiver node for ROS2.
It subscribes to the serial_receiver topic and sends the messages to the serial port.
Arduino will send messages to the serial port defined in the parameters.
This script is used to receive messages from the arduino board using the serial port 
then republish the messages to the serial_receiver topic.
The parameters are:
- port: The serial port to use. Default is /dev/ttyACM0
- baudrate: The baudrate to use. Default is 115200
'''

class SimpleSerialReceiver(Node):
    def __init__(self):
        super().__init__("simple_serial_receiver")

        # Declare parameters
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)

        # Get parameters
        self.port_ = self.get_parameter("port").value
        self.baudrate_ = self.get_parameter("baudrate").value

        # Create a publisher to republish the serial_receiver topic
        self.pub_ = self.create_publisher(String, "serial_receiver", 10)
        self.arduino_ = serial.Serial(port=self.port_, baudrate=self.baudrate_, timeout=0.1)

        self.frequency_ = 0.01
        self.timer_ = self.create_timer(self.frequency_, self.timerCallback)

    def timerCallback(self):
        # Check if the node is still running and the serial port is open
        if rclpy.ok() and self.arduino_.is_open:
            data = self.arduino_.readline() # Read a line from the serial port

            try:
                data.decode("utf-8")
            except:
                return

            msg = String()
            msg.data = str(data)
            self.pub_.publish(msg)


def main():
    rclpy.init()

    simple_serial_receiver = SimpleSerialReceiver()
    rclpy.spin(simple_serial_receiver)
    
    simple_serial_receiver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from msds_msgs.srv import SetMode
import subprocess

class ModeManager(Node):
    def __init__(self):
        super().__init__('mode_manager')

        self.current_mode = 'standby'  # default mode

        # Service to receive and set mode
        self.mode_service = self.create_service(SetMode, 'set_mode', self.set_mode_callback)

        # Publisher to broadcast current mode for other subsystem nodes
        self.mode_switch = self.create_publisher(String, 'robot_mode', 10)
        self.timer = self.create_timer(1.0, self.publish_mode)

        self.get_logger().info(f"Mode Manager Node has been started. Current mode is {self.current_mode}")

    def publish_mode(self):
        msg = String()
        msg.data = self.current_mode
        self.mode_switch.publish(msg)

    def set_mode_callback(self, request, response):
        valid_modes = ['standby', 'mapping', 'delivery', 'manual']
        mode = request.mode.strip().lower()
        if mode in valid_modes:
            self.get_logger().info(f"Switching to {mode} mode...")
            # bash_path = "/home/eyiza/Desktop/msds_ws/bash_scripts/mode_set.sh"
            bash_path = "/home/eyiza/msds_ws/bash_scripts/mode_set.sh" # On Raspberry Pi
            try:
                subprocess.Popen(['bash', bash_path, mode])
                # subprocess.Popen(['ros2', 'launch', launch_file])
                self.current_mode = mode
                response.success = True
                response.message = f"{mode.capitalize()} mode launched successfully."
                self.get_logger().info(response.message)

            except Exception as e:
                self.get_logger().error(f"Failed to launch {mode} mode: {e}")
                response.success = False
                response.message = f"Failed to launch {mode} mode: {e}"
            
        else:
            response.success = False
            response.message = f"Invalid mode: {request.data}. Valid modes are: {valid_modes}"
            self.get_logger().warn(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ModeManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
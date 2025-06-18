import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

'''
This node is used to relay the velocity commands from the controller to the robot.
The controller publishes the velocity commands to the /msds_controller/reference_unstamped topic.
The twist_relay node subscribes to this topic and relays the velocity commands to the /msds_controller/reference topic.
The twist_relay node is used to convert the unstamped velocity commands to stamped velocity commands.
'''

class TwistRelayNode(Node):
    def __init__(self):
        super().__init__("twist_relay")

        # To interface between the controller and twist relay
        self.controller_sub = self.create_subscription(
            Twist,
            "/msds_controller/reference_unstamped",
            self.controller_twist_callback,
            10
        )
        # The final output to the controller
        self.controller_pub = self.create_publisher(
            TwistStamped,
            "/msds_controller/reference",
            10
        )

        # To interface between the joystick and twist relay
        self.joy_sub = self.create_subscription(
            TwistStamped,
            "/input_joy/cmd_vel_stamped",
            self.joy_twist_callback,
            10
        )
        # The final output to the joystick
        self.joy_pub = self.create_publisher(
            Twist,
            "/input_joy/cmd_vel",
            10
        )

    def controller_twist_callback(self, msg):
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.twist = msg
        self.controller_pub.publish(twist_stamped)

    def joy_twist_callback(self, msg):
        twist = Twist()
        twist = msg.twist
        self.joy_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TwistRelayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
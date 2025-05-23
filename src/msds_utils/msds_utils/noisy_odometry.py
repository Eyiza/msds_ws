import rclpy
from rclpy.node import Node
from rclpy.constants import S_TO_NS # S_TO_NS is a constant that converts seconds to nanoseconds
from rclpy.time import Time
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
from tf_transformations import quaternion_from_euler

'''
    The aim of this controller is to simulate the noisy sensors measurement like in a real encoder.
    A noise of 0.05 is added to the wheel radius and a noise of 0.2 is added to the wheel separation base.
'''

class NoisyOdometry(Node):
    def __init__(self):
        super().__init__('noisy_odometry')
        self.wheel_radius = 0.0485 + 0.05  # Wheel radius in meters
        self.wheel_separation_base = 0.7 + 0.2 # Distance between wheels in meters

        self.fl_prev_pos = 0.0 # Front Left Previous Position
        self.fr_prev_pos = 0.0 # Front Right Previous Position
        self.rr_prev_pos = 0.0 # Rear Right Previous Position
        self.rl_prev_pos = 0.0 # Rear Left Previous Position

        # Initialize the previous time
        self.prev_time = self.get_clock().now()

        # Initialize the odometry data - The robot's position in the odom frame
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        # Create a subscriber to the joint states
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )
        # Create a publisher for the odometry data
        self.odometry_publisher = self.create_publisher(
            Odometry,
            'msds_controller/odom_noisy',
            10
        )
        # Fill the Odometry message with invariant parameters
        self.odom_data = Odometry()
        self.odom_data.header.frame_id = "odom"
        self.odom_data.child_frame_id = "base_footprint_ekf"
        self.odom_data.pose.pose.orientation.x = 0.0
        self.odom_data.pose.pose.orientation.y = 0.0
        self.odom_data.pose.pose.orientation.z = 0.0
        self.odom_data.pose.pose.orientation.w = 1.0

        # Fill the TF message
        self.tf_broadcaster = TransformBroadcaster(self)
        self.transform_data = TransformStamped()
        self.transform_data.header.frame_id = "odom"
        self.transform_data.child_frame_id = "base_footprint_noisy"

    def joint_state_callback(self, msg):
        if len(msg.velocity) < 4:
                self.get_logger().warn("Not enough wheel velocities received")
                return
            
        # Extract the wheel velocities from the joint states
        # Order: FL, FR, RL, RR
        wheel_velocities = msg.velocity[:4]

        # Extract the time from the joint states
        current_time = Time.from_msg(msg.header.stamp)
        # Calculate the time difference
        dt = (current_time - self.prev_time).nanoseconds / S_TO_NS
        # Check if the time is 0.0 to avoid division by zero
        if dt == 0.0:
            return
        
        # Update the previous time
        self.prev_time = current_time
        
        # Calculate the linear and angular velocities - Mecanum forward kinematics
        v_x = 0.25 * self.wheel_radius * (wheel_velocities[0] + wheel_velocities[1] + wheel_velocities[2] + wheel_velocities[3])
        v_y = 0.25 * self.wheel_radius * (-wheel_velocities[0] + wheel_velocities[1] - wheel_velocities[3] + wheel_velocities[2])
        w_z =  0.25 * self.wheel_radius / self.wheel_separation_base * (-wheel_velocities[0] + wheel_velocities[1] + wheel_velocities[3] - wheel_velocities[2])

        # Calculate the change in position
        delta_x = (v_x * math.cos(self.theta) - v_y * math.sin(self.theta)) * dt
        delta_y = (v_x * math.sin(self.theta) + v_y * math.cos(self.theta)) * dt
        delta_theta = w_z * dt

        # Update the odometry data
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Normalize the angle
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Compose and publish the odom message
        q = quaternion_from_euler(0.0, 0.0, self.theta)
        self.odom_data.pose.pose.orientation.x = q[0]
        self.odom_data.pose.pose.orientation.y = q[1]
        self.odom_data.pose.pose.orientation.z = q[2]
        self.odom_data.pose.pose.orientation.w = q[3]

        self.odom_data.header.stamp = self.get_clock().now().to_msg()

        # The position of the robot in the odom frame
        self.odom_data.pose.pose.position.x = self.x
        self.odom_data.pose.pose.position.y = self.y

        # The velocities of the robot in the odom frame
        self.odom_data.twist.twist.linear.x = v_x
        self.odom_data.twist.twist.linear.y = v_y
        self.odom_data.twist.twist.angular.z = w_z

        # Publish the odometry data
        self.odometry_publisher.publish(self.odom_data)

        # TF - The transform between the odom and base_footprint 
        self.transform_data.transform.translation.x = self.x
        self.transform_data.transform.translation.y = self.y

        self.transform_data.transform.rotation.x = q[0]
        self.transform_data.transform.rotation.y = q[1]
        self.transform_data.transform.rotation.z = q[2]
        self.transform_data.transform.rotation.w = q[3]
        self.transform_data.header.stamp = self.get_clock().now().to_msg()

        # Publish the transform
        self.tf_broadcaster.sendTransform(self.transform_data)

def main(args=None):
    rclpy.init(args=args)
    node = NoisyOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

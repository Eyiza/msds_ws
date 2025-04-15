#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped
import numpy as np

from rclpy.constants import S_TO_NS # S_TO_NS is a constant that converts seconds to nanoseconds
from rclpy.time import Time
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
from tf_transformations import quaternion_from_euler


class SimpleController(Node):

    def __init__(self):
        super().__init__("simple_controller")
        self.declare_parameter("wheel_radius", 0.033) # The radius of the wheel in meters
        self.declare_parameter("wheel_separation", 0.17) # The distance between the wheels in meters

        self.wheel_radius_ = self.get_parameter("wheel_radius").get_parameter_value().double_value # double_value means the value is a double
        self.wheel_separation_ = self.get_parameter("wheel_separation").get_parameter_value().double_value

        self.get_logger().info("Using wheel radius %d" % self.wheel_radius_)
        self.get_logger().info("Using wheel separation %d" % self.wheel_separation_)

        self.left_wheel_prev_pos_ = 0.0 # Left wheel Previous position
        self.right_wheel_prev_pos_ = 0.0 # Right wheel Previous position

        # Recall that to calculate the velocity of the wheels, we need to know the time between the two positions
        # i.e the previous time and the current time
        self.prev_time_ = self.get_clock().now()

        # The robot's position in the odom frame
        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0 # Orientation of the robot
    
        # Create a publisher to publish the wheel commands
        # The topic name is "simple_velocity_controller/commands"
        # The message type is Float64MultiArray
        self.wheel_cmd_pub_ = self.create_publisher(Float64MultiArray, "simple_velocity_controller/commands", 10)

        # Create a subscription to subscribe to the velocity commands coming from the joystick
        # The topic name is "msds_controller/cmd_vel" which is manually created by us and not existing already
        # The message type is TwistStamped which is a standard message type in ROS2
        self.vel_sub_ = self.create_subscription(TwistStamped, "msds_controller/cmd_vel", self.velCallback, 10)

        # Create a subscription to subscribe to the joint states.
        # The ROS2 control library publishes the joint states to the "joint_states" topic
        # For the simulated robot, it'll be done by the Gazebo plugin
        # In the real robot, it'll be done by the wheel encoders
        self.joint_sub_ = self.create_subscription(JointState,"joint_states", self.jointCallback, 10)

        # Create a publisher to publish the odometry 
        # The topic name is "msds_controller/odom" which is manually created by us and not existing already        
        self.odom_pub_ = self.create_publisher(Odometry, "msds_controller/odom", 10)

        # The conversion matrix is a 2x2 matrix that converts the robot speed to wheel speed
        # Chec my note for formula.
        self.speed_conversion_ = np.array([[self.wheel_radius_/2, self.wheel_radius_/2],
                                           [self.wheel_radius_/self.wheel_separation_, -self.wheel_radius_/self.wheel_separation_]])
        self.get_logger().info("The conversion matrix is %s" % self.speed_conversion_)


        # Fill the Odometry message with invariant parameters
        self.odom_msg_ = Odometry() # Check interface for the message type and the fields needed
        self.odom_msg_.header.frame_id = "odom" # The name of the frame the robot is using to express its movement. It is a fixed frame
        self.odom_msg_.child_frame_id = "base_footprint" # The moving frame that is moving with respect to the fixed one 

        # The values of the odometry message are set to 0.0 because we don't know the position of the robot yet
        # and so that the quaternion is fully normalized
        self.odom_msg_.pose.pose.orientation.x = 0.0
        self.odom_msg_.pose.pose.orientation.y = 0.0
        self.odom_msg_.pose.pose.orientation.z = 0.0
        self.odom_msg_.pose.pose.orientation.w = 1.0

        # Fill the TF message
        # The TF message is used to publish the transform between the odom and base_footprint frames
        self.br_ = TransformBroadcaster(self)
        self.transform_stamped_ = TransformStamped()
        self.transform_stamped_.header.frame_id = "odom"
        self.transform_stamped_.child_frame_id = "base_footprint"


    def velCallback(self, msg):
        # Implements the differential kinematic model
        # Given v and w, calculate the velocities of the wheels

        # TwistStamped is a message type that contains a linear and angular velocity.
        # It contains a header and twist.
        # The header contains a timestamp and a frame_id
        # twist contains linear (Vector3) and angular (Vector3) velocities
        # Vector3 contains x, y, z axis velocities
        robot_speed = np.array([[msg.twist.linear.x],
                                [msg.twist.angular.z]])
        # We used x for linear because the robot is moving in the x axis which is the ground
        # y implies moving left and right which is not the case in differential drive
        # and z is not used because the robot is not moving up and down
        # We used z for angular because the robot is rotating around the z axis
        # x and y implies rotating around impossible axes which would cause the robot to rotate in the air
        # Together they make up V and W

        # Inverse kinematics to get the wheel speeds
        # We need to multiply the inverse of the conversion matrix with the robot speed
        # This outputs a 2x1 matrix representing each wheel's speed.
        # matmul is a numpy function that multiplies two matrices
        wheel_speed = np.matmul(np.linalg.inv(self.speed_conversion_), robot_speed) 

        # When we've calculated the wheel speeds, we need to publish them
        # The message type is Float64MultiArray which contains a MultiArrayDimension
        # and a data field which is a list of floats
        # The data field contains the wheel speeds
        # The MultiArrayDimension is not used in this case
        # The wheel speeds contains a 2x1 matrix with the right wheel rotational speed on top and the left wheel rotational speed on the bottom
        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = [wheel_speed[1, 0], wheel_speed[0, 0]]

        # Publish the wheel speeds to move the robot according to the joystick commands
        self.wheel_cmd_pub_.publish(wheel_speed_msg)


    def jointCallback(self, msg):
        # Implements the inverse differential kinematic model
        # Given the position of the wheels, calculates their velocities
        # then calculates the velocity of the robot wrt the robot frame
        # and then converts it in the global frame and publishes the TF

        # Calculate the difference between the current and previous position of each wheel.
        # It's needed to calulate the speed of each wheel which is change in position over change in time
        # The position of the left wheel is at index 1 and the right wheel is at index 0
        dp_left = msg.position[1] - self.left_wheel_prev_pos_ 
        dp_right = msg.position[0] - self.right_wheel_prev_pos_ 
        dt = Time.from_msg(msg.header.stamp) - self.prev_time_

        # dt.nanoseconds is the time in nanoseconds and S_TO_NS is a constant that converts seconds to nanoseconds
        dt_sec = dt.nanoseconds / S_TO_NS # Convert the time to seconds
        # Check if the time is 0.0 to avoid division by zero
        if dt_sec == 0.0:
            self.get_logger().warn("The time is 0.0, skipping the calculation")
            return

        # Actualize the prev pose for the next iteration
        self.left_wheel_prev_pos_ = msg.position[1]
        self.right_wheel_prev_pos_ = msg.position[0]
        self.prev_time_ = Time.from_msg(msg.header.stamp)

        # Calculate the rotational speed of each wheel
        # The rotational speed is the change in position (radians) over the change in time (seconds) - rad/s
        fi_left = dp_left / dt_sec
        fi_right = dp_right / dt_sec

        # Calculate the linear and angular velocity of the robot
        # Formula is in my note
        linear = (self.wheel_radius_ * fi_right + self.wheel_radius_ * fi_left) / 2
        angular = (self.wheel_radius_ * fi_right - self.wheel_radius_ * fi_left) / self.wheel_separation_

        # Calculate the position increment i.e change in position of the overall robot
        # Formula is in my note
        d_s = (self.wheel_radius_ * dp_right + self.wheel_radius_ * dp_left) / 2
        d_theta = (self.wheel_radius_ * dp_right - self.wheel_radius_ * dp_left) / self.wheel_separation_

        # Update the robot's position and orientation
        self.theta_ += d_theta
        self.x_ += d_s * math.cos(self.theta_)
        self.y_ += d_s * math.sin(self.theta_)
        
        # Compose and publish the odom message
        q = quaternion_from_euler(0, 0, self.theta_) # Initialize the quaternion
        self.odom_msg_.pose.pose.orientation.x = q[0]
        self.odom_msg_.pose.pose.orientation.y = q[1]
        self.odom_msg_.pose.pose.orientation.z = q[2]
        self.odom_msg_.pose.pose.orientation.w = q[3]

        self.odom_msg_.header.stamp = self.get_clock().now().to_msg()

        # The position of the robot in the odom frame
        self.odom_msg_.pose.pose.position.x = self.x_
        self.odom_msg_.pose.pose.position.y = self.y_

        # The velocities of the robot in the odom frame
        self.odom_msg_.twist.twist.linear.x = linear
        self.odom_msg_.twist.twist.angular.z = angular

        # Publish the odom message
        # The odom message is published to the "msds_controller/odom" topic
        self.odom_pub_.publish(self.odom_msg_)

        # TF - The transform between the odom and base_footprint frames
        self.transform_stamped_.transform.translation.x = self.x_
        self.transform_stamped_.transform.translation.y = self.y_
        self.transform_stamped_.transform.rotation.x = q[0]
        self.transform_stamped_.transform.rotation.y = q[1]
        self.transform_stamped_.transform.rotation.z = q[2]
        self.transform_stamped_.transform.rotation.w = q[3]
        self.transform_stamped_.header.stamp = self.get_clock().now().to_msg()
        self.br_.sendTransform(self.transform_stamped_)


def main():
    rclpy.init()

    simple_controller = SimpleController()
    rclpy.spin(simple_controller)
    
    simple_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
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

class MecanumOdometry(Node):
    def __init__(self):
        super().__init__('mecanum_odometry_publisher')
        self.declare_parameter('wheel_radius', 0.0485)
        self.declare_parameter('wheel_separation_base', 0.7)
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint_odom')
        self.declare_parameter('joint_state_base', 'velocity')

        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_separation_base = self.get_parameter('wheel_separation_base').get_parameter_value().double_value
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.joint_state_base = self.get_parameter('joint_state_base').get_parameter_value().string_value

        if self.joint_state_base != 'velocity' and self.joint_state_base != 'position':
            self.get_logger().warn("Joint state base is not velocity or position. Defaulting to velocity.")
            self.joint_state_base = 'velocity'

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
            'wheel_states',
            self.joint_state_callback,
            10
        )

        # Create a publisher for the odometry data
        self.odometry_publisher = self.create_publisher(
            Odometry,
            'msds_controller/odom',
            10
        )

        # Initialize odometry data
        self.odom_data = Odometry()
        self.odom_data.header.frame_id = 'odom'
        self.odom_data.child_frame_id = 'base_footprint'
        self.odom_data.pose.pose.orientation.x = 0.0
        self.odom_data.pose.pose.orientation.y = 0.0
        self.odom_data.pose.pose.orientation.z = 0.0
        self.odom_data.pose.pose.orientation.w = 1.0

        # Initialize transform data
        self.tf_broadcaster = TransformBroadcaster(self)
        self.transform_data = TransformStamped()
        self.transform_data.header.frame_id = self.odom_frame
        self.transform_data.child_frame_id = self.base_frame

        self.get_logger().info("Mecanum Odometry Node has been started")

    def joint_state_callback(self, msg):
        v_x = 0.0
        v_y = 0.0
        w_z = 0.0

        if self.joint_state_base == 'velocity':
            if len(msg.velocity) < 4:
                self.get_logger().warn("Not enough wheel velocities received")
                return
            
            # Extract the wheel velocities from the joint states
            # Order: FL, FR, RR, RL
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
            v_y = 0.25 * self.wheel_radius * (-wheel_velocities[0] + wheel_velocities[1] - wheel_velocities[2] + wheel_velocities[3])
            w_z =  0.25 * self.wheel_radius / self.wheel_separation_base * (-wheel_velocities[0] + wheel_velocities[1] + wheel_velocities[2] - wheel_velocities[3])

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
        
        elif self.joint_state_base == 'position':
            if len(msg.position) < 4:
                self.get_logger().warn("Not enough wheel positions received")
                return
            
            # Extract the wheel positions from the joint states
            # Order: FL, FR, RR, RL
            wheel_positions = msg.position[:4]

            # Calculate the change in position
            delta_fl = wheel_positions[0] - self.fl_prev_pos
            delta_fr = wheel_positions[1] - self.fr_prev_pos
            delta_rr = wheel_positions[2] - self.rr_prev_pos
            delta_rl = wheel_positions[3] - self.rl_prev_pos

            # Extract the time from the joint states
            current_time = Time.from_msg(msg.header.stamp)
            # Calculate the time difference
            dt = (current_time - self.prev_time).nanoseconds / S_TO_NS
            # Check if the time is 0.0 to avoid division by zero
            if dt == 0.0:
                return

            # Update the previous positions
            self.fl_prev_pos = wheel_positions[0]
            self.fr_prev_pos = wheel_positions[1]
            self.rr_prev_pos = wheel_positions[2]
            self.rl_prev_pos = wheel_positions[3]

            # Update the previous time
            self.prev_time = current_time

            # Calculate the rotational speed of each wheel
            rs_fl = delta_fl / dt
            rs_fr = delta_fr / dt
            rs_rr = delta_rr / dt
            rs_rl = delta_rl / dt 

            # Calculate the linear and angular velocities - Mecanum forward kinematics
            v_x = 0.25 * self.wheel_radius * (rs_fl + rs_fr + rs_rr + rs_rl)
            v_y = 0.25 * self.wheel_radius * (-rs_fl + rs_fr - rs_rr + rs_rl)
            w_z =  0.25 * self.wheel_radius / self.wheel_separation_base * (-rs_fl + rs_fr + rs_rr - rs_rl)    

            # Calculate the position increment 
            delta_x = 0.25 * self.wheel_radius * (delta_fl + delta_fr + delta_rr + delta_rl)
            delta_y = 0.25 * self.wheel_radius * (-delta_fl + delta_fr - delta_rr + delta_rl)
            delta_theta =  0.25 * self.wheel_radius / self.wheel_separation_base * (-delta_fl + delta_fr + delta_rr - delta_rl)            
            
            # Update the odometry data
            self.x += (delta_x * math.cos(self.theta)) - (delta_y * math.sin(self.theta))
            self.y +=  (delta_x * math.sin(self.theta)) - (delta_y * math.cos(self.theta))
            self.theta += delta_theta

            # Normalize the angle
            self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        else:
            self.get_logger().warn("Joint state base is not velocity or position. Defaulting to velocity.")
            return
        
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

        if self.publish_tf:
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
    node = MecanumOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

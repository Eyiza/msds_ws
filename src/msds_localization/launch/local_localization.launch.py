from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition, IfCondition
import os

def generate_launch_description():

    # Static transform publisher to publish the transform between base_footprint_ekf and imu_link_ekf
    # This is used to transform the IMU data from the IMU frame to the base_footprint frame
    # Translation: (0, 0, 0.103) - The translation is 0.103 meters in the z direction
    # Rotation: (1, 0, 0, 0) - This is a quaternion representation of the rotation
    # The rotation is 180 degrees around the x-axis
    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--x", "0", "--y", "0","--z", "0.103",
                   "--qx", "1", "--qy", "0", "--qz", "0", "--qw", "0",
                   "--frame-id", "base_footprint_ekf",
                   "--child-frame-id", "imu_link_ekf"],
    )

    # Start the ekf_node from the robot_localization package
    # This node is used to fuse the IMU data and the odometry data
    # The ekf_node uses the Extended Kalman Filter to fuse the data
    # The parameters are loaded from the ekf.yaml file
    # The ekf.yaml file contains the parameters for the ekf_node
    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen", # Print the output in the same terminal
        parameters=[os.path.join(get_package_share_directory("msds_localization"), "config", "ekf.yaml")],
    )

    imu_republisher = Node(
        package="msds_localization",
        executable="imu_republisher.py",
    )

    return LaunchDescription([
        static_transform_publisher,
        robot_localization,
        imu_republisher
    ])
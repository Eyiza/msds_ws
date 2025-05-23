from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():

    # Static transform publisher to publish the transform between base_footprint_ekf and imu_link_ekf
    # This is used to transform the IMU data from the IMU frame to the base_footprint frame
    # ros2 run tf2_ros tf2_echo base_footprint imu_link
    # Translation: (-0.087, 0.055, 0.095) - The translation is the offset between the IMU and the base_footprint
    # Rotation: (1, 0, 0, 0) - This is a quaternion representation of the rotation
    # The rotation is 180 degrees around the x-axis
    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--x", "-0.087", "--y", "0.055","--z", "0.095", 
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
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    use_slam = LaunchConfiguration("use_slam")

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false"
    )

    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("msds_firmware"),
            "launch",
            "hardware_interface.launch.py"
        ),
    )

    laser_driver = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("rplidar_ros"),
            "launch",
            "rplidar_c1_launch.py",
        ),
        launch_arguments=[os.path.join(
                get_package_share_directory("msds_bringup"),
                "config",
                "rplidar_ac1.yaml"
            )],
        output="screen"
    )

    laser_driver = Node(
            package="rplidar_ros",
            executable="rplidar_node",
            name="rplidar_node",
            parameters=[os.path.join(
                get_package_share_directory("msds_bringup"),
                "config",
                "rplidar_a1.yaml"
            )],
            output="screen"
    )
    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("msds_controller"),
            "launch",
            "controller.launch.py"
        ),
    )

    imu_driver_node = Node(
        package="msds_firmware",
        executable="mpu6050_driver.py"
    )

    safety_stop = Node(
        package="msds_utils",
        executable="safety_stop",
        output="screen",
    )

    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("msds_localization"),
            "launch",
            "global_localization.launch.py"
        ),
        condition=UnlessCondition(use_slam)
    )

    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("msds_mapping"),
            "launch",
            "slam.launch.py"
        ),
        condition=IfCondition(use_slam)
    )

    return LaunchDescription([
            use_slam_arg,
            hardware_interface,
            laser_driver,
            controller,
            imu_driver_node,
            safety_stop,
            localization,
            slam
        ])
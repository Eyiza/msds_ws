import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    use_slam = LaunchConfiguration("use_slam")

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="true"
    )

    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("msds_firmware"),
            "launch",
            "hardware_interface.launch.py"
        ),
    )

    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("msds_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
                "use_sim_time": "false"
            }.items()
    )

    imu_driver_node = Node(
        package="msds_firmware",
        executable="mpu6050_driver.py"
    )


    laser_filter = Node(
        package="msds_utils",
        executable="laser_filter",
        parameters=[{
            "standoff_angles": [1.25, 1.75, 3.75, 4.24, 4.74, 5.24, 
                                173.51, 174.01, 174.51, 176.50, 177.00, 177.50, 178.00, 
                                -57.17, -56.67, -56.17, -55.67, -55.17, -54.67, 
                                -127.07, -126.57]
        }],
    )

    laser_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rplidar_ros"),
                "launch",
                "rplidar_c1_launch.py"
            )
        ),
        launch_arguments={
            "serial_port": "/dev/rplidar",
            "frame_id": "laser_link",
            "scan_mode": "Standard" # or "DenseBoost"
        }.items()
    )

    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[os.path.join(get_package_share_directory("msds_localization"), "config", "ekf.yaml")],
    )
    

    safety_stop = Node(
        package="msds_utils",
        executable="safety_stop",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "scan_topic": "/scan_filtered",
        }]
    )

    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("msds_localization"),
            "launch",
            "global_localization.launch.py"
        ),
        launch_arguments={
                "use_sim_time": "false"
            }.items(),
        condition=UnlessCondition(use_slam),
    )

    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("msds_mapping"),
            "launch",
            "slam.launch.py"
        ),
        launch_arguments={
                "use_sim_time": "false"
            }.items(),
        condition=IfCondition(use_slam)
    )

    return LaunchDescription([
            use_slam_arg,
            hardware_interface,
            controller,
            imu_driver_node,
            laser_filter,
            laser_driver,
            robot_localization,
            safety_stop,
            localization,
            slam
        ])
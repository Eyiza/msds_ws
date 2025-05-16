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
        default_value="false"
    )

    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("msds_firmware"),
            "launch",
            "hardware_interface.launch.py"
        ),
    )

    # laser_driver = IncludeLaunchDescription(
    #     os.path.join(
    #         get_package_share_directory("rplidar_ros"),
    #         "launch",
    #         "rplidar_c1_launch.py",
    #     ),
    #     launch_arguments=[os.path.join(
    #             get_package_share_directory("msds_bringup"),
    #             "config",
    #             "rplidar_c1.yaml"
    #         )]
    # )

    laser_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rplidar_ros"),
                "launch",
                "rplidar_c1_launch.py"
            )
        ),
        launch_arguments={
            "param_file": os.path.join(
                get_package_share_directory("msds_bringup"),
                "config",
                "rplidar_c1.yaml"
            )
        }.items()
    )
    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("msds_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
                "use_sim_time": False
            }.items()
    )

    imu_driver_node = Node(
        package="msds_firmware",
        executable="mpu6050_driver.py"
    )

    safety_stop = Node(
        package="msds_utils",
        executable="safety_stop",
        output="screen",
        parameters=[{"use_sim_time": False}]
    )

    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("msds_localization"),
            "launch",
            "global_localization.launch.py"
        ),
        launch_arguments={
                "use_sim_time": False
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
                "use_sim_time": False
            }.items(),
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
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.event_handlers import OnProcessExit


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

    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("msds_controller"),
            "launch",
            "joystick_teleop.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "False"
        }.items()
    )

    local_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[os.path.join(get_package_share_directory("msds_localization"), "config", "ekf.yaml")],
    )
    

    laser_filter = Node(
        package="msds_utils",
        executable="laser_filter",
        parameters=[{
            "standoff_angles": [3.25, 3.75, 4.24, 4.74, 5.24, 0.75, 1.25, 
                                1.75, 2.25, 2.75, 3.25, 3.75, 4.24, 4.74, 
                                168.52, 169.02, 169.51, 170.01, 170.51, 171.01,
                                -131.07, -130.57,
                                171.51, -57.67, -57.17, -56.67, -58.17 ]
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

    safety_stop = Node(
        package="msds_utils",
        executable="safety_stop",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "scan_topic": "/scan_filtered",
        }]
    )

    global_localization = IncludeLaunchDescription(
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

    # slam = IncludeLaunchDescription(
    #     os.path.join(
    #         get_package_share_directory("msds_mapping"),
    #         "launch",
    #         "slam.launch.py"
    #     ),
    #     launch_arguments={
    #             "use_sim_time": "false"
    #         }.items(),
    #     condition=IfCondition(use_slam)
    # )

    # navigation = IncludeLaunchDescription(
    #     os.path.join(
    #         get_package_share_directory("msds_navigation"),
    #         "launch",
    #         "navigation.launch.py"
    #     ),
    #     launch_arguments={
    #             "use_sim_time": "false"
    #         }.items(),
    # )

    # rosbridge_server = IncludeLaunchDescription(
    #     XMLLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory("rosbridge_server"),
    #             "launch",
    #             "rosbridge_websocket_launch.xml"
    #         )
    #     ),
    #     launch_arguments={
    #         "allowed_origins": "*",
    #     }.items()
    # )

    return LaunchDescription([
            use_slam_arg,
            hardware_interface,
            imu_driver_node,
            controller,
            joystick,
            local_localization,
            laser_filter,
            laser_driver,
            safety_stop,
            global_localization,
            # slam,
            # navigation,
            # rosbridge_server
        ])

    # return LaunchDescription([
    #     use_slam_arg,

    #     hardware_interface,      # Arduino
    #     imu_driver_node,         # MPU6050
    #     controller,              # ros2_control
    #     laser_driver,            # RPLiDAR
    #     laser_filter,
    #     safety_stop,

    #     TimerAction(period=5.0, actions=[local_localization]),
    #     TimerAction(period=10.0, actions=[global_localization, slam]),
    #     # TimerAction(period=10.0, actions=[navigation])
    # ])
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    msds_controller_pkg = get_package_share_directory('msds_controller')

    use_sim_time_arg = DeclareLaunchArgument(name="use_sim_time", default_value="False",
                                      description="Use simulated time"
    )

    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("msds_firmware"),
            "launch",
            "hardware_interface.launch.py"
        ),
    )

    imu_driver_node = Node(
        package="msds_firmware",
        executable="mpu6050_driver.py"
    )

    mode_manager = Node(
        package="msds_modes",
        executable="mode_manager.py",
        output="screen"
    )

    laser_filter = Node(
        package="msds_utils",
        executable="laser_filter",
        parameters=[{
            "standoff_angles": [-124.58, -124.08, -53.67, -53.18, -52.68, -52.18, 
                                3.75, 4.24, 4.74, 5.74, 6.24, 6.74, 7.24, 175.01, 
                                175.51, 176.01, 176.50, 177.00, 178.00, 178.50, 
                                179.50, 180.00]
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

    twist_mux_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("twist_mux"),
                "launch",
                "twist_mux_launch.py"
            )
        ),
            launch_arguments={
                "cmd_vel_out": "/msds_controller/reference_unstamped",
                "config_locks": os.path.join(msds_controller_pkg, "config", "twist_mux_locks.yaml"),
                "config_topics": os.path.join(msds_controller_pkg, "config", "twist_mux_topics.yaml"),
                "config_joy": os.path.join(msds_controller_pkg, "config", "twist_mux_joy.yaml"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }.items(),
    )

    twist_relay_node = Node(
        package="msds_utils",
        executable="twist_relay",
        name="twist_relay",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}]
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

    rosbridge_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rosbridge_server"),
                "launch",
                "rosbridge_websocket_launch.xml"
            )
        ),
        launch_arguments={
            "allowed_origins": "*",
        }.items()
    )

    return LaunchDescription([
            use_sim_time_arg,
            hardware_interface,
            imu_driver_node,
            laser_filter,
            laser_driver,
            twist_mux_launch,
            twist_relay_node,
            safety_stop,
            mode_manager,
            rosbridge_server
        ])

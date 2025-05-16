import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        "port",
        default_value="/dev/esp"
    )
    port = LaunchConfiguration("port")

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(
                    get_package_share_directory("msds_description"),
                    "urdf",
                    "msds.urdf.xacro",
                ),
                " is_sim:=False",
                " port:=", port,
            ]
        ),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {
             "robot_description": robot_description,
             "use_sim_time": False},
            os.path.join(
                get_package_share_directory("msds_controller"),
                "config",
                "msds_controllers.yaml",
            ),
        ],
    )

    # imu_driver_node = Node(
    #     package="msds_firmware",
    #     executable="mpu6050_driver.py"
    # )

    return LaunchDescription(
        [
            port_arg,
            robot_state_publisher_node,
            controller_manager,
            # imu_driver_node
        ]
    )
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # use_sim_time_arg = DeclareLaunchArgument(name="use_sim_time", default_value="False",
    #                                   description="Use simulated time"
    # )

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

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        parameters=[os.path.join(get_package_share_directory("msds_controller"), "config", "joy_config.yaml"), # Load parameters from the yaml file
                    {"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )

    joy_teleop = Node(
        package="joy_teleop",
        executable="joy_teleop",
        parameters=[os.path.join(get_package_share_directory("msds_controller"), "config", "joy_teleop.yaml"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    local_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[os.path.join(get_package_share_directory("msds_localization"), "config", "ekf.yaml")],
    )
   

    return LaunchDescription([
            # use_sim_time_arg,
            controller,
            joy_node,
            joy_teleop,
            local_localization,
        ])

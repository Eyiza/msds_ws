import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

'''
    This file is used to launch the joystick teleop node which is used to control the robot using a joystick.
    The joystick teleop node is used to convert the joystick input to velocity commands.
'''

def generate_launch_description():

    msds_controller_pkg = get_package_share_directory('msds_controller')

    use_sim_time_arg = DeclareLaunchArgument(name="use_sim_time", default_value="True",
                                      description="Use simulated time"
    )

    # This node is used to start the joystick and read the input from it 
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        parameters=[os.path.join(get_package_share_directory("msds_controller"), "config", "joy_config.yaml"), # Load parameters from the yaml file
                    {"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )
    # Instead of listing all the parameters in this launch file, we can use a yaml file to load all the parameters at once.
    # The yaml file is located in the config folder of the package

    # This node is used to convert the joystick input to velocity commands
    joy_teleop = Node(
        package="joy_teleop",
        executable="joy_teleop",
        parameters=[os.path.join(get_package_share_directory("msds_controller"), "config", "joy_teleop.yaml"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    twist_mux_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("twist_mux"),
                "launch",
                "twist_mux_launch.py"
            )
        ),
            # This node is used to multiplex the velocity commands from the sources and the controller.
            # Twist_mux produces unstamped velocity commands, which are then relayed/published to the twist_relay node 
            # which converts them to stamped velocity commands.
            # config_locks contains what we'll use to start or stop the robot incase of an obstacle
            # config_topics contains the topics that will be used to publish the velocity commands
            # config_joy contains the interface that will allow us to reduce or to increase the velocity of the robot.
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


    return LaunchDescription(
        [
            use_sim_time_arg,
            joy_teleop,
            joy_node,
            twist_mux_launch,
            twist_relay_node,
        ]
    )

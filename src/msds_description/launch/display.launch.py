import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Declare the launch argument for the model file
    # This allows the user to specify the path to the URDF file when launching
    # the launch file. The default value is set to the URDF file in the package directory.
    # The user can override this default value by providing a different path when launching.    
    msds_description_dir = get_package_share_directory("msds_description")

    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(
                                        msds_description_dir, "urdf", "msds.urdf.xacro"
                                        ),
                                      description="Absolute path to robot urdf file")

    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]),
                                       value_type=str)
    # The robot_description parameter is set to the output of the xacro command,
    # which processes the URDF file specified by the user. The Command function
    # is used to execute the xacro command with the provided model argument.
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(msds_description_dir, "rviz", "display.rviz")],
    )

    return LaunchDescription([
        model_arg,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])
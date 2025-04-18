from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

'''
This is a launch file for the msds controller node.
It launches the controller nodes defined in the msds_controllers.yaml
'''

def generate_launch_description():
    # Load the controller managers
    # The first contoller defined in the yaml file is the joint state broadcaster.
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster", # Name of the controller you want to spwan
            "--controller-manager", # Namespace flag
            "/controller_manager", # Namespace
        ],
    )

    wheel_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mecanum_drive_controller",
                   "--controller-manager", 
                   "/controller_manager"
        ],
    )


    return LaunchDescription(
        [
            joint_state_broadcaster_spawner,
            wheel_controller,
        ]
    )
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("msds_description"),
            "launch",
            "gazebo.launch.py"
        ),
    )
    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("msds_controller"),
            "launch",
            "controller.launch.py"
        )
    )
    
    # joystick = IncludeLaunchDescription(
    #     os.path.join(
    #         get_package_share_directory("msds_controller"),
    #         "launch",
    #         "joystick_teleop.launch.py"
    #     ),
    #     launch_arguments={
    #         "use_sim_time": "True"
    #     }.items()
    # )
    
    return LaunchDescription([
        gazebo,
        controller,
        # joystick,
    ])
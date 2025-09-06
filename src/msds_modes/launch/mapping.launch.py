import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

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

    local_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[os.path.join(get_package_share_directory("msds_localization"), "config", "ekf.yaml")],
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
    )

    navigation = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("msds_navigation"),
            "launch",
            "navigation.launch.py"
        ),
        launch_arguments={
                "use_sim_time": "false"
            }.items(),
    )

   
    return LaunchDescription([
            # use_sim_time_arg,
            controller,
            local_localization,
            slam,
            navigation
        ])

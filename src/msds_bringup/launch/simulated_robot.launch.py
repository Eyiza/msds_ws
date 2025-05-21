import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_slam = LaunchConfiguration("use_slam")

    # Argument to determine if SLAM should be used or localization with amcl
    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false"
    )

    # Laser filter node to filter out standoff angles
    laser_filter = Node(
        package="msds_utils",
        executable="laser_filter"
    )

    gazebo_controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("msds_controller"),
            "launch",
            "gazebo_controller.launch.py"
        )
    )

    # Safety stop node to ensure the robot stops in case of emergency
    safety_stop = Node(
        package="msds_utils",
        executable="safety_stop", # safety_stop.py if using Python
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "scan_topic": "/scan_filtered",
            }]
    )

    local_localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("msds_localization"),
            "launch",
            "local_localization.launch.py"
        )
    )

    # Include the localization launch file if SLAM is not used
    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("msds_localization"),
            "launch",
            "global_localization.launch.py"
        ),
        condition=UnlessCondition(use_slam)
    )

    # Include the slam launch file if SLAM is used
    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("msds_mapping"),
            "launch",
            "slam.launch.py"
        ),
        condition=IfCondition(use_slam)
    )

    # To visualize the necessary topics for localization in rviz
    rviz_localization = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(
                get_package_share_directory("msds_localization"),
                "rviz",
                "global_localization.rviz"
            )
        ],
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=UnlessCondition(use_slam)
    )

    # To visualize the necessary topics for mapping in rviz
    rviz_slam = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(
                get_package_share_directory("msds_mapping"),
                "rviz",
                "slam.rviz"
            )
        ],
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_slam)
    )
    
    return LaunchDescription([
        use_slam_arg,
        gazebo_controller,
        laser_filter,
        safety_stop,
        local_localization,
        # localization,
        slam,
        rviz_localization,
        rviz_slam
    ])
import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

'''
This launch file is used to start the SLAM Toolbox and Map Server nodes for the msds robot.
It includes the following nodes:
- slam_toolbox: The SLAM Toolbox node for mapping and localization.
- map_saver_server: The Map Server node for saving the generated map.
- lifecycle_manager: The Lifecycle Manager node for managing the lifecycle of the SLAM Toolbox and Map Server nodes.
- safety_stop: A node for safety stop functionality.
'''

def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_config = LaunchConfiguration("slam_config")

    ros_distro = os.environ["ROS_DISTRO"] # Get the ROS distribution from the environment variable
    lifecycle_nodes = ["map_saver_server"] # Lifecycle nodes to be managed by the lifecycle manager

    if ros_distro != "humble":
        # From distributions above humble, slam_toolbox is a lifecycle node
        lifecycle_nodes.append("slam_toolbox")

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true"
    )

    # Path to the slam configuration file
    slam_config_arg = DeclareLaunchArgument(
        "slam_config",
        default_value=os.path.join(
            get_package_share_directory("msds_mapping"),
            "config",
            "slam_toolbox.yaml"
        ),
        description="Full path to slam yaml file to load"
    )

    
    # nav2_map_saver allows the user to save the map to the file system
    nav2_map_saver = Node(
        package="nav2_map_server",
        executable="map_saver_server",
        name="map_saver_server",
        output="screen",
        parameters=[
            {"save_map_timeout": 5.0}, # 5second. It is the time to wait for the map to be saved
            {"use_sim_time": use_sim_time},
            {"free_thresh_default", "0.196"}, # Default value for free space threshold
            {"occupied_thresh_default", "0.65"}, # Default value for occupied space threshold
        ],
    )

    slam_toolbox = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_config, # Path to the slam configuration file
            {"use_sim_time": use_sim_time},
        ],
    )

    # The lifecycle manager is responsible for managing the lifecycle of the nodes
    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
        output="screen",
        parameters=[
            {"node_names": lifecycle_nodes}, # Nodes to manage
            {"use_sim_time": use_sim_time}, 
            {"autostart": True} # Automatically start the nodes
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        slam_config_arg,
        nav2_map_saver,
        slam_toolbox,
        nav2_lifecycle_manager,
    ])

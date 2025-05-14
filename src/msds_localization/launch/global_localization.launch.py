import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

'''
This launch file is used to start the localization stack for the msds robot.
It includes the following nodes:
- Map server: Loads the map from a YAML file and makes it available to the localization stack.
- AMCL: The Adaptive Monte Carlo Localization node, which uses the map to localize the robot in the environment.
        It implements the Odometry Motion Model, Laser Model, and Resampling algorithms.
- Lifecycle manager: Manages the lifecycle of the map server and AMCL nodes, allowing them to be started and stopped as needed.
The launch file takes the following arguments:
- map_name: The name of the map to load. This should be the name of a directory in the maps folder of the msds_mapping package.
- use_sim_time: Whether to use simulated time (true) or real time (false). This is useful for running the localization stack in a simulation environment.
The launch file is intended to be used in conjunction with the msds_mapping package, which contains the map files and configuration for the robot's localization stack.
'''

def generate_launch_description():
    # Declare the launch arguments
    map_name_arg = DeclareLaunchArgument(
        "map_name",
        default_value="small_house"
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true"
    )

    # Declare the launch argument for the AMCL configuration file
    # This file contains parameters for the AMCL algorithm, such as the number of particles,
    # the laser scan topic, and the initial pose of the robot
    amcl_config_arg = DeclareLaunchArgument(
        "amcl_config",
        default_value=os.path.join(
            get_package_share_directory("msds_localization"),
            "config",
            "amcl.yaml"
        ),
        description="Full path to amcl yaml file to load"
    )

    # Get the launch configuration values
    map_name = LaunchConfiguration("map_name")
    use_sim_time = LaunchConfiguration("use_sim_time")
    amcl_config = LaunchConfiguration("amcl_config")

    # Define the list of lifecycle nodes to be managed by the lifecycle manager
    # Name should be the same as the name of the node
    lifecycle_nodes = ["map_server", "amcl"]


    map_path = PathJoinSubstitution([
        get_package_share_directory("msds_mapping"),
        "maps",
        map_name,
        "map.yaml"
    ])
    
    nav2_map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen", # Display output of the node in the terminal
        parameters=[
            {"yaml_filename": map_path},
            {"use_sim_time": use_sim_time}
        ],
    )

    # Load the AMCL node with the specified configuration file
    # The AMCL node is responsible for localizing the robot in the environment
    # The configuration file contains parameters for the AMCL algorithm
    nav2_amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        emulate_tty=True,
        parameters=[
            amcl_config,
            {"use_sim_time": use_sim_time},
        ],
    )

    # Create the lifecycle manager node to manage lifecycle nodes
    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {"node_names": lifecycle_nodes},
            {"use_sim_time": use_sim_time},
            {"autostart": True}
        ],
    )

    return LaunchDescription([
        map_name_arg,
        use_sim_time_arg,
        amcl_config_arg,
        nav2_map_server,  
        nav2_amcl,
        nav2_lifecycle_manager,
    ])
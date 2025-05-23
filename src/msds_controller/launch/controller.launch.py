import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration,PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

'''
This is a launch file for the msds controller node.
It launches the controller nodes defined in the msds_controllers.yaml
'''

def generate_launch_description():

    msds_controller_pkg = get_package_share_directory('msds_controller')

    use_sim_time_arg = DeclareLaunchArgument(name="use_sim_time", default_value="True",
                                      description="Use simulated time"
    )
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('msds_controller'),
            'config',
            'msds_controllers.yaml',
        ]
    )
 
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
        arguments=["msds_controller",
                   "--controller-manager", 
                   "/controller_manager",
                   '--param-file',
                    robot_controllers,
                    # '--controller-ros-args',
                    # '-r /msds_controller/tf_odometry:=/tf', # Uncomment if using simulation
        ],
    )

    odometry = Node(
        package="msds_utils",
        executable="mecanum_odometry",
        name="mecanum_odometry_publisher",
        output="screen",
        parameters=[{
            "publish_tf": True,
            "base_frame": "base_footprint",
            "joint_state_base": "position"
        }]
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
        executable="twist_relay"
    )


    return LaunchDescription(
        [
            use_sim_time_arg,
            joint_state_broadcaster_spawner,
            wheel_controller,
            odometry,
            twist_mux_launch,
            twist_relay_node,
        ]
    )
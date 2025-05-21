import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler, TimerAction, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition = "True" if ros_distro == "humble" else "False"

    msds_description = get_package_share_directory("msds_description")
    msds_controller_pkg = get_package_share_directory('msds_controller')

    use_sim_time_arg = DeclareLaunchArgument(name="use_sim_time", default_value="True",
                                      description="Use simulated time"
    )
    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(
                                        msds_description, "urdf", "msds.urdf.xacro"
                                        ),
                                      description="Absolute path to robot urdf file"
    )
    world_name_arg = DeclareLaunchArgument(name="world_name", default_value="empty")
    world_path = PathJoinSubstitution([
                    msds_description, 
                    "worlds", 
                    PythonExpression(expression=["'", 
                        LaunchConfiguration("world_name"), "'", " + '.world'"])
                    ]
                )

    use_sim_time = LaunchConfiguration("use_sim_time")

    robot_description = ParameterValue(Command([
            "xacro ",
            LaunchConfiguration("model"),
            " is_ignition:=",
            is_ignition
        ]),
        value_type=str
    )
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('msds_controller'),
            'config',
            'msds_controllers.yaml',
        ]
    )
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            str(Path(msds_description).parent.resolve())
            ]
        )

    # gazebo = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([os.path.join(
    #                 get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
    #             launch_arguments=[
    #                 ("gz_args", [" -v 4", " -r", " empty.sdf"])
    #             ]
    #          )
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output='screen',
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}]
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "msds",
                   '-allow_renaming', 'true'
                   ],
    )    

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster", # Name of the controller you want to spwan
            "--controller-manager", "/controller_manager",
        ],
        parameters=[{"use_sim_time": True}],
    )

    wheel_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["msds_controller",
                   "--controller-manager", "/controller_manager",
                   '--param-file', robot_controllers,
                    '--controller-ros-args', '--ros-args  -r /msds_controller/tf_odometry:=/tf',
        ],
    )

    noisy_odometry = Node(
        package="msds_utils",
        executable="noisy_odometry",
        parameters=[
            {"use_sim_time": use_sim_time},
        ],
    )
    

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"
        ],
        remappings=[
            ('/imu', '/imu/out'), # Remap the topic name to match the one used in the simulation
        ]
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
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        model_arg,
        world_name_arg,
        gazebo_resource_path,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            # launch_arguments=[('gz_args', [' -r -v 1 empty.sdf'])]   
            launch_arguments={
                    "gz_args": PythonExpression(["'", world_path, " -v 4 -r'"])
                }.items() 
        ),    
        # gazebo,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                # on_exit=[joint_state_broadcaster_spawner],
                on_exit=[
                    TimerAction(period=5.0, actions=[joint_state_broadcaster_spawner]),
                ],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[wheel_controller],
                # on_exit=[
                #     TimerAction(period=5.0, actions=[wheel_controller])
                # ],
            )
        ),
        noisy_odometry,
        gz_ros2_bridge,
        robot_state_publisher_node,
        gz_spawn_entity,
        twist_mux_launch,
        twist_relay_node,
    ])
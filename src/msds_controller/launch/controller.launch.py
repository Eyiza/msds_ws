from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition, IfCondition

'''
This is a launch file for the msds controller node.
It launches the controller nodes defined in the msds_controllers.yaml
'''


# The context parameter allows access to the real time value of the arguments of the launch file.
# The args and kwargs parameters are used to pass the parameters to the nodes.
# The function is used to create the nodes for the noisy controller.
def noisy_controller(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_python = LaunchConfiguration("use_python")

    # .perform(context) is used to get the real time value of the arguments passed to the launch file.
    wheel_radius = float(LaunchConfiguration("wheel_radius").perform(context))
    wheel_separation = float(LaunchConfiguration("wheel_separation").perform(context))
    wheel_radius_error = float(LaunchConfiguration("wheel_radius_error").perform(context))
    wheel_separation_error = float(LaunchConfiguration("wheel_separation_error").perform(context))

    noisy_controller_py = Node(
        package="msds_controller",
        executable="noisy_controller.py",
        parameters=[
            {"wheel_radius": wheel_radius + wheel_radius_error,
             "wheel_separation": wheel_separation + wheel_separation_error,
             "use_sim_time": use_sim_time}],
        condition=IfCondition(use_python),
    )

    noisy_controller_cpp = Node(
        package="msds_controller",
        executable="noisy_controller",
        parameters=[
            {"wheel_radius": wheel_radius + wheel_radius_error,
             "wheel_separation": wheel_separation + wheel_separation_error,
             "use_sim_time": use_sim_time}],
        condition=UnlessCondition(use_python),
    )

    # Return a list of nodes to be launched
    return [
        noisy_controller_py,
        noisy_controller_cpp,
    ]


def generate_launch_description():

    # Declare the launch arguments. These are the parameters that can be passed to the launch file. 
    # You can also pass them directlt to their Node definitions in the lanch file.
    # The ones used here also allow it to be changed at runtime instead of hardcoding them in the launch file.
    # The use_sim_time argument is used to determine whether to use simulation time or not.

    # By default, it is set to True which means simulation time will be used.
    # This is useful when running the simulation in Gazebo or any other simulator.
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
    )

    # This is a boolean argument that determines whether to use the simple controller we created or the differential drive controller plugin.
    # By default, it is set to True which means the simple controller will be used.
    use_simple_controller_arg = DeclareLaunchArgument(
        "use_simple_controller",
        default_value="True",
    )

    # This is a boolean argument that determines whether to use the python controller or the C++ controller.
    # By default, it is set to False which means the C++ controller will be used.
    use_python_arg = DeclareLaunchArgument(
        "use_python",
        default_value="False",
    )

    # Thes argument that determines the wheel radius and wheel separation.
    # By default, it is set to 0.033 and 0.17 respectively.
    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.033",
    )
    wheel_separation_arg = DeclareLaunchArgument(
        "wheel_separation",
        default_value="0.17",
    )

    # Introducing Noise
    wheel_radius_error_arg = DeclareLaunchArgument(
        "wheel_radius_error",
        default_value="0.005",
    )
    wheel_separation_error_arg = DeclareLaunchArgument(
        "wheel_separation_error",
        default_value="0.02",
    )
    
    # These are the launch configurations that are used to pass the parameters to the nodes.
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_simple_controller = LaunchConfiguration("use_simple_controller")
    use_python = LaunchConfiguration("use_python")
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")

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

    wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["msds_controller", # Name of the controller you want to spwan which is of the differential drive type
                   "--controller-manager", 
                   "/controller_manager"
        ],
        condition=UnlessCondition(use_simple_controller), # Use the differential drive controller if use_simple_controller is False
    )

    # GroupAction is used to group multiple actions together.
    # It is used to spawn the simple controller and the wheel controller.
    simple_controller = GroupAction(
        condition=IfCondition(use_simple_controller),
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["simple_velocity_controller", 
                           "--controller-manager", 
                           "/controller_manager"
                ]
            ),
            Node(
                package="msds_controller",
                executable="simple_controller.py",
                parameters=[
                    {"wheel_radius": wheel_radius,
                    "wheel_separation": wheel_separation,
                    "use_sim_time": use_sim_time}],
                condition=IfCondition(use_python), # Use the python controller if use_python is True
            ),
            Node(
                package="msds_controller",
                executable="simple_controller",
                parameters=[
                    {"wheel_radius": wheel_radius,
                    "wheel_separation": wheel_separation,
                    "use_sim_time": use_sim_time}],
                condition=UnlessCondition(use_python),
            ),
        ]
    )

    # The OpaqueFunction implements a mechanism to access the real time value of parameters that is assigned to our launch file
    # This allows us to make calculations based on the parameters passed to the launch file.
    # In this case, it is used to pass the parameters to the noisy controller.
    # The OpaqueFunction is used to create a function that can be called at runtime.
    noisy_controller_launch = OpaqueFunction(function=noisy_controller)

    return LaunchDescription(
        [
            use_sim_time_arg,
            use_simple_controller_arg,
            use_python_arg,
            wheel_radius_arg,
            wheel_separation_arg,
            wheel_radius_error_arg,
            wheel_separation_error_arg,
            joint_state_broadcaster_spawner,
            wheel_controller_spawner,
            simple_controller,
            noisy_controller_launch,
        ]
    )
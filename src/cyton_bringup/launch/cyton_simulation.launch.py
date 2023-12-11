# $LICENSE$

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os



def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="cyton_bringup",
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="cyton_controller.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="cyton_description",
            description="Description package with robot URDF/xacro files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="cyton.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_controller",
            default_value="joint_trajectory_controller",
            choices=["joint_trajectory_controller"],
            description="Robot controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "world",
            default_value="box.sdf",
            description="Robot controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name="verbose",
            default_value="false",
            description='Set to "true" to run verbose logging.',
        ),
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name="use_sim",
            default_value="True",
            description='Set to "true" to run simulation',
        ),
    )


    cyton_bringup_dir = get_package_share_directory("cyton_bringup")
    # Initialize Arguments
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")
    robot_controller = LaunchConfiguration("robot_controller")
    world = LaunchConfiguration("world")
    verbose = LaunchConfiguration("verbose")
    use_sim = LaunchConfiguration("use_sim")


    robot_controllers = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controllers_file]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "rviz_1.rviz"])
    

    # Get URDF via xacro
    # xacro_file = os.path.join(get_package_share_directory(description_package), 'urdf', description_file)
    xacro_file = PathJoinSubstitution([FindPackageShare(description_package), 'urdf', description_file])
    robot_description = {
        'robot_description': Command(['xacro ', xacro_file, " use_sim:=", use_sim])
    }

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description,{'use_sim_time': True}],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", [rviz_config_file]],
    )

    # Gazebo nodes
    # world_path = os.path.join(cyton_bringup_dir, "worlds", world)
    world_path = PathJoinSubstitution(
                [FindPackageShare(cyton_bringup_dir), "worlds", world]
            )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_ign_gazebo"), "/launch", "/ign_gazebo.launch.py"]
        ),              launch_arguments = {'ign_args': "-v 4 -r /app/src/cyton_bringup/worlds/box.sdf"}.items()) 

    # Spawn robot
    gazebo_spawn_robot = Node(
        package="ros_gz_sim",#was ros_ign_gazebo
        executable="create",
        name="spawn_cyton",
        arguments=["-name", "cyton", "-topic", "robot_description"],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    gripper_action_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_action_controller", "--controller-manager", "/controller_manager"],
    )

    robot_controllers = [robot_controller]
    robot_controller_spawners = []
    for controller in robot_controllers:
        robot_controller_spawners += [
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "-c", "/controller_manager"],
            )
        ]

    ign_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ign_bridge",
        arguments=[
            "/clock" + "@rosgraph_msgs/msg/Clock" + "[ignition.msgs.Clock",
        ],
        output="screen",
    )

    # Delay loading and activation of `joint_state_broadcaster` after start of ros2_control_node
    delay_joint_state_broadcaster_spawner_after_gazebo_spawn_robot = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gazebo_spawn_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    # Delay loading and activation of `joint_state_broadcaster` after start of ros2_control_node
    delay_gripper_action_controller_spawner_after_gazebo_spawn_robot = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gazebo_spawn_robot,
            on_exit=[gripper_action_controller_spawner],
        )
    )

    # Delay rviz start after Joint State Broadcaster to avoid unnecessary warning output.
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay loading and activation of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawners_after_joint_state_broadcaster_spawner = []
    for controller in robot_controller_spawners:
        delay_robot_controller_spawners_after_joint_state_broadcaster_spawner += [
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[
                        TimerAction(
                            period=3.0,
                            actions=[controller],
                        ),
                    ],
                )
            )
        ]

    return LaunchDescription(
        declared_arguments
        + [
            ign_bridge,
            gazebo,
            gazebo_spawn_robot,
            robot_state_pub_node,
            # delay_rviz_after_joint_state_broadcaster_spawner,
            delay_joint_state_broadcaster_spawner_after_gazebo_spawn_robot,
            delay_gripper_action_controller_spawner_after_gazebo_spawn_robot,
        ]
        + delay_robot_controller_spawners_after_joint_state_broadcaster_spawner
    )
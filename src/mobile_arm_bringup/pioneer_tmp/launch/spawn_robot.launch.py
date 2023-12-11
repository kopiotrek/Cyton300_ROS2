import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.substitutions import FindPackageShare, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="mobile_arm_bringup",
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "arm_controllers_file",
            default_value="cyton_controller.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "platform_controllers_file",
            default_value="pioneer_controller.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="mobile_arm_description",
            description="Description package with robot URDF/xacro files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="main.xacro",
            description="URDF/XACRO description file with the robot.",
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
            name="use_sim",
            default_value="True",
            description='Set to "true" to run simulation',
        ),
    )


    # Initialize Arguments
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    arm_controllers_file = LaunchConfiguration("platform_controllers_file")
    platform_controllers_file = LaunchConfiguration("platform_controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    world = LaunchConfiguration("world")
    use_sim = LaunchConfiguration("use_sim")

    package_name = 'pioneer_bringup'
    world_file_name = 'test_zone_v1.world'

    world_path = PathJoinSubstitution(
                [FindPackageShare(runtime_config_package), "worlds", world]
            )

    robot_state_pub = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(
                package_name), 'launch', 'robot_state_pub.launch.py'
        )]), launch_arguments={'use_sim_time': use_sim, 'use_sim': use_sim}.items()
    )

    
    spawn_entity = Node(
        package="ros_gz_sim",#was ros_ign_gazebo
        executable="create",
        name="spawn_cyton",
        arguments=["-name", "mobile_arm", "-topic", "robot_description", 
                                   '-x', '1.0',
                                   '-y', '1.0',
                                   '-Y', '3.14',],
        output="screen",
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "-c", "/controller_manager"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[diff_drive_spawner],
        )
    )

    # gazebosrv = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [os.path.join(gazebo_ros_package_dir, 'launch', 'gzserver.launch.py')]),
    #     launch_arguments={'verbose': 'true',
    #                       'world': world_file_name}.items()
    # )

    # gazebocli = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [os.path.join(gazebo_ros_package_dir, 'launch', 'gzclient.launch.py')])
    # )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_ign_gazebo"), "/launch", "/ign_gazebo.launch.py"]
        ),              launch_arguments = {'ign_args': f"-v 4 -r {world_path}"}.items()) 

    ign_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ign_bridge",
        arguments=[
            "/clock" + "@rosgraph_msgs/msg/Clock" + "[ignition.msgs.Clock",
        ],
        output="screen",
    )

    return LaunchDescription(
        declared_arguments
        + [
        robot_state_pub,
        gazebo,
        ign_bridge,
        spawn_entity,
        delayed_diff_drive_spawner,
        joint_broad_spawner]
    )
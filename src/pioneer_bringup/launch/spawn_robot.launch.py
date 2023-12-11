import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'pioneer_bringup'
    gazebo_ros_package_dir = get_package_share_directory('gazebo_ros')
    world_file_name = 'test_zone_v1.world'

    robot_state_pub = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(
                package_name), 'launch', 'robot_state_pub.launch.py'
        )]), launch_arguments={'use_sim_time': 'true', ' use_ros2_control': 'true'}.items()
    )



    # spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
    #                     arguments=['-topic', 'robot_description',
    #                                '-entity', 'my_bot',
    #                                '-x', '6.5',
    #                                '-y', '-7.5',
    #                                '-Y', '3.14',
    #                                ],
    #                     output='screen')
    
    spawn_entity = Node(
        package="ros_gz_sim",#was ros_ign_gazebo
        executable="create",
        name="spawn_cyton",
        arguments=["-name", "my_bot", "-topic", "robot_description", 
                                   '-x', '0.0',
                                   '-y', '0.0',
                                   '-Y', '0.0',],
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

    world_file_name = "empty.sdf"
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_ign_gazebo"), "/launch", "/ign_gazebo.launch.py"]
        ),              launch_arguments = {'ign_args': f"-v 4 -r {world_file_name}"}.items()) 

    ign_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ign_bridge",
        arguments=[
            "/clock" + "@rosgraph_msgs/msg/Clock" + "[ignition.msgs.Clock",
        ],
        output="screen",
    )

    return LaunchDescription([
        robot_state_pub,
        gazebo,
        ign_bridge,
        spawn_entity,
        delayed_diff_drive_spawner,
        joint_broad_spawner,
    ])
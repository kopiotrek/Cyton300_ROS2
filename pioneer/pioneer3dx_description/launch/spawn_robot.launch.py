import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'pioneer3dx_description'
    gazebo_ros_package_dir = get_package_share_directory('gazebo_ros')
    world_file_name = 'test_zone_v1.world'

    robot_state_pub = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(
                package_name), 'launch', 'robot_state_pub.launch.py'
        )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot',
                                   '-x', '6.5',
                                   '-y', '-7.5',
                                   '-Y', '3.14',
                                   ],
                        output='screen')

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["diff_drive_controller"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster"],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[diff_drive_spawner],
        )
    )

    gazebosrv = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(gazebo_ros_package_dir, 'launch', 'gzserver.launch.py')]),
        launch_arguments={'verbose': 'true',
                          'world': world_file_name}.items()
    )

    gazebocli = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(gazebo_ros_package_dir, 'launch', 'gzclient.launch.py')])
    )


    return LaunchDescription([
        robot_state_pub,
        gazebosrv,
        gazebocli,
        spawn_entity,
        delayed_diff_drive_spawner,
        joint_broad_spawner,
    ])
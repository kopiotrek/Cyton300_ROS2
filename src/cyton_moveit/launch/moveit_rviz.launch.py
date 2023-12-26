import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_sim = LaunchConfiguration("use_sim")
    declare_use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="False",
        description="Whether simulation is used",
    )

    moveit_config = MoveItConfigsBuilder("cyton", package_name="cyton_moveit").to_moveit_configs()
    rviz_config = PathJoinSubstitution(
        [
            FindPackageShare("cyton_moveit"),
            "config",
            "moveit.rviz",
        ]
    )

    xacro_file = os.path.join(get_package_share_directory('cyton_bringup'), 'urdf', 'cyton.urdf.xacro')
    robot_description_content = {
        'robot_description': Command(['xacro ', xacro_file, " use_sim:=", use_sim])
    }

    rviz_parameters = [
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
        robot_description_content
    ]

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        parameters=rviz_parameters,
    )

    actions = [
        declare_use_sim_arg,
        # SetParameter(name="use_sim_time", value=use_sim),
        rviz_node,
    ]

    return LaunchDescription(actions)

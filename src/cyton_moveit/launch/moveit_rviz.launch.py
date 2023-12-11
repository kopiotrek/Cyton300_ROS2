import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, SetParameter
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declare_description_package_arg = DeclareLaunchArgument(
            "description_package",
            default_value="cyton_description",
            description="Description package with robot URDF/xacro files. Usually the argument \
        is not set, it enables use of a custom description.",
    )
    declare_description_file_arg = DeclareLaunchArgument(
            "description_file",
            default_value="cyton.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
    )
    declare_use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="true",
        description="Start robot in Gazebo simulation.",
    )

    use_sim = LaunchConfiguration("use_sim")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")

    moveit_config = MoveItConfigsBuilder("cyton", package_name="cyton_moveit").to_moveit_configs()
    xacro_file = PathJoinSubstitution([FindPackageShare(description_package), 'urdf', description_file])
    robot_description_content = {
        'robot_description': Command(['xacro ', xacro_file, " use_sim:=", use_sim])
    }

    moveit_config = MoveItConfigsBuilder("cyton", package_name="cyton_moveit").to_moveit_configs()
    rviz_config = PathJoinSubstitution(
        [
            FindPackageShare("cyton_moveit"),
            "config",
            "moveit.rviz",
        ]
    )
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
        declare_description_package_arg,
        declare_description_file_arg,
        # SetParameter(name="use_sim_time", value=use_sim),
        rviz_node,
    ]

    return LaunchDescription(actions)

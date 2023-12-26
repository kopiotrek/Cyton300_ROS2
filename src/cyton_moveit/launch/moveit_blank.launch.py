import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    declare_use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="False",
        description="Start robot in Gazebo simulation.",
    )
    use_sim = LaunchConfiguration("use_sim")


    moveit_config = MoveItConfigsBuilder("cyton", package_name="cyton_moveit").to_moveit_configs()
    xacro_file = os.path.join(get_package_share_directory('cyton_bringup'), 'urdf', 'cyton.urdf.xacro')
    robot_description_content = {
        'robot_description': Command(['xacro ', xacro_file, " use_sim:=", use_sim])
    }

    moveit_config.robot_description = {"robot_description": robot_description_content}
    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": True,
        "capabilities": "",
        "disable_capabilities": "",
        "monitor_dynamics": False,
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 2.0,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.0,
        "use_sim_time": use_sim
    }

    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
    ]

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_params
    )

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
        robot_description_content,
         {"use_sim_time": use_sim} #Problem with unresponsive efector-controlling ball
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
        move_group_node,
        rviz_node
    ]
    return LaunchDescription(actions)

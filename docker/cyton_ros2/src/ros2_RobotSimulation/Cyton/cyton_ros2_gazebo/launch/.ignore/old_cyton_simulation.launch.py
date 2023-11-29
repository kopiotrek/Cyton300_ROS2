#!/usr/bin/python3

# ===================================== COPYRIGHT ===================================== #
#                                                                                       #
#  IFRA (Intelligent Flexible Robotics and Assembly) Group, CRANFIELD UNIVERSITY        #
#  Created on behalf of the IFRA Group at Cranfield University, United Kingdom          #
#  E-mail: IFRA@cranfield.ac.uk                                                         #
#                                                                                       #
#  Licensed under the Apache-2.0 License.                                               #
#  You may not use this file except in compliance with the License.                     #
#  You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0  #
#                                                                                       #
#  Unless required by applicable law or agreed to in writing, software distributed      #
#  under the License is distributed on an "as-is" basis, without warranties or          #
#  conditions of any kind, either express or implied. See the License for the specific  #
#  language governing permissions and limitations under the License.                    #
#                                                                                       #
#  IFRA Group - Cranfield University                                                    #
#  AUTHORS: Mikel Bueno Viso - Mikel.Bueno-Viso@cranfield.ac.uk                         #
#           Seemal Asif      - s.asif@cranfield.ac.uk                                   #
#           Phil Webb        - p.f.webb@cranfield.ac.uk                                 #
#                                                                                       #
#  Date: July, 2022.                                                                    #
#                                                                                       #
# ===================================== COPYRIGHT ===================================== #

# ======= CITE OUR WORK ======= #
# You can cite our work with the following statement:
# IFRA (2022) ROS2.0 ROBOT SIMULATION. URL: https://github.com/IFRA-Cranfield/ros2_RobotSimulation.

# cyton_simulation.launch.py:
# Launch file for the cyton Robot GAZEBO SIMULATION in ROS2 Foxy:

# Import libraries:
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
import yaml

# LOAD FILE:
def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available.
        return None
# LOAD YAML:
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available.
        return None

# ========== **GENERATE LAUNCH DESCRIPTION** ========== #
def generate_launch_description():
    
    # ***** GAZEBO ***** #   
    # DECLARE Gazebo WORLD file:
    world_file = os.path.join(
        get_package_share_directory('cyton_ros2_gazebo'),
        'worlds',
        'cyton.sdf')
    # DECLARE Ignition LAUNCH file:
    ignition = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch'), '/ign_gazebo.launch.py']), #was ros_ign_gazebo
                launch_arguments={'world': world_file}.items(),
             )

    # ***** ROBOT DESCRIPTION ***** #
    # cyton Description file package:
    cyton_description_path = os.path.join(
        get_package_share_directory('cyton_ros2_gazebo'))
    # cyton ROBOT urdf file path:
    xacro_file = os.path.join(cyton_description_path,
                              'urdf',
                              'cyton.urdf.xacro')
    # Generate ROBOT_DESCRIPTION for cyton:
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description_config = doc.toxml()
    robot_description = {'robot_description': robot_description_config}

    # ROBOT STATE PUBLISHER NODE:
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description]
    )

    # # SPAWN ROBOT TO GAZEBO:
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        # arguments=['-topic', 'robot_description',
                        #            '-entity', 'cyton'],
                        name="spawn_cyton",
                        arguments=["-name", "cyton", "-topic", "robot_description"],
                        output='screen')

    # ***** CONTROLLERS ***** #
    controller_manager = Node(
        package="controller_manager",
        executable="spawner",
        parameters=["joint_trajectory_controller", "-c", "/controller_manager"])

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",   
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",   
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
        )
    
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
    )



    # ***** RETURN LAUNCH DESCRIPTION ***** #
    return LaunchDescription([
        ignition, 
        # joint_state_publisher_node,
        node_robot_state_publisher,
        rviz_node,
        spawn_entity,
        controller_manager,
        # load_joint_state_broadcaster,
        # load_joint_trajectory_controller
        
        joint_state_broadcaster_spawner,
        # joint_state_controller, #OBSOLETE dynamic_joint_states 
        # joint_state_broadcaster, #brak unikatowego topica
        # joint_trajectory_controller, #joint_trajectory_controller
        # velocity_controller, #velocity_controller/commands
    ])
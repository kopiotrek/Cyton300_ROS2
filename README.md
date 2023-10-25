# Cyton300_ROS2

Running on ROS Humble

How to run:
1. docker-compose up --build --d
2. docker-compose exec cyton_service bash

Notes:

sudo chmod 777 /dev/ttyusb0

Cyton consists of: 
    - 1 MX-64 dynamixel
    - 6 MX-28 dynamixels
    - 1 AX-12A dynamixels

    <?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
    <xacro:macro name="cyton_ros2_control" params="name initial_positions_file">
        <xacro:property name="initial_positions" value="${load_yaml(initial_positions_file)['initial_positions']}"/>

        <ros2_control name="${name}" type="system">
            <hardware>
              <plugin>dynamixel_hardware/DynamixelHardware</plugin>
              <param name="usb_port">/dev/ttyUSB0</param>
              <param name="baud_rate">57600</param>
              <!-- <param name="use_dummy">true</param> -->
            </hardware>
            <!-- <joint name="joint1_joint"> -->
            <joint name="joint1">
                <param name="id">1</param>
                <command_interface name="position"/>
                <state_interface name="position">
                  <param name="initial_value">${initial_positions['joint1']}</param>
                </state_interface>
                <state_interface name="velocity"/>
            </joint>
            <!-- <joint name="joint2_joint"> -->
            <joint name="joint2">
                <param name="id">2</param>
                <command_interface name="position"/>
                <state_interface name="position">
                  <param name="initial_value">${initial_positions['joint2']}</param>
                </state_interface>
                <state_interface name="velocity"/>
            </joint>
            <!-- <joint name="joint3_joint"> -->
            <joint name="joint3">
                <param name="id">3</param>
                <command_interface name="position"/>
                <state_interface name="position">
                  <param name="initial_value">${initial_positions['joint3']}</param>
                </state_interface>
                <state_interface name="velocity"/>
            </joint>
            <!-- <joint name="joint4_joint"> -->
            <joint name="joint4">
                <param name="id">4</param>
                <command_interface name="position"/>
                <state_interface name="position">
                  <param name="initial_value">${initial_positions['joint4']}</param>
                </state_interface>
                <state_interface name="velocity"/>
            </joint>
            <!-- <joint name="joint5_joint"> -->
            <joint name="joint5">
                <param name="id">5</param>
                <command_interface name="position"/>
                <state_interface name="position">
                  <param name="initial_value">${initial_positions['joint5']}</param>
                </state_interface>
                <state_interface name="velocity"/>
            </joint>
            <!-- <joint name="joint6_joint"> -->
            <joint name="joint6">
                <param name="id">6</param>
                <command_interface name="position"/>
                <state_interface name="position">
                  <param name="initial_value">${initial_positions['joint6']}</param>
                </state_interface>
                <state_interface name="velocity"/>
            </joint>
            <!-- <joint name="joint7_joint"> -->
            <joint name="joint7">
                <param name="id">7</param>
                <command_interface name="position"/>
                <state_interface name="position">
                  <param name="initial_value">${initial_positions['joint7']}</param>
                </state_interface>
                <state_interface name="velocity"/>
            </joint>
            <!-- <joint name="gripper_joint">
                <command_interface name="position"/>
                <state_interface name="position">
                  <param name="initial_value">${initial_positions['gripper_joint']}</param>
                </state_interface>
                <state_interface name="velocity"/>
            </joint> -->
            <!-- <joint name="mimic_gripper_joint">
                <state_interface name="position">
                  <param name="initial_value">${initial_positions['mimic_gripper_joint']}</param>
                </state_interface>
                <state_interface name="velocity"/>
            </joint> -->

        </ros2_control>
    </xacro:macro>
</robot>

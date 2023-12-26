# Cyton300_ROS2

Running on ROS Humble

How to run:
1. docker-compose -f compose.simulation.yaml up

Notes:

sudo chmod 777 /dev/ttyUSB0


Cyton consists of: 
    - 1 MX-64 dynamixel
    - 6 MX-28 dynamixels
    - 1 AX-12A dynamixels

# Test arm

ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [joint1, joint2, joint3, joint4, joint5, joint6, joint7],
    points: [
        { positions: [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0], time_from_start: { sec: 5, nanosec: 0 } },
    ]
  }
}"

Issues:
Drgania - Nie można zapisać PID serw do uruchomienia, brak sterowania prędkością
Problem z zegarem symulacji
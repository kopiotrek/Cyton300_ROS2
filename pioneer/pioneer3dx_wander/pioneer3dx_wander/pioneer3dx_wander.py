import rclpy
from rclpy.node import Node
import time
import random

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid


class Wanderer(Node):

    def __init__(self):
        super().__init__('wanderer')

        self.left_flag = True
        self.last_map_size = 0

        self._handle_parameters()
        self._create_publishers()
        self._create_subcribers()
        self._create_timers()

    def _handle_parameters(self):
        self._declare_default_parameters()
        self.random_turn_timer_period = self.get_parameter('random_turn_timer_period').value
        self.mapping_timeout_period = self.get_parameter('mapping_timeout_period').value

    def _declare_default_parameters(self):
        self.declare_parameter('random_turn_timer_period', 30.0)
        self.declare_parameter('mapping_timeout_period', 100.0)

    def _create_subcribers(self):
        self.lidar_subscriber = self.create_subscription(LaserScan, '/r0/scan_raw',  self.lidar_callback, 10)
        self.map_subscriber = self.create_subscription(OccupancyGrid, '/map',  self.map_callback, 10)
        
    def _create_publishers(self):
        self.velocity_publisher = self.create_publisher(Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)

    def _create_timers(self):
        self.random_turn_timer = self.create_timer(self.random_turn_timer_period, self.random_turn_timer_callback)
        self.mapping_timeout = self.create_timer(self.mapping_timeout_period, self.mapping_timeout_callback)

    def lidar_callback(self, msg: LaserScan):
        lidar_points = len(msg.ranges)

        if lidar_points == 0:
            return
        
        cmd = Twist()
        cmd.linear.x = 0.5

        middle_start = lidar_points // 2 - 100
        middle_end = middle_start + 200

        right_end = middle_start
        right_start = right_end - 100

        left_start = middle_end
        left_end = left_start + 100

        if any(value < 1.0 for value in msg.ranges[middle_start:middle_end]):
            cmd.linear.x = 0.0

            right_average = sum(msg.ranges[right_start:right_end]) / len(msg.ranges[right_start:right_end])
            left_average = sum(msg.ranges[left_start:left_end]) / len(msg.ranges[left_start:left_end])

            if left_average > right_average:
                cmd.angular.z = 0.5
            else:
                cmd.angular.z = -0.5

        self.velocity_publisher.publish(cmd)

    def random_turn_timer_callback(self):
        duration = 4
        start_time = time.time()
        left_flag = random.choice([True, False])

        while time.time() - start_time < duration:
            cmd = Twist()
            if left_flag:
                cmd.angular.z = 0.5
            else:
                cmd.angular.z = -0.5

            self.velocity_publisher.publish(cmd)

            time.sleep(0.1) 

    def map_callback(self, msg: OccupancyGrid):
        print(len(msg.data))

        if len(msg.data) > self.last_map_size :
            self.mapping_timeout.reset()

        self.last_map_size = len(msg.data)

    def mapping_timeout_callback(self):
        raise SystemExit


def main(args=None):
    rclpy.init(args=args)

    wanderer = Wanderer()
    try:
        rclpy.spin(wanderer)
    except SystemExit:
        print("END MAPPING")
    wanderer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

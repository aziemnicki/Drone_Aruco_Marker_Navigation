from tello_msgs.srv import TelloAction
from tello_msgs.msg import TelloResponse
from tello_interface.srv import TelloState

import time
from enum import Enum

import rclpy
from rclpy.node import Node

from std_msgs.msg import Empty
from nav_msgs.msg import Odometry

class PIDNode(Node):

    def __init__(self):
        super().__init__('pid_node')
        self.tello_sub = self.create_subscription(Odometry, '/repeater/tello_1/pose/info', self.main_callback, 10)
        # self.tello_pub = self.create_publisher(Empty, '', 10)

    def main_callback(self, odom):
        x = round(odom.pose.pose.position.x, 3)
        y = round(odom.pose.pose.position.y, 3)
        z = round(odom.pose.pose.position.z, 3)
        self.get_logger().info(f'X: {x}, Y: {y}, Z: {z}')
        time.sleep(0.2)

def main(args=None):
    rclpy.init()

    pid = PIDNode()

    rclpy.spin(pid)
    pid.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
        

    
from tello_msgs.srv import TelloAction
from tello_msgs.msg import TelloResponse
from tello_interface.srv import TelloState

import time
from enum import Enum

import rclpy
from rclpy.node import Node

from std_msgs.msg import Empty
from ros2_aruco_interfaces.msg import ArucoMarkers

class ArucoNode(Node):

    def __init__(self):
        super().__init__('aruco_node')
        self.aruco_sub = self.create_subscription(ArucoMarkers, '/aruco_markers', self.main_callback, 10)
        # self.tello_pub = self.create_publisher(Empty, '', 10)

    def main_callback(self, marker):
        for marker_pose in marker.poses:
            x = round(marker_pose.position.x, 3)
            y = round(marker_pose.position.y, 3)
            z = round(marker_pose.position.z, 3)
            self.get_logger().info(f'X: {x}, Y: {y}, Z: {z}')
            ori_x = round(marker_pose.orientation.x, 3)
            ori_y = round(marker_pose.orientation.y, 3)
            ori_z = round(marker_pose.orientation.z, 3)
            self.get_logger().info(f'OriX: {ori_x}, OriY: {ori_y}, OriZ: {ori_z}')
            # self.get_logger().info(f'{marker}')
            time.sleep(0.2)

def main(args=None):
    rclpy.init()

    aruco = ArucoNode()

    rclpy.spin(aruco)
    aruco.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
        

    

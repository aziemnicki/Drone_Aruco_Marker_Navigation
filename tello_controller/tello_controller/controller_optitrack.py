from tello_msgs.srv import TelloAction
from tello_msgs.msg import TelloResponse
from tello_interface.srv import TelloState

import rclpy
from rclpy.node import Node

from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, Pose
from gazebo_msgs.msg import ModelStates
from ros2_aruco_interfaces.msg import ArucoMarkers

import time
from enum import Enum
from scipy.spatial.transform import Rotation
import numpy as np
from threading import Timer
from tello_controller.pid import PIDController


class ControllerNode(Node):
    
    class TelloState(Enum):
        LANDED = 1
        TAKINGOFF = 2
        HOVERING = 3
        FLYING = 4
        LANDING = 5
        NONE = 0

    state = TelloState.LANDED
    next_state = TelloState.NONE
    action_done = False

    pos_x = pos_y = pos_z = ori_roll = ori_pitch = ori_yaw = 0.0

    pid_x = PIDController(1.5, 0.001, 0.5)
    pid_y = PIDController(1.5, 0.001, 0.5)
    pid_z = PIDController(1.5, 0.001, 0.5)
    pid_yaw = PIDController(1.0, 0.0001, 0.5, rotation=True)

    index = 0
    # points = [
    #     [0.0, 1.0, 0.75, 0.0],
    #     [0.0, 1.0, 0.75, 3.14],
    #     [-1.0, 1.0, 0.75, -1.57],
    #     [-2.0, 1.0, 0.75, -1.57],
    #     [-2.0, 0.0, 0.75, 0.0],
    #     [-2.0, -1.0, 0.75, 0.0],
    #     [-1.0, -1.0, 0.75, 1.57],
    #     [0.0, -1.0, 0.75, 3.14],
    #     [0.0, 0.0, 0.75, 3.14]]

    # points = [
    #     [1.0, 0.0, 0.0, 0.0],
    #     [1.0, 0.5, 0.0, 0.0],
    #     [-1.0, 0.5, 0.0, 0.0],
    #     [0.0, 0.0, 0.0, 0.0]]

    visited_aruco = []                    # punkty odwiedzone
    # points = [[2.15, -1.0, 0.75,  3.14]]  # punkt początkowy przed 1 aruco
    points = []
    aruco_dict = {                        # punkty względne do oblecenia konstrukcji "L"
        0: [[0.0, 0.65, 0.0, -4.71], [-1.15, 0.0, 0.0, 0.0]],
        1: [[-0.35, 0.0, 0.0, 4.71], [ 0.0, 0.27, 0.0, 0.0]],
        2: [[0.0, 0.67, 0.0, 0.0]],
        3: [[0.0, 1.07, 0.0, -4.71], [-1.15, 0.0, 0.0, 0.0]],
        4: [[-1.15, 0.0, 0.0, 1.57], [0.0, -0.65, 0.0, 0.0]],
        5: [[0.0, -0.65, 0.0, 1.57], [0.9, 0.0, 0.0, 0.0]],
        6: [[0.1, 0.0, 0.0, -1.57], [0.0, -0.35, 0.0, 0.0]]
        }
    last_marker=False
    flying=True

    def __init__(self):
        super().__init__('controller_node')
        self.tello_controller = self.create_subscription(Empty, '/iisrl/tello_controller', self.main_callback, 10)
        self.tello_response = self.create_subscription(TelloResponse, '/tello_response', self.tello_response_callback, 10)
        self.optitrack_sub = self.create_subscription(Pose, "/optitrack/pose", self.optitrack_callback, 10)
        # self.aruco_sub = self.create_subscription(ArucoMarkers, '/aruco_markers', self.aruco_callback, 10)

        self.tello_service_server = self.create_service(TelloState, '/iisrl/tello_state', self.state_callback)
        self.tello_service_client = self.create_client(TelloAction, '/tello_action')

        self.service_request = TelloAction.Request()

        self.get_logger().info('Node initialized')


    def optitrack_callback(self, msg):
        self.pos_x = msg.position.x
        self.pos_y = msg.position.y
        self.pos_z = msg.position.z

        rot = Rotation.from_quat([msg.orientation.x,
                                  msg.orientation.y,
                                  msg.orientation.z,
                                  msg.orientation.w])
        roll, pitch, yaw = rot.as_euler('xyz')

        self.ori_roll = roll
        self.ori_pitch = pitch
        self.ori_yaw = yaw

        # self.get_logger().info(f"position z: {self.ori_yaw}")

    
    def aruco_callback(self, markers):
        self.get_logger().info(f"wykryte")
        for i, marker_id in enumerate(markers.marker_ids):
            if marker_id == 3:
                self.last_marker = True
                break
            if marker_id not in self.visited_aruco and markers.poses[i].position.z < 0.5:
                self.visited_aruco.append(marker_id)
                for next_move in self.aruco_dict[marker_id]:
                    self.points.append([x + y for x, y in zip(self.points[-1], next_move)])
                self.flying=True


    def state_callback(self, request, response):
        response.state = str(self.state)
        response.value = int(self.state.value)

        return response


    def main_callback(self, msg):
        self.get_logger().info('Node activated')
        self.action_done = False

        self.state = self.TelloState.LANDED
        self.next_state = self.TelloState.TAKINGOFF

        self.controller()


    def controller(self):
        if self.state == self.TelloState.LANDED and self.next_state == self.TelloState.TAKINGOFF:
            self.taking_off_func()

        if self.state == self.TelloState.HOVERING:
            if self.action_done:
                self.landing_func()
            elif self.flying:
                    self.flying_func()
                    self.flying=False
            else:
                Timer(0.1, self.controller).start()


    def tello_response_callback(self, msg):
        if msg.rc == 1:
            self.state = self.next_state
            self.next_state = self.TelloState.NONE

        self.controller()


    def taking_off_func(self):
        self.state = self.TelloState.TAKINGOFF
        self.next_state = self.TelloState.HOVERING

        # start drona
        while not self.tello_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Oczekuje na dostepnosc uslugi Tello...")

        self.service_request.cmd = 'takeoff'
        self.tello_service_client.call_async(self.service_request)
        


    def flying_func(self):
        self.state = self.TelloState.FLYING
        self.next_state = self.TelloState.FLYING

        if self.action_done:
            self.state = self.TelloState.HOVERING
            self.next_state = self.TelloState.NONE
            self.controller()
        else:
            self.mission_func()
    

    def mission_func(self):
        # misja do wykonania
        while not self.tello_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Oczekuje na dostepnosc uslugi Tello...")
            
        # self.get_logger().info(f'index = {self.points[self.index]}')
        self.pid_x.setpoint, self.pid_y.setpoint, self.pid_z.setpoint, self.pid_yaw.setpoint = self.points[self.index]

        # self.pid_x.setpoint, self.pid_y.setpoint, self.pid_z.setpoint, self.pid_yaw.setpoint = [1.0, 0.0, 0.0, 0.0]
        
        dist_err = np.sqrt((self.pos_x - self.pid_x.setpoint)**2 + (self.pos_y - self.pid_y.setpoint)**2)
        angle_err = abs(self.ori_yaw - self.pid_yaw.setpoint)

        self.get_logger().info(f'Dist_err: {dist_err}')
        self.get_logger().info(f'Angle_err: {angle_err}')
        
        if dist_err > 0.15 or angle_err > 0.018:
        # if dist_err > 0.1:
            vel_x_glob = self.pid_x(self.pos_x)
            vel_y_glob = self.pid_y(self.pos_y)
            vel_z_glob = self.pid_z(self.pos_z)
            vel_yaw = self.pid_yaw(self.ori_yaw)

            # vel_x_loc = vel_x_glob
            # vel_y_loc = vel_y_glob

            vel_x_loc = (vel_x_glob * np.cos(self.ori_yaw)) + (vel_y_glob * np.sin(self.ori_yaw))
            vel_y_loc = (-vel_x_glob * np.sin(self.ori_yaw)) + (vel_y_glob * np.cos(self.ori_yaw))

            vel_x_loc = (vel_x_loc * 15) / 1.4
            vel_y_loc = (vel_y_loc * 15) / 1.4

            if vel_x_loc > 1.0 and vel_x_loc < 6.0:
                vel_x_loc = 6.0

            if vel_x_loc < -1.0 and vel_x_loc > -6.0:
                vel_x_loc = -6.0

            if vel_y_loc > 1.0 and vel_y_loc < 6.0:
                vel_y_loc = 6.0

            if vel_y_loc < -1.0 and vel_y_loc > -6.0:
                vel_y_loc = -6.0

            if vel_x_loc > 15:
                vel_x_loc = 15
            elif vel_x_loc < -15:
                vel_x_loc = -15

            if vel_y_loc > 15:
                vel_y_loc = 15
            elif vel_y_loc < -15:
                vel_y_loc = -15

            

            # self.get_logger().info(f"vel x: {vel_x_loc}")
            # self.get_logger().info(f"vel y: {vel_y_loc}")
            # self.get_logger().info(f"vel z: {vel_z_glob}")
            # self.get_logger().info(f"position x: {self.pos_x}")
            # self.get_logger().info(f"position y: {self.pos_y}")
            # self.get_logger().info(f"position z: {self.pos_z}")
            

            self.service_request.cmd = f'rc {-int(vel_y_loc)} {int(vel_x_loc)} {0.0} {int(vel_yaw)}'
            self.get_logger().info(self.service_request.cmd)
            # self.service_request.cmd = f'rc {0 * self.vel_ratio} {10 * self.vel_ratio} {0.0 * self.vel_ratio} {0.0 * self.vel_ratio}'

            self.tello_service_client.call_async(self.service_request)

            Timer(0.1, self.flying_func).start()
        else:    
            self.get_logger().info(f'Goal position reached: {self.pos_z}')
            self.service_request.cmd = f'rc 0.0 0.0 0.0 0.0'
            self.tello_service_client.call_async(self.service_request)

            if self.index == 0:
                self.points.append([self.pos_x, self.pos_y, self.pos_z, self.ori_yaw])

            if self.index < len(self.points)-1:
                self.index += 1
                self.pid_x.reset_state()
                self.pid_y.reset_state()
                self.pid_z.reset_state()
                self.pid_yaw.reset_state()
                self.get_logger().info(f'{self.index}')
            else:
                self.action_done = True
                self.index = 0
            # self.action_done = True

            self.state = self.TelloState.HOVERING
            self.controller()
            

    def landing_func(self):
        self.state = self.TelloState.LANDING
        self.next_state = self.TelloState.LANDED

        # ladowanie drona
        while not self.tello_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Oczekuje na dostepnosc uslugi Tello...")

        self.service_request.cmd = 'land'
        self.tello_service_client.call_async(self.service_request)


def main(args=None):
    rclpy.init()

    cn = ControllerNode()

    rclpy.spin(cn)
    cn.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



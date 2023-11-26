from tello_msgs.srv import TelloAction
from tello_msgs.msg import TelloResponse
from tello_interface.srv import TelloState

import time
from enum import Enum
from scipy.spatial.transform import Rotation
from simple_pid import PID
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from std_msgs.msg import Empty
from gazebo_msgs.msg import ModelStates


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

    pid_x = PID(1.68, 0.86, 1.68)
    pid_y = PID(3.0, 0.1, 2.02)
    pid_z = PID(1.68, 2.0, 10.8)

    # pid_x = PID(1.5, 0.001, 0.5)
    # pid_y = PID(1.5, 0.001, 0.5)
    # pid_z = PID(1.5, 0.001, 0.5)

    pid_yaw = PID(1.0, 0.0001, 0.5)

    def __init__(self):
        super().__init__('controller_node')
        self.tello_controller = self.create_subscription(Empty, '/iisrl/tello_controller', self.main_callback, 10)
        self.tello_response = self.create_subscription(TelloResponse, '/drone1/tello_response', self.tello_response_callback, 10)

        self.tello_position = self.create_subscription(ModelStates, '/gazebo/model_states', self.position_callback, 10,
                                                       )

        self.tello_service_server = self.create_service(TelloState, '/iisrl/tello_state', self.state_callback)
        self.tello_service_client = self.create_client(TelloAction, '/drone1/tello_action')
        self.service_request = TelloAction.Request()

    def position_callback(self, msg):
        idx = msg.name.index('tello_1')

        self.pos_x = msg.pose[idx].position.x
        self.pos_y = msg.pose[idx].position.y
        self.pos_z = msg.pose[idx].position.z

        rot = Rotation.from_quat([msg.pose[idx].orientation.x,
                                  msg.pose[idx].orientation.y,
                                  msg.pose[idx].orientation.z,
                                  msg.pose[idx].orientation.w])
        roll, pitch, yaw = rot.as_euler('xyz')

        self.ori_roll = roll
        self.ori_pitch = pitch
        self.ori_yaw = yaw
        self.get_logger().info("Received position message")
        # self.get_logger().info(f"position x: {self.pos_x}")
        # self.get_logger().info(f"position y: {self.pos_y}")
        # self.get_logger().info(f"position z: {self.pos_z}")
        # self.get_logger().info(f"orientation r: {self.ori_roll}")
        # self.get_logger().info(f"orientation p: {self.ori_pitch}")
        # self.get_logger().info(f"orientation y: {self.ori_yaw}")

    def state_callback(self, request, response):
        response.state = str(self.state)
        response.value = int(self.state.value)

        return response

    def main_callback(self, msg):
        self.get_logger().info("Uruchomiono wezeł")
        self.action_done = False # False, gdy istnieje misja do wykonania; True, gdy testujemy start i ladowanie

        self.state = self.TelloState.LANDED
        self.next_state = self.TelloState.TAKINGOFF

        self.controller()

    def controller(self):
        if self.state == self.TelloState.LANDED and self.next_state == self.TelloState.TAKINGOFF:
            self.taking_off_func()

        if self.state == self.TelloState.HOVERING:
            if self.action_done:
                self.action_done = False
                self.landing_func()
            else:
                self.flying_func()

    def tello_response_callback(self, msg):
        if msg.rc == 1:
            self.state = self.next_state
            self.next_state = self.TelloState.NONE

        self.controller()
        self.get_logger().info("Received Tello response message")

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
            self.next_state = self.TelloState.HOVERING
        else:
            self.mission_func()

    def mission_func(self):
        # misja do wykonania
        ###
        while not self.tello_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Oczekuje na dostepnosc uslugi Tello...")

        self.pid_x.setpoint = 0.0
        self.pid_y.setpoint = 0.0
        self.pid_z.setpoint = 1.5
        self.get_logger().info(f'przed while')

        while np.sqrt((self.pos_x - self.pid_x.setpoint)**2 + (self.pos_y - self.pid_y.setpoint)**2 + (self.pos_z - self.pid_z.setpoint)**2) > 0.01:
            self.get_logger().info(f'Current position: {self.pos_z}')
            self.get_logger().info(f'Current position: {self.pos_z-self.pid_z.setpoint}')
            rclpy.spin_once(self)

            vel_x_glob = self.pid_x(self.pos_x)
            vel_y_glob = self.pid_y(self.pos_y)
            vel_z_glob = self.pid_z(self.pos_z)

            vel_x_loc = (vel_x_glob * np.cos(self.ori_yaw)) + (vel_y_glob * np.sin(self.ori_yaw))
            vel_y_loc = (-vel_x_glob * np.sin(self.ori_yaw)) + (vel_y_glob * np.cos(self.ori_yaw))

            self.service_request.cmd = f'rc {vel_x_loc} {vel_y_loc} {vel_z_glob} 0.0'
            self.tello_service_client.call_async(self.service_request)

        self.get_logger().info(f'Goal position reached: {self.pos_z}')
        self.service_request.cmd = f'rc 0 0 0 0.0'
        self.tello_service_client.call_async(self.service_request)



        self.state = self.TelloState.HOVERING
        self.next_state = self.TelloState.NONE
        self.action_done = True

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
    rclpy.init(args=args)

    cn = ControllerNode()

    rclpy.spin(cn)
    cn.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
    

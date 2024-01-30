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
from scipy.spatial.transform import Rotation
import numpy as np
from threading import Timer
from tello_controller.pid import PIDController


class ControllerNode(Node):

    pos_x = pos_y = pos_z = ori_roll = ori_pitch = ori_yaw = 0.0
    pid_x = PIDController(0.6, 0.001, 0.25)
    pid_y = PIDController(0.6, 0.001, 0.25)
    pid_z = PIDController(0.6, 0.001, 0.25)
    pid_yaw = PIDController(0.7, 0.0, 0.25, rotation=True)

    index = 0
    visited_aruco = []
    points=[]
    # points = [[2.15, -1.0, 0.75, 2.50],[2.15, 0.0, 0.75, 2.50], [2.15, 0.0, 0.75, -2.21], [1.15, 0.0, 0.75, -2.21], [1.15, 0.0, 0.75, 2.50]]
    aruco_dict = {
        0: [[0.0, 0.65, 0.0, 0.0], [0.0, 0.0, 0.0, -4.71], [-1.15, 0.0, 0.0, 0.0]],
        1: [[0.0, 0.27, 0.0, 0.0], [0.0, 0.0, 0.0, 4.71]],
        2: [[0.0, 0.67, 0.0, 0.0]],
        3: [[0.0, 1.07, 0.0, 0.0], [0.0, 0.0, 0.0, -4.71], [-1.15, 0.0, 0.0, 0.0]],
        4: [[0.0, -1.15, 0.0, 0.0], [0.0, 0.0, 0.0, 1.57], [0.0, -0.65, 0.0, 0.0]],
        5: [[0.0, -0.65, 0.0, 0.0], [0.0, 0.0, 0.0, 1.57], [0.9, 0.0, 0.0, 0.0]],
        6: [[0.0, -0.35, 0.0, 0.0], [0.0, 0.0, 0.0, -1.57]]
        }
    aruco = False

    def __init__(self):
        super().__init__('controller_node')
        self.tello_controller = self.create_subscription(Empty, '/iisrl/tello_controller', self.main_callback, 10)
        self.position_sub = self.create_subscription(ModelStates, '/gazebo/model_states', self.position_callback, 10)
        self.aruco_sub = self.create_subscription(ArucoMarkers, '/aruco_markers', self.aruco_callback, 10)

        self.tello_service_client = self.create_client(TelloAction, '/drone1/tello_action')
        self.service_request = TelloAction.Request()

        self.get_logger().info('Node initialized')


    def position_callback(self, msg):
        '''
        Odczytanie aktualnej pozycji drona.
        '''
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


    def aruco_callback(self, markers):
        '''
        Odebranie danych z markera AruCo i dodanie punktów do listy.
        '''
        for i, marker_id in enumerate(markers.marker_ids):
            if marker_id == 7:
                break
            if marker_id not in self.visited_aruco and markers.poses[i].position.z < 0.5:
                self.visited_aruco.append(marker_id)
                for next_move in self.aruco_dict[marker_id]:
                    self.points.append([x + y for x, y in zip(self.points[-1], next_move)])


    def main_callback(self, msg):
        self.get_logger().info('Node activated')

        self.taking_off_func()
        self.mission_func()

        # self.calc_return_path()



    def taking_off_func(self):
        '''
        Start drona i utrzymanie na określonej wysokości.
        '''
        while not self.tello_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Oczekuje na dostepnosc uslugi Tello...")

        self.service_request.cmd = 'takeoff'
        self.tello_service_client.call_async(self.service_request)
        time.sleep(3)
        self.get_logger().info('Takeoff done!')

        pos_x_loc = self.pos_x
        pos_y_loc = self.pos_y
        self.points.append([pos_x_loc, pos_y_loc, 0.75, 3.14] )
        self.get_logger().info(f"{self.points[0]}")



    def mission_func(self):
        '''
        Budowanie mapy położeń znaczników AruCo.
        '''
        while not self.tello_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Oczekuje na dostepnosc uslugi Tello...")

        pos_x_loc = (self.pos_x * np.cos(self.ori_yaw)) + (self.pos_y * np.sin(self.ori_yaw))
        pos_y_loc = (-self.pos_x * np.sin(self.ori_yaw)) + (self.pos_y * np.cos(self.ori_yaw))

        x_setpoint, y_setpoint, z_setpoint, yaw_setpoint = self.points[self.index]

        x_setpoint_loc = (x_setpoint * np.cos(self.ori_yaw)) + (y_setpoint * np.sin(self.ori_yaw))
        y_setpoint_loc = (-x_setpoint * np.sin(self.ori_yaw)) + (y_setpoint * np.cos(self.ori_yaw))
        # x_setpoint_loc = x_setpoint
        # y_setpoint_loc = y_setpoint
        self.get_logger().info(f"setpoint X loc - {round(x_setpoint_loc,3)}")
        self.get_logger().info(f"setpoint y loc - {round(y_setpoint_loc,3)}")
        self.get_logger().info(f"setpoint X - {round(x_setpoint, 3)}")
        self.get_logger().info(f"setpoint y - {round(y_setpoint,3)}")


        self.pid_x.setpoint = x_setpoint_loc
        self.pid_y.setpoint = y_setpoint_loc
        self.pid_z.setpoint = z_setpoint
        self.pid_yaw.setpoint = yaw_setpoint

        # dist_diff = np.sqrt((pos_x_loc - self.pid_x.setpoint)**2 + (pos_y_loc - self.pid_y.setpoint)**2 + (self.pos_z - self.pid_z.setpoint)**2)
        dist_diff = np.sqrt((pos_x_loc - x_setpoint_loc)**2 + (pos_y_loc - y_setpoint_loc)**2 + (self.pos_z - z_setpoint)**2)
        angle_diff = abs(self.pid_yaw.setpoint - self.ori_yaw)

        if dist_diff > 0.05 or angle_diff > 0.018:
            vel_x = self.pid_x(pos_x_loc)
            vel_y = self.pid_y(pos_y_loc)
            vel_z = self.pid_z(self.pos_z)
            vel_yaw = self.pid_yaw(self.ori_yaw)
            # self.get_logger().info(f"vel X - {vel_x}")
            # self.get_logger().info(f"vel Y - {vel_y}")

            self.service_request.cmd = f'rc {vel_x} {vel_y} {vel_z} {vel_yaw}'
            self.tello_service_client.call_async(self.service_request)
            Timer(0.1, self.mission_func).start()
        else:
            self.get_logger().info(f'Goal position reached')
            self.service_request.cmd = f'rc 0.0 0.0 0.0 0.0'
            self.tello_service_client.call_async(self.service_request)
            
            if self.index < len(self.points)-1:
                self.index += 1
                Timer(0.1, self.mission_func).start()
            else:
                self.index = 0
                msg = Empty()
                self.main_callback(msg)
            

    def landing_func(self):   
        '''
        Ladowanie drona.
        '''
        while not self.tello_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Oczekuje na dostepnosc uslugi Tello...")

        self.service_request.cmd = 'land'
        self.tello_service_client.call_async(self.service_request)
        self.get_logger().info('Landing done!')



def main(args=None):
    rclpy.init()
    cn=ControllerNode()
    rclpy.spin(cn)
    cn.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

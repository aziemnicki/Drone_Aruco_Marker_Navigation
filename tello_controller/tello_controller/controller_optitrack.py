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
import copy


class ControllerNode(Node):

    pos_x = pos_y = pos_z = ori_roll = ori_pitch = ori_yaw = 0.0
    pid_x = PIDController(7.5, 0.01, 15)
    pid_y = PIDController(7.5, 0.01, 15)
    pid_z = PIDController(7.5, 0.01, 15)
    pid_yaw = PIDController(1.0, 0.001, 0.5, rotation=True)

    next_id = 0
    fly = False
    index = 0
    visited_aruco = []
    points_loc = []
    points_test = []
    test_it = 0
    points_glob = []
    aruco_dict = {
        0: [[0.0, -0.9, 0.0, 0.0], [1.60, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.57]],
        1: [[-0.2, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, -1.57]],
        2: [[0.0, -0.75, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]],
        3: [[0.0, 1.07, 0.0, 0.0], [0.0, 0.0, 0.0, -4.71], [-1.15, 0.0, 0.0, 0.0]],
        4: [[0.0, -1.15, 0.0, 0.0], [0.0, 0.0, 0.0, 1.57], [0.0, -0.65, 0.0, 0.0]],
        5: [[0.0, -0.65, 0.0, 0.0], [0.0, 0.0, 0.0, 1.57], [0.9, 0.0, 0.0, 0.0]],
        6: [[0.0, -0.35, 0.0, 0.0], [0.0, 0.0, 0.0, -1.57]]
        }
    aruco = False

    def __init__(self):
        super().__init__('controller_node')
        self.tello_controller = self.create_subscription(Empty, '/iisrl/tello_controller', self.main_callback, 10)
        self.optitrack_sub = self.create_subscription(Pose, "/optitrack/pose", self.optitrack_callback, 10)
        self.aruco_sub = self.create_subscription(ArucoMarkers, '/aruco_markers', self.aruco_callback, 10)

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
        self.ori_roll, self.ori_pitch, self.ori_yaw = rot.as_euler('xyz')



    def aruco_callback(self, markers):
        '''
        Odebranie danych z markera AruCo i dodanie punktów do listy.
        '''
        if self.fly == False:
            pos_x_loc = (self.pos_x * np.cos(self.ori_yaw)) + (self.pos_y * np.sin(self.ori_yaw))
            pos_y_loc = (-self.pos_x * np.sin(self.ori_yaw)) + (self.pos_y * np.cos(self.ori_yaw))
            self.points_test = []
            self.test_it = 0

            # self.points_test = [[pos_x_loc, pos_y_loc, 0.75, self.ori_yaw]]
            for i, marker_id in enumerate(markers.marker_ids):
                self.get_logger().info(f'{marker_id}')
                if marker_id != self.next_id:
                    continue
                if marker_id == 3:
                    self.get_logger().info('AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA')
                    self.landing_func()
                    break
                temp_pos = [pos_x_loc, pos_y_loc, 0.75, self.ori_yaw]
                for next_move in self.aruco_dict[marker_id]:
                    # self.points_test.append([x + y for x, y in zip(self.points_test[-1], next_move)])
                    temp_pos[0] += next_move[0]
                    temp_pos[1] += next_move[1]
                    temp_pos[2] += next_move[2]
                    temp_pos[3] += next_move[3]
                    temp_pos2 = copy.deepcopy(temp_pos)
                    self.points_test.append(temp_pos2)
                        
            self.get_logger().info(f'{self.points_test}')
            # self.points_test.pop(0)
                        # self.points_test.append([pos_x_loc, pos_y_loc-1.0, 0.75, self.ori_yaw])
                        # self.points_test.append([pos_x_loc+1.60, pos_y_loc-1.0, 0.75, self.ori_yaw])
                        # self.points_test.append([pos_x_loc+1.60, pos_y_loc-1.0, 0.75, self.ori_yaw + 1.57])
            if 0 < len(self.points_test) < 4:
                self.next_id +=1
                self.fly = True
                self.get_logger().info('Aruco callback!')


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

        pos_x_loc = (self.pos_x * np.cos(self.ori_yaw)) + (self.pos_y * np.sin(self.ori_yaw))
        pos_y_loc = (-self.pos_x * np.sin(self.ori_yaw)) + (self.pos_y * np.cos(self.ori_yaw))
        self.points_glob.append([self.pos_x, self.pos_y, self.pos_z, self.ori_yaw])
        self.points_loc.append([pos_x_loc, pos_y_loc, self.pos_z, self.ori_yaw])
        # self.points_test.append([pos_x_loc, pos_y_loc, 0.75, self.ori_yaw])
        self.get_logger().info(f"{self.points_loc} aaaaaaaaaaaaaaaaaaaaaa")
            # self.get_logger().info(f"setpoint y loc - {round(y_setpoint_loc,3)}")
            # self.get_logger().info(f"setpoint X - {round(x_setpoint, 3)}")
            # self.get_logger().info(f"setpoint y - {round(y_setpoint,3)}")



    def mission_func(self):
        '''
        Budowanie mapy położeń znaczników AruCo.
        '''
        while not self.tello_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Oczekuje na dostepnosc uslugi Tello...")
        if self.fly:
            pos_x_loc = (self.pos_x * np.cos(self.ori_yaw)) + (self.pos_y * np.sin(self.ori_yaw))
            pos_y_loc = (-self.pos_x * np.sin(self.ori_yaw)) + (self.pos_y * np.cos(self.ori_yaw))

            x_setpoint, y_setpoint, z_setpoint, yaw_setpoint = self.points_test[self.test_it]

            # x_setpoint_loc = (x_setpoint * np.cos(self.ori_yaw)) + (y_setpoint * np.sin(self.ori_yaw))
            # y_setpoint_loc = (-x_setpoint * np.sin(self.ori_yaw)) + (y_setpoint * np.cos(self.ori_yaw))
            # x_setpoint_loc = x_setpoint
            # y_setpoint_loc = y_setpoint
            # self.get_logger().info(f"setpoint X loc - {round(x_setpoint_loc,3)}")
            # self.get_logger().info(f"setpoint y loc - {round(y_setpoint_loc,3)}")
            # self.get_logger().info(f"setpoint X - {round(x_setpoint, 3)}")
            # self.get_logger().info(f"setpoint y - {round(y_setpoint,3)}")
            

            self.pid_x.setpoint = x_setpoint
            self.pid_y.setpoint = y_setpoint
            self.pid_z.setpoint = z_setpoint
            self.pid_yaw.setpoint = yaw_setpoint

            # dist_diff = np.sqrt((pos_x_loc - self.pid_x.setpoint)**2 + (pos_y_loc - self.pid_y.setpoint)**2 + (self.pos_z - self.pid_z.setpoint)**2)
            dist_err = np.sqrt((pos_x_loc - self.pid_x.setpoint) ** 2 + (pos_y_loc - self.pid_y.setpoint) ** 2)
            angle_err = abs(self.ori_yaw - self.pid_yaw.setpoint)
            if self.test_it == 2:
                if len(self.points_test) == 3:
                    pos_x_loc2 = (self.pos_x * np.cos(self.ori_yaw + self.aruco_dict[self.next_id-1][self.test_it][3])) + (self.pos_y * np.sin(self.ori_yaw + self.aruco_dict[self.next_id-1][self.test_it][3]))
                    pos_y_loc2 = (-self.pos_x * np.sin(self.ori_yaw + self.aruco_dict[self.next_id-1][self.test_it][3])) + (self.pos_y * np.cos(self.ori_yaw + self.aruco_dict[self.next_id-1][self.test_it][3]))
                    self.get_logger().info(f"ABC: {pos_x_loc2}, {pos_y_loc2}")
                    self.points_test.append([pos_x_loc2, pos_y_loc2, self.pos_z, self.ori_yaw + self.aruco_dict[self.next_id-1][self.test_it][3]])
                self.get_logger().info(f"Angle err: {angle_err}")
                if angle_err > 0.3:
                    vel_yaw = self.pid_yaw(self.ori_yaw)
                    self.get_logger().info(f"Chce sie krecic: {vel_yaw}")
                    self.service_request.cmd = f'rc {0} {0} {0} {-int(vel_yaw * 30)}'
                    self.tello_service_client.call_async(self.service_request)
                else:
                    self.test_it +=1
                    self.get_logger().info(f'Goal orientation reached')

            else:
                if dist_err > 0.15:
                    vel_x = self.pid_x(pos_x_loc)
                    vel_y = self.pid_y(pos_y_loc)
                    vel_z = self.pid_z(self.pos_z)
                    vel_yaw = self.pid_yaw(self.ori_yaw)


                    self.get_logger().info(f"Vel_x: {vel_x}, Vel_y: {vel_y}, angle err: {dist_err}")

                    self.get_logger().info(f"Pos_x_loc: {pos_x_loc}, Pos_y_loc: {pos_y_loc}")
                    self.get_logger().info(f"setpoint_x: {x_setpoint}, setpoint_y: {y_setpoint}")

                    vel_x = (vel_x * 15) / 1.2
                    vel_y = (vel_y * 15) / 1.2

                    # vel_x = max(min(-6, vel_x) if vel_x < 0 else min(15, vel_x), -15 if vel_x < 0 else 6)
                    # vel_y = max(min(-6, vel_y) if vel_y < 0 else min(15, vel_y), -15 if vel_y < 0 else 6)
                    if -2 < vel_x <2:
                        vel_x = 0
                    elif vel_x >15:
                        vel_x = 15
                    elif vel_x < -15:
                        vel_x = -15
                    
                    if -2 < vel_y <2:
                        vel_y = 0
                    elif vel_y >15:
                        vel_y = 15
                    elif vel_y < -15:
                        vel_y = -15

                    # self.get_logger().info(f"vel X - {vel_x}")
                    # self.get_logger().info(f"vel Y - {vel_y}")

                    self.service_request.cmd = f'rc {-int(vel_y)} {int(vel_x)} {0} {-int(vel_yaw * 30)}'
                    self.tello_service_client.call_async(self.service_request)
                    # Timer(0.1, self.mission_func).start()
                else:
                    self.get_logger().info(f'Goal position reached')
                    self.service_request.cmd = f'rc 0 0 0 0'
                    self.tello_service_client.call_async(self.service_request)
                    self.test_it +=1
                    self.pid_x.reset_state()
                    self.pid_y.reset_state()
                    self.pid_z.reset_state()
                    self.pid_yaw.reset_state()
                    if self.test_it == len(self.points_test):
                        self.fly = False
                        msg = Empty()
                        self.main_callback(msg)

        Timer(0.1, self.mission_func).start()

                # if self.index < len(self.points_loc)-1:
                #     self.index += 1
                #     Timer(0.1, self.mission_func).start()
                # else:
                #     self.index = 0
                #     msg = Empty()
                #     self.main_callback(msg)
            

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

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from optitrack_sim.msg import OptiData 
from tf_transformations import quaternion_from_euler
import time
import yaml
import socket

UDP_IP = "192.168.110.2"
UDP_PORT = 12111

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('optitrack_simulator')
        self.twist_publisher_ = self.create_publisher(Pose, '/optitrack/pose', 10)  
        self.receive_from_server()
        
    def receive_from_server(self):
        sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        sock.bind((UDP_IP,UDP_PORT))

        while True:
            data,addr = sock.recvfrom(1024)
            decoded_data = data.decode('utf-8')
            
            try:
                received_object = yaml.safe_load(decoded_data)
            except:
                continue
                
            if not isinstance(received_object,dict) or 'rigid_bodies' not in received_object:
                continue
            
            if not received_object['rigid_bodies']:
                continue 
            
            
            with open('data.txt', 'a') as f:
                f.write(str(received_object))
                
            rigid_bodies = [float(i) for i in received_object['Tello']]
            
            if not rigid_bodies[0]:
                print("Drone not in cage!")
                continue
            
            quaternion = quaternion_from_euler(rigid_bodies[3],rigid_bodies[4],rigid_bodies[5])
            
            pose_msg = Pose()
            print(rigid_bodies)
            # Position
            pose_msg.position.x = rigid_bodies[1]
            pose_msg.position.y = rigid_bodies[2]
            pose_msg.position.z = rigid_bodies[3]
            # Orientation
            pose_msg.orientation.x = quaternion[0]
            pose_msg.orientation.y = quaternion[1]
            pose_msg.orientation.z = quaternion[2]
            pose_msg.orientation.w = quaternion[3]
            self.twist_publisher_.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

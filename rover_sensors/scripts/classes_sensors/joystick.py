#!/usr/bin/env python3

# Python libraries
import rospy
import socket

# ROS messages
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

class Joystick:
    def __init__(self) -> None:
        ip = rospy.get_param("/jetson/ip/value", default = "0.0.0.0")
        port = rospy.get_param("/jetson/port/value", default = 5005)
        self.__kpt = rospy.get_param("/control/kpt/value", default = 1)
        self.__kpr = rospy.get_param("/control/kpr/value", default = 1)

        self.__linear, self.__angular, self.__button = None, None, None
        self.__manual = False
        self.__last_val = 0

        self.__udp_connection(ip, port)

        self.__vel = Twist()
        self.__vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
        self.__manual_pub = rospy.Publisher("/manual_mode", Bool, queue_size = 10)

    def __udp_connection(self, ip:str, port):
        rospy.loginfo("Starting the UPD connection")
        self.__sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.__sock.bind((ip, port))
        rospy.loginfo("Connection succesful")

    def __map_signal(self, signal:int) -> None:
        return signal / 2000 - 1

    def __receive_data(self) -> None:
        data, _ = self.__sock.recvfrom(1024)
        message = data.decode()
        self.__linear, self.__angular, self.__button = map(int, message.split(','))
    
    def __activate_manual(self) -> None:
        if self.__last_val == 0 and self.__button == 1:
            self.stop()
            self.__manual = not self.__manual
            self.__manual_pub.publish(True) if self.__manual else self.__manual_pub.publish(False)
        self.__last_val = self.__button
    
    def manual_control(self) -> None:
        self.__receive_data()
        self.__activate_manual()
        if self.__manual:
            v = self.__map_signal(self.__linear)*self.__kpt
            w = -self.__map_signal(self.__angular)*self.__kpr
            self.__vel.linear.x = 0.0 if abs(v) < 0.03 else v
            self.__vel.angular.z = 0.0 if abs(w) < 0.03 else w
            self.__vel_pub.publish(self.__vel)

    def stop(self) -> None:
        self.__vel.linear.x = 0.0
        self.__vel.angular.z = 0.0
        self.__vel_pub.publish(self.__vel)

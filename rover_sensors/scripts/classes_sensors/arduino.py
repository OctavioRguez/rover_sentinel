#!/usr/bin/env python3

# Python libraries
import rospy
import smbus

# ROS messages
from std_msgs.msg import Int32

class Arduino:
    def __init__(self) -> None:
        self.__addr = 0x12
        self.__bus = smbus.SMBus(1) # I2C

        self.__buzzer_state = 0

        self.__sound_pub = rospy.Publisher("/sensor/sound", Int32, queue_size = 10)
        self.__distance_pub = rospy.Publisher("/sensor/distance", Int32, queue_size = 10)
        rospy.Subscriber("/buzzer", Int32, self.__buzzer_callback)

    def __buzzer_callback(self, msg:Int32) -> None:
        self.__buzzer_state = msg.data
    
    def send_buzzer_data(self) -> None:
        self.__bus.write_byte(self.__addr, self.__buzzer_state)

    def receive_sensor_data(self) -> None:
        # Read 4 bytes
        data = self.__bus.read_i2c_block_data(self.__addr, 0, 4)

        # Convert bytes in values
        sound = (data[0] << 8) + data[1]
        distance = (data[2] << 8) + data[3]
        self.__sound_pub.publish(sound)
        self.__distance_pub.publish(distance)
        rospy.loginfo(sound, distance)

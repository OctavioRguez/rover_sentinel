#!/usr/bin/env python3

# Python libraries
import rospy
import smbus

# ROS messages
from std_msgs.msg import Int8, Float32, Bool

class Arduino:
    def __init__(self) -> None:
        self.__addr = 0x12
        self.__bus = smbus.SMBus(1) # I2C

        self.__sound_pub = rospy.Publisher("/detected/sound", Bool, queue_size = 10)
        self.__distance_pub = rospy.Publisher("/sensor/distance", Float32 , queue_size = 10)
        rospy.Subscriber("/buzzer", Int8, self.__buzzer_callback)

    def __buzzer_callback(self, msg:Int8) -> None:
        self.send_buzzer_data(msg.data)

    def send_buzzer_data(self, data:int) -> None:
        self.__bus.write_byte(self.__addr, data)

    def receive_sensors_data(self) -> None:
        # Read 4 bytes
        data = self.__bus.read_i2c_block_data(self.__addr, 0, 4)

        # Convert bytes in values
        sound = (data[0] << 8) + data[1]
        distance = (data[2] << 8) + data[3] % 1201
        self.__distance_pub.publish(distance/10)
        if (sound <= 660 or sound >= 670):
            self.send_buzzer_data(1)
            self.__sound_pub.publish(True)

    def stop(self) -> None:
        rospy.loginfo("Stopping Arduino Master")

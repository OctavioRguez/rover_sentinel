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
        self.__message = True

        self.__time = rospy.Time.now().to_sec()
        self.__min = float("inf")
        self.__max = 0

        self.__sound_pub = rospy.Publisher("/detected/sound", Bool, queue_size = 10)
        self.__distance_pub = rospy.Publisher("/sensor/distance", Float32 , queue_size = 10)
        self.__ir_left_pub = rospy.Publisher("/sensor/ir/left", Bool, queue_size = 10)
        self.__ir_right_pub = rospy.Publisher("/sensor/ir/right", Bool, queue_size = 10)
        rospy.Subscriber("/buzzer", Int8, self.__buzzer_callback)
        rospy.loginfo(f"Calibrating sound sensor...")

    def __buzzer_callback(self, msg:Int8) -> None:
        self.send_buzzer_data(msg.data)
    
    def __analyze_sound(self, sound:int) -> None:
        if rospy.Time.now().to_sec() - self.__time < 15:
            # Calibrate sound sensor
            self.__min = min(self.__min, sound)
            self.__max = max(self.__max, sound)
        elif (sound < (self.__min - 50) or sound > (self.__max + 50)):
            self.__sound_pub.publish(True)
        elif self.__message:
            rospy.loginfo(f"Finished calibration.\nMin: {self.__min}, Max: {self.__max}")
            self.__message = False

    def send_buzzer_data(self, data:int) -> None:
        self.__bus.write_byte(self.__addr, data)

    def receive_sensors_data(self) -> None:
        # Read 6 bytes
        data = self.__bus.read_i2c_block_data(self.__addr, 0, 6)

        # Convert bytes in values (analogic)
        sound = (data[0] << 8) + data[1]
        self.__analyze_sound(sound)

        distance = (data[2] << 8) + data[3]
        distance = 200 if distance > 300 else distance
        self.__distance_pub.publish(distance/100)

        # Digital values
        left, right = data[4:]
        self.__ir_left_pub.publish(left)
        self.__ir_right_pub.publish(right)

    def stop(self) -> None:
        rospy.loginfo("Stopping Arduino Master")

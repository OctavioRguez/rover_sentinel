#!/usr/bin/env python3

# Python libraries
import rospy

# Classes
from classes_sensors import Joystick

if __name__=='__main__':
    rospy.init_node("JoyStick_Manual_Control")
    rate = rospy.Rate(rospy.get_param('/node_rate/value', default = 30))
    
    joystick = Joystick()
    rospy.on_shutdown(joystick.stop)

    print("The Manual Control w/Joystick Node is Running")
    try:    
        while not rospy.is_shutdown():
            joystick.manual_control()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

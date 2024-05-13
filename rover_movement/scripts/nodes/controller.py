#!/usr/bin/env python3

# Python libraries
import rospy

# Classes
from classes_movement import Controller

if __name__ == "__main__":
    rospy.init_node('Rover_Controller')
    rate = rospy.Rate(rospy.get_param('/node_rate/value', default = 10))

    rover = Controller()
    rospy.on_shutdown(rover.stop)

    print("The Rover Controller is Running")
    try:    
        while not rospy.is_shutdown():
            rover.control()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python3

# Python libraries
import rospy

# Classes
from classes_movement import Rover_Navigation

if __name__ == "__main__":
    rospy.init_node('Rover_Navigation')
    rate = rospy.Rate(rospy.get_param('/node_rate/value', default = 10))

    rover = Rover_Navigation()
    rospy.on_shutdown(rover.stop)

    print("The Rover Navigation is Running")
    try:    
        while not rospy.is_shutdown():
            rover.move()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
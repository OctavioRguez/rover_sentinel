#!/usr/bin/env python3

# Import python libraries
import rospy

# Import Classes
from classes_movement import Controller

if __name__ == "__main__":
    # Initialise and Setup node
    rospy.init_node('Rover_Controller')

    # Configure the Node
    rate = rospy.Rate(rospy.get_param('/node_rate/value', default = 10))

    # Classes
    rover = Controller()

    # Shutdown hook
    rospy.on_shutdown(rover.stop)

    print("The Rover Controller is Running")
    try:    
        while not rospy.is_shutdown():
            rover.control()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
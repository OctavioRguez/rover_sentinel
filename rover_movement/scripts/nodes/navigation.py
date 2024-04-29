#!/usr/bin/env python3

# Import python libraries
import rospy

# Import Classes
from classes_movement import Rover_Navigation

if __name__ == "__main__":
    # Initialise and Setup node
    rospy.init_node('Rover_Navigation')

    # Configure the Node
    rate = rospy.Rate(rospy.get_param('/node_rate/value', default = 10))

    # Classes
    rover = Rover_Navigation()

    # Shutdown hook
    rospy.on_shutdown(rover.stop)

    print("The Rover Navigation is Running")
    try:    
        while not rospy.is_shutdown():
            rover.move()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python3

# Import python libraries
import rospy

# Import Classes
from classes_vision import Camera

if __name__=='__main__':
    # Initialise and Setup node
    rospy.init_node("Camera_Node")

    # Configure the Node
    rate = rospy.Rate(rospy.get_param('/node_rate/value', default = 10))
    
    # Classes
    cam = Camera()

    # Shutdown hook
    rospy.on_shutdown(cam.stop)

    print("The Camera Node is Running")
    try:    
        while not rospy.is_shutdown():
            cam.get_image()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

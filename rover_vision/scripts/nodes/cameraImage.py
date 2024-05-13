#!/usr/bin/env python3

# Python libraries
import rospy

# Classes
from classes_vision import Camera

if __name__=='__main__':
    rospy.init_node("Camera_Node")
    rate = rospy.Rate(rospy.get_param('/node_rate/value', default = 10))
    
    cam = Camera()
    rospy.on_shutdown(cam.stop)

    print("The Camera Node is Running")
    try:    
        while not rospy.is_shutdown():
            cam.get_image()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python3

# Python libraries
import rospy

# Classes
from classes_sensors import Arduino

if __name__=='__main__':
    rospy.init_node("Arduino_Master")
    rate = rospy.Rate(rospy.get_param('/node_rate/value', default = 30))
    
    master = Arduino()
    rospy.on_shutdown(master.stop)

    print("The Arduino Master Node is Running")
    try:    
        while not rospy.is_shutdown():
            master.receive_sensors_data()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

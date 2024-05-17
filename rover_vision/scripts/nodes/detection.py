#!/usr/bin/env python3

# Python libraries
import rospy

# Classes
from classes_vision import modelPredict

if __name__=='__main__':
    rospy.init_node("Classification_Node")
    rate = rospy.Rate(rospy.get_param('/node_rate/value', default = 30))
    
    model = modelPredict()
    rospy.on_shutdown(model.stop)

    print("The Classification Node is Running")
    try:    
        while not rospy.is_shutdown():
            model.predict()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

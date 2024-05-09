#!/usr/bin/env python3

# Import python libraries
import rospy

# Import Classes
from classes_vision import modelPredict

if __name__=='__main__':
    # Initialise and Setup node
    rospy.init_node("Classification_Node")

    # Configure the Node
    rate = rospy.Rate(rospy.get_param('/node_rate/value', default = 10))
    
    # Classes
    model = modelPredict()

    # Shutdown hook
    rospy.on_shutdown(model.stop)

    print("The Prediction Node is Running")
    try:    
        while not rospy.is_shutdown():
            model.start()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

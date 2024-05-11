#!/usr/bin/env python3

# Import python libraries
import rospy

# Import Classes
from classes_slam import Localization

if __name__ == "__main__":
    # Initialise and Setup node
    rospy.init_node('Slam_Position')

    # Configure the Node
    rate = rospy.Rate(rospy.get_param('/node_rate/value', default = 10))

    # Classes
    handler = Localization()

    # Shutdown hook
    rospy.on_shutdown(handler.stop)

    print("The SLAM position publisher is Running")
    try:
        while not rospy.is_shutdown():
            handler.update_odom()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python
import rospy
from Classes import obstacleAvoidance

if __name__ == "__main__":
    # Initialise and Setup node
    rospy.init_node('Puzzlebot_Navigation_obstacleAvoidance')

    # Configure the Node
    rate = rospy.Rate(rospy.get_param('/node_rate', default = 10))

    # Classes
    navigation_handler = obstacleAvoidance()

    # Shutdown hook
    rospy.on_shutdown(navigation_handler._stop)

    print("The Puzzlebot Navigation and Obstacle Avoidance is Running")
    try:    
        while not rospy.is_shutdown():
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
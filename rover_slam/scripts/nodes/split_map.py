#!/usr/bin/env python3

# Import python libraries
import rospy

# Import Classes
from classes_slam import Map_Sections

if __name__ == "__main__":
    # Initialise and Setup node
    rospy.init_node('Split_Map')

    # Configure the Node
    rate = rospy.Rate(rospy.get_param('/node_rate/value', default = 10))

    # Classes
    sections = Map_Sections()

    # Shutdown hook
    rospy.on_shutdown(sections.stop)

    print("The Map split is Running")
    try:
        sections.split(2, 2)
    except rospy.ROSInterruptException:
        pass
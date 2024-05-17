#!/usr/bin/env python3

# Python libraries
import rospy

# Classes
from classes_slam import Map_Sections

if __name__ == "__main__":
    rospy.init_node('Split_Map')
    rate = rospy.Rate(rospy.get_param('/node_rate/value', default = 30))

    sections = Map_Sections()
    rospy.on_shutdown(sections.stop)

    print("The Map splitter is Running")
    try:
        sections.split(2, 2)
    except rospy.ROSInterruptException:
        pass
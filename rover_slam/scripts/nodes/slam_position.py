#!/usr/bin/env python3

# Python libraries
import rospy

# Classes
from classes_slam import Localization_SLAM

if __name__ == "__main__":
    rospy.init_node('Slam_Position')
    rate = rospy.Rate(rospy.get_param('/node_rate/value', default = 30))

    pose_finder = Localization_SLAM()
    rospy.on_shutdown(pose_finder.stop)

    print("The SLAM position publisher is Running")
    try:
        while not rospy.is_shutdown():
            pose_finder.update_pose()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
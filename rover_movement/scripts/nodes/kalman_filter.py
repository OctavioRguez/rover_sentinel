#!/usr/bin/env python3

# Python libraries
import rospy

# Classes
from classes_movement import Kalman_Filter

if __name__ == "__main__":
    rospy.init_node('Rover_Kalman')
    rate = rospy.Rate(rospy.get_param('/node_rate/value', default = 30))

    filter = Kalman_Filter()
    rospy.on_shutdown(filter.stop)

    print("The Rover Kalman Localization is Running")
    try:    
        while not rospy.is_shutdown():
            if not filter._last_time:
                filter._last_time = rospy.Time.now().to_sec()
            else:
                filter.apply_filter()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

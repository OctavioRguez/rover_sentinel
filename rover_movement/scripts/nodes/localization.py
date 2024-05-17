#!/usr/bin/env python3

# Python libraries
import rospy

# Classes
from classes_movement import Localization, Transform

if __name__ == "__main__":
    rospy.init_node('Rover_Localization')
    rate = rospy.Rate(rospy.get_param('/node_rate/value', default = 30))

    rover = Localization()
    tf = Transform()
    rospy.on_shutdown(rover.stop)

    print("The Rover Localization is Running")
    try:    
        while not rospy.is_shutdown():
            if not rover._last_time:
                rover._last_time = rospy.Time.now().to_sec()
            else:
                rover.update_odometry()
                tf.update_transform()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

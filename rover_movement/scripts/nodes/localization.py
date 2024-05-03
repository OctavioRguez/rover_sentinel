#!/usr/bin/env python3

# Import python libraries
import rospy

# Import Classes
from classes_movement import Localization, Transform

if __name__ == "__main__":
    # Initialise and Setup node
    rospy.init_node('Rover_Localization')

    # Configure the Node
    rate = rospy.Rate(rospy.get_param('/node_rate/value', default = 10))

    # Classes
    rover = Localization()
    tf = Transform()

    # Shutdown hook
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
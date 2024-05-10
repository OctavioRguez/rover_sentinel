#!/usr/bin/env python3

# Import python libraries
import rospy

# Import Classes
from classes_slam import RRT
from classes_slam import Dijkstra_Path

if __name__ == "__main__":
    # Initialise and Setup node
    rospy.init_node('RRT_Planning')

    # Configure the Node
    rate = rospy.Rate(rospy.get_param("/node_rate/value", default = 10))

    # Classes
    rrt = RRT()
    finder = Dijkstra_Path()

    # Shutdown hook
    rospy.on_shutdown(rrt.stop)

    print("The RRT Planning is Running")
    try:
        nodes = rrt.rrt()
        path = finder.dijkstra(nodes, (150, 75))
        rrt.plot(nodes, path)
    except rospy.ROSInterruptException:
        pass

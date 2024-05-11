#!/usr/bin/env python3

# Import python libraries
import rospy

# Import Classes
from classes_slam import PRM
from classes_slam import Dijkstra_Path

if __name__ == "__main__":
    # Initialise and Setup node
    rospy.init_node('PRM_Dijkstra_Planning')

    # Configure the Node
    rate = rospy.Rate(rospy.get_param("/node_rate/value", default = 10))

    # Classes
    prm = PRM()

    # Shutdown hook
    rospy.on_shutdown(prm.stop)

    print("The PRM_Dijkstra Planning is Running")
    try:
        graph = prm.calculate_prm()
        dijkstra = Dijkstra_Path(graph)
        path = dijkstra.calculate_dijkstra()
        prm.plot(path)
    except rospy.ROSInterruptException:
        pass

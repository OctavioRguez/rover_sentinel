#!/usr/bin/env python3

# Python libraries
import rospy

# Classes
from classes_slam import PRM, Dijkstra_Path

if __name__ == "__main__":
    rospy.init_node('PRM_Dijkstra_Planning')
    rate = rospy.Rate(rospy.get_param("/node_rate/value", default = 30))

    prm = PRM()
    rospy.on_shutdown(prm.stop)

    print("The PRM_Dijkstra Planning is Running")
    try:
        graph = prm.calculate_prm()
        dijkstra = Dijkstra_Path(graph, (800, 800))
        path = dijkstra.calculate_dijkstra(None, None)
        prm.plot(path)
    except rospy.ROSInterruptException:
        pass

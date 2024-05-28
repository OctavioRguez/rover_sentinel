#!/usr/bin/env python3

# Python libraries
import rospy
import numpy as np

# ROS messages
from std_msgs.msg import Bool, Int8, String
from geometry_msgs.msg import Twist
from rover_slam.msg import Quadrants, Border

# Rover packages
from classes_slam import Map_Sections, PRM, Dijkstra_Path

class StateMachine:
    def __init__(self) -> None:
        self.__state = "EXPLORATION"
        self.__map = False
        self.__control_ready = False
        self.__sound = False
        self.__person = False
        self.__manual_mode = False

        self.__curr_quadrant = None
        self.__quadrants = Quadrants()
        self.__visited_quadrants = []
        self.__last_quadrant_time = 0

        self.__cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
        self.__buzzer_pub = rospy.Publisher("/buzzer", Int8, queue_size = 10)
        self.__border_pub = rospy.Publisher("/curr_border", Border, queue_size = 10)
        self.__control_pub = rospy.Publisher("/enable/control", Bool, queue_size = 10)
        self.__nav_pub = rospy.Publisher("/enable/navegation", Bool, queue_size = 10)
        self.__kalman_pub = rospy.Publisher("/filter_mode", String, queue_size = 10)

        rospy.Subscriber("/borders", Quadrants, self.__borders_callback)
        rospy.Subscriber("/detected/sound", Bool, self.__sound_callback)
        rospy.Subscriber("/detected/person", Bool, self.__person_callback)
        rospy.Subscriber("/ready/map", Bool, self.__ready_map_callback)
        rospy.Subscriber("/ready/control", Bool, self.__ready_control_callback)
        rospy.Subscriber("/manual_mode", Bool, self.__manual_callback)
        rospy.wait_for_message("/start", Bool, timeout = 360)

    def __borders_callback(self, msg:Quadrants) -> None:
        self.__quadrants = msg

    def __manual_callback(self, msg:Bool) -> None:
        self.__manual_mode = msg.data

    def __sound_callback(self, msg:Bool) -> None:
        self.__sound = msg.data

    def __person_callback(self, msg:Bool) -> None:
        self.__person = msg.data

    def __ready_map_callback(self, msg:Bool) -> None:
        self.__map = msg.data
    
    def __ready_control_callback(self, msg:Bool) -> None:
        self.__control_ready = msg.data

    def run(self):
        if self.__state == "EXPLORATION":
            if self.__map:
                self.__cmd_vel.publish(Twist())
                self.__move_to_control()
                rospy.loginfo("State: CONTROL - Moving to a quadrant...")

        elif self.__state == "CONTROL":
            flag = False
            if self.__manual_mode:
                self.__cmd_vel.publish(Twist())
                self.__state = "MANUAL"
                self.__kalman_pub.publish("Manual")
                rospy.loginfo("State: MANUAL - Use the joystick for moving the rover...")
            elif self.__control_ready:
                self.__border_pub.publish(self.__curr_quadrant)
                self.__state = "NAVIGATION"
                self.__kalman_pub.publish("Navigation")
                self.__last_quadrant_time = rospy.Time.now().to_sec()
                rospy.loginfo("State: NAVIGATION - Navigating...")
            elif self.__person:
                self.__state = "ALERT2"
                self.__kalman_pub.publish("")
                rospy.loginfo("State: ALERT2 - Person detected")
            elif self.__sound:
                self.__buzzer_pub.publish(1)
                self.__state = "ALERT1"
                self.__kalman_pub.publish("")
                rospy.loginfo("State: ALERT1 - Sound detected, investigating...")
            else:
                flag = True
            self.__control_pub.publish(flag)

        elif self.__state == "NAVIGATION":
            flag = False
            if self.__manual_mode:
                self.__cmd_vel.publish(Twist())
                self.__state = "MANUAL"
                self.__kalman_pub.publish("Manual")
                rospy.loginfo("State: MANUAL - Use the joystick for moving the rover...")
            elif self.__person:
                self.__state = "ALERT2"
                self.__kalman_pub.publish("")
                rospy.loginfo("State: ALERT2 - Person detected")
            elif self.__sound:
                self.__buzzer_pub.publish(1)
                self.__state = "ALERT1"
                self.__kalman_pub.publish("")
                rospy.loginfo("State: ALERT1 - Sound detected, investigating...")
            else:
                flag = True
            self.__nav_pub.publish(flag)
            self.__check_time()

        elif self.__state == "MANUAL":
            if not self.__manual_mode:
                self.__state = "NAVIGATION"
                self.__kalman_pub.publish("Navigation")
                self.__last_quadrant_time = rospy.Time.now().to_sec()
                rospy.loginfo("State: NAVIGATION - Navigating...")

        elif self.__state == "ALERT1":
            self.__alert1()
            if self.__person:
                self.__state = "ALERT2"
                self.__kalman_pub.publish("")
                rospy.loginfo("State: ALERT2 - Person detected")
        
        elif self.__state == "ALERT2":
            self.__alert2()

    def __move_to_control(self) -> None:
        shape = Map_Sections().split(2, 2)
        rospy.loginfo("Map splited into quadrants, calculating PRM...")
        graph = PRM().calculate_prm()
        rospy.loginfo("PRM calculated, creating a path to a quadrant...")
        self.__planner = Dijkstra_Path(graph, shape)
        self.__planning()

    def __alert1(self) -> None:
        self.__buzzer_pub.publish(1)

    def __alert2(self) -> None:
        self.__cmd_vel.publish(Twist())
        self.__buzzer_pub.publish(1)

    def __select_quadrant(self) -> None:
        num_of_quadrants = len(self.__quadrants.borders)
        if num_of_quadrants == len(self.__visited_quadrants):
            self.__visited_quadrants = []
        while True:
            self.__curr_quadrant = self.__quadrants.borders[np.random.randint(0, num_of_quadrants)]
            if not self.__curr_quadrant in self.__visited_quadrants:
                self.__visited_quadrants.append(self.__curr_quadrant)
                break

    def __check_time(self) -> None:
        current_time = rospy.Time.now().to_sec()
        elapsed_time = current_time - self.__last_quadrant_time
        if elapsed_time >= 30:
            rospy.loginfo("Time elapsed, selecting another quadrant...")
            self.__planning()

    def __planning(self) -> None:
        self.__cmd_vel.publish(Twist())
        while True:
            self.__select_quadrant()
            path = self.__planner.calculate_dijkstra(self.__curr_quadrant, self.__quadrants.offset)
            if (self.__curr_quadrant.upper.x <= path[-1][0] <= self.__curr_quadrant.lower.x 
                and self.__curr_quadrant.upper.y <= path[-1][1] <= self.__curr_quadrant.lower.y):
                print(path)
                break
            rospy.loginfo("Unable to create a path to the current quadrant, selecting another quadrant...")
            rospy.sleep(1)
        self.__state = "CONTROL"
        self.__kalman_pub.publish("Control")

    def stop(self) -> None:
        self.__cmd_vel.publish(Twist())
        rospy.loginfo("Stoping the State Machine Node")
        rospy.signal_shutdown("Stoping the State Machine Node")

if __name__ == "__main__":
    rospy.init_node("StateMachine")
    rate = rospy.Rate(rospy.get_param('/node_rate/value', default = 30))

    state_machine = StateMachine()
    rospy.on_shutdown(state_machine.stop)

    print("The State Machine is Running")
    rospy.loginfo("Estado: EXPLORACION - Construyendo el mapa...")
    try:    
        while not rospy.is_shutdown():
            state_machine.run()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

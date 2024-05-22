#!/usr/bin/env python3

# Python libraries
import rospy
import random
import time

# ROS messages
from std_msgs.msg import Bool, Int8
from rover_slam.msg import Quadrants, Border

# Rover packages
from classes_movement import Rover_Navigation, Controller, Kalman_Filter
from classes_slam import Map_Sections, PRM, Dijkstra_Path
from classes_sensors import Joystick

class StateMachine:
    def __init__(self) -> None:
        self.__state = "EXPLORATION"
        self.__map = False
        self.__control_ready = False
        self.__sound = False
        self.__person = False
        self.__manual_mode = False

        self.__curr_quadrant = None
        self.__quadrants = []
        self.__visied_quadrants = []
        self.__last_quadrant_time = 0

        self.__nav = Rover_Navigation()
        self.__control_class = Controller()
        self.__joystick = Joystick()

        self.__filter = Kalman_Filter()
        self.__mode = "Nav"

        self.__buzzer_pub = rospy.Publisher("/buzzer", Int8, queue_size = 10)
        self.__border_pub = rospy.Publisher("/curr_border", Border, queue_size = 10)

        rospy.Subscriber("/borders", Quadrants, self.__borders_callback)
        rospy.Subscriber("/detected/sound", Bool, self.__sound_callback)
        rospy.Subscriber("/detected/person", Bool, self.__person_callback)
        rospy.Subscriber("/ready/map", Bool, self.__ready_map_callback)
        rospy.Subscriber("/ready/control", Bool, self.__ready_control_callback)
        rospy.Subscriber("/manual_mode", Bool, self.__manual_callback)
        rospy.wait_for_message("/start", Bool, timeout = 360)

    def __borders_callback(self, msg:Quadrants) -> None:
        self.__quadrants = msg.borders

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
        self.__filter.predict(self.__mode)
        if self.__state == "EXPLORATION":
            if self.__map:
                self.__nav.stop()
                self.__move_to_control()
                self.__state = "CONTROL"
                self.__mode = "Control"
                rospy.loginfo("State: CONTROL - Moving to a quadrant...")
            self.__exploration()

        elif self.__state == "CONTROL":
            if self.__manual_mode:
                self.__nav.stop()
                self.__state = "MANUAL"
                self.__mode = "Manual"
                rospy.loginfo("State: MANUAL - Use the joystick for moving the rover...")
            elif self.__control_ready:
                self.__border_pub.publish(self.__curr_quadrant)
                self.__state = "NAVIGATION"
                self.__mode = "Nav"
                rospy.loginfo("State: NAVIGATION - Navigating...")
            elif self.__person:
                self.__state = "ALERT2"
                self.__mode = ""
                rospy.loginfo("State: ALERT2 - Person detected")
            elif self.__sound:
                self.__buzzer_pub.publish(1)
                self.__state = "ALERT1"
                self.__mode = ""
                rospy.loginfo("State: ALERT1 - Sound detected, investigating...")
            self.__control()

        elif self.__state == "NAVIGATION":
            if self.__manual_mode:
                self.__nav.stop()
                self.__state = "MANUAL"
                self.__mode = "Manual"
                rospy.loginfo("State: MANUAL - Use the joystick for moving the rover...")
            elif self.__person:
                self.__state = "ALERT2"
                self.__mode = ""
                rospy.loginfo("State: ALERT2 - Person detected")
            elif self.__sound:
                self.__buzzer_pub.publish(1)
                self.__state = "ALERT1"
                self.__mode = ""
                rospy.loginfo("State: ALERT1 - Sound detected, investigating...")
            self.__navigation()

        elif self.__state == "MANUAL":
            if not self.__manual_mode:
                self.__state = "NAVIGATION"
                self.__mode = "Nav"
                rospy.loginfo("State: NAVIGATION - Navigating...")
            self.__manual()

        elif self.__state == "ALERT1":
            if self.__person:
                self.__state = "ALERT2"
                self.__mode = ""
                rospy.loginfo("State: ALERT2 - Person detected")
            self.__alert1()
        
        elif self.__state == "ALERT2":
            self.__alert2()

    def __exploration(self) -> None:
        self.__nav.move()

    def __move_to_control(self) -> None:
        Map_Sections().split(2, 2)
        graph, shape = PRM().calculate_prm()
        self.__planner = Dijkstra_Path(graph, shape)
        self.__select_quadrant()
        self.__planner.calculate_dijkstra(self.__curr_quadrant)
    
    def __control(self) -> None:
        self.__control_class.control()

    def __navigation(self) -> None:
        self.__nav.move()
        self.__check_time()

    def __manual(self) -> None:
        self.__joystick.manual_control()

    def __alert1(self) -> None:
        self.__control_class.rotate()

    def __alert2(self) -> None:
        self.__nav.stop()
        self.__buzzer_pub.publish(1)

    def __select_quadrant(self) -> None:
        num_of_quadrants = len(self.__quadrants)
        while True:
            self.__curr_quadrant = self.__quadrants[random.randint(0, num_of_quadrants - 1)]
            if not self.__curr_quadrant in self.__visied_quadrants:
                self.__visied_quadrants.append(self.__curr_quadrant)
                self.__last_quadrant_time = time.time()

    def __check_time(self) -> None:
        current_time = time.time()
        elapsed_time = current_time - self.__last_quadrant_time
        if elapsed_time >= 30:
            if len(self.__quadrants) == len(self.__visied_quadrants):
                self.__visied_quadrants = []
            self.__select_quadrant()
            self.__planner.calculate_dijkstra(self.__curr_quadrant)
            self.__state = "CONTROL"
            self.__mode = "Control"
            self.__nav.stop()

    def stop(self) -> None:
        self.__nav.stop()
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

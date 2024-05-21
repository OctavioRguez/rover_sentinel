#!/usr/bin/env python3

# Python libraries
import rospy

# ROS messages
from std_msgs.msg import Bool, Int8

# Rover packages
from classes_movement import Rover_Navigation, Controller
from classes_slam import Map_Sections, PRM, Dijkstra_Path
from classes_sensors import Joystick

class StateMachine:
    def __init__(self) -> None:
        self.__state = "EXPLORATION"
        self.__map = False
        self.__sound = False
        self.__person = False
        self.__manual_mode = False

        self.__nav = Rover_Navigation()
        self.__control = Controller()
        self.__joystick = Joystick()

        self.__buzzer_pub = rospy.Publisher("/buzzer", Int8, queue_size = 10)

        rospy.Subscriber("/detected/sound", Bool, self.__sound_callback)
        rospy.Subscriber("/detected/person", Bool, self.__person_callback)
        rospy.Subscriber("/map_ready", Bool, self.__map_ready_callback)
        rospy.Subscriber("/manual_mode", Bool, self.__manual_callback)

    def __manual_callback(self, msg:bool) -> None:
        self.__manual_mode = msg.data

    def __sound_callback(self, msg:bool) -> None:
        self.__sound = msg.data

    def __person_callback(self, msg:bool) -> None:
        self.__person = msg.data

    def __map_ready_callback(self,msg:bool) -> None:
        self.__map = msg.data

    def run(self):
        if self.__state == "EXPLORATION":
            if self.__map:
                self.__nav.stop()
                self.__move_to_navegation()
                self.__state = "NAVIGATION"
                rospy.loginfo("State: NAVIGATION - Navigating...")
            self.__exploration()

        elif self.__state == "NAVIGATION":
            if self.__manual_mode:
                self.__nav.stop()
                self.__state = "MANUAL"
                rospy.loginfo("State: MANUAL - Use the joystick for moving the rover...")
            elif self.__person:
                self.__state = "ALERT2"
                rospy.loginfo("State: ALERT2 - Person detected")
            elif self.__sound:
                self.__buzzer_pub.publish(1)
                self.__state = "ALERT1"
                rospy.loginfo("State: ALERT1 - Sound detected, investigating...")
            self.__navigation()

        elif self.__state == "MANUAL":
            if not self.__manual_mode:
                self.__state = "NAVIGATION"
                rospy.loginfo("State: NAVIGATION - Navigating...")
            self.__manual()

        elif self.__state == "ALERT1":
            if self.__person:
                self.__state = "ALERT2"
                rospy.loginfo("State: ALERT2 - Person detected")
            self.__alert1()
        
        elif self.__state == "ALERT2":
            self.__alert2()

    def __exploration(self) -> None:
        self.__nav.move()

    def __move_to_navegation(self) -> None:
        Map_Sections().split(2, 2)
        graph, shape = PRM().calculate_prm()
        self.__planner = Dijkstra_Path(graph, shape)

    def __navigation(self) -> None:
        self.__nav.compute_lasers()
        self.__nav.move()
        # self.__planner.calculate_dijkstra()
        # self.__control.control()

    def __manual(self) -> None:
        self.__joystick.manual_control()

    def __alert1(self) -> None:
        self.__control.rotate()

    def __alert2(self) -> None:
        self.__nav.stop()
        self.__buzzer_pub.publish(1)

    def stop(self) -> None:
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

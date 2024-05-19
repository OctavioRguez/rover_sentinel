#!/usr/bin/env python3

# Python libraries
import rospy

# ROS messages
from std_msgs.msg import Bool

class StateMachine:
    def __init__(self) -> None:
        self.__state = "EXPLORACION"
        self.__map = False
        self.__sound = False
        self.__person = False
        self.__manual = False

        rospy.Subscriber("/detected/sound", Bool, self.__sound_callback)
        rospy.Subscriber("/detected/person", Bool, self.__person_callback)
        rospy.Subscriber("/map_ready", Bool, self.__map_ready_callback)
        rospy.Subscriber("/manual_mode", Bool, self.__manual_callback)

    def __manual_callback(self, msg:bool) -> None:
        self.__manual = msg.data

    def __sound_callback(self, msg:bool) -> None:
        self.__sound = msg.data

    def __person_callback(self, msg:bool) -> None:
        self.__person = msg.data

    def __map_ready_callback(self,msg:bool) -> None:
        self.__map = msg.data
        
    def run(self):
        if self.__state == "EXPLORACION":
            if self.__map:
                self.__state = "NAVEGACION"
            self.exploracion()

        elif self.__state == "NAVEGACION":
            if self.__manual:
                self.__state = "MANUAL"
            elif self.__person:
                self.__state = "ALERTA2"
            elif self.__sound:
                self.__state = "ALERTA1"
            self.navegacion()

        elif self.__state == "MANUAL":
            if not self.__manual:
                self.__state = "NAVEGACION"
            self.manual()

        elif self.__state == "ALERTA1":
            if self.__person:
                self.__state = "ALERTA2"
            self.alerta1()
        
        elif self.__state == "ALERTA2":
            self.alerta2()

    def exploracion(self) -> None:
        rospy.loginfo("Estado: EXPLORACION - Construyendo el mapa...")

    def navegacion(self) -> None:
        rospy.loginfo("Estado: NAVEGACION - Navegando...")

    def manual(self) -> None:
        rospy.loginfo("Estado: MANUAL - Manual...")

    def alerta1(self) -> None:
        rospy.loginfo("Estado: ALERTA1 - Sonido detectado, investigando...")

    def alerta2(self) -> None:
        rospy.loginfo("Estado: ALERTA2 - Persona detectada")
    
    def stop(self) -> None:
        rospy.loginfo("Stoping the State Machine Node")
        rospy.signal_shutdown("Stoping the State Machine Node")

if __name__ == "__main__":
    rospy.init_node("StateMachine")
    rate = rospy.Rate(rospy.get_param('/node_rate/value', default = 30))

    state_machine = StateMachine()
    rospy.on_shutdown(state_machine.stop)

    print("The State Machine is Running")
    try:    
        while not rospy.is_shutdown():
            state_machine.run()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

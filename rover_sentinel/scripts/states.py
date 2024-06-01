#!/usr/bin/env python3

# Python libraries
import rospy
import numpy as np
import pytz
import datetime
import subprocess, signal
import smtplib
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from email.mime.image import MIMEImage

# ROS messages
from std_msgs.msg import Bool, Int8, String
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray
from rover_slam.msg import Quadrants, Border

# Rover packages
from classes_slam import Map_Sections, PRM, Dijkstra_Path

class StateMachine:
    def __init__(self) -> None:
        self.__state = "EXPLORATION"
        self.__map = False
        self.__control_ready = False
        self.__sound = False
        self.__active_investigation = False
        self.__person = False
        self.__manual_mode = False

        self.__path = Path()
        self.__path.header.frame_id = "odom"

        self.__pose = (0, 0)
        self.__nav_time = 15
        self.__curr_quadrant = None
        self.__quadrants = Quadrants()
        self.__visited_quadrants = []
        self.__last_quadrant_time = 0

        # Border markers for RVIZ
        self.__markers = MarkerArray()
        self.__types = ["Current", "Visited", "Unvisited"]

        # Start SLAM
        self.__process = subprocess.Popen(["roslaunch", "rover_sentinel", "exploration.launch"], 
                                          stdout = subprocess.PIPE, stderr = subprocess.PIPE)

        # Email parameters
        self.__send_to = ["A01639786@tec.mx"]
        self.__smtp_server = "smtp-mail.outlook.com"
        self.__smtp_port = 587
        self.__email_msg = MIMEMultipart()
        self.__email_msg['From'] = "puzzlebot2@hotmail.com"
        self.__email_msg['To'] = ', '.join(self.__send_to)
        self.__email_msg['Subject'] = "Intruder Alert"

        self.__cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
        self.__path_pub = rospy.Publisher("/path", Path, queue_size = 10)
        self.__buzzer_pub = rospy.Publisher("/buzzer", Int8, queue_size = 10)
        self.__border_pub = rospy.Publisher("/curr_border", Border, queue_size = 10)
        self.__control_pub = rospy.Publisher("/enable/control", Bool, queue_size = 10)
        self.__nav_pub = rospy.Publisher("/enable/navegation", Bool, queue_size = 10)
        self.__kalman_pub = rospy.Publisher("/filter_mode", String, queue_size = 10)
        self.__markers_pub = rospy.Publisher("/visualization/borders", MarkerArray, queue_size = 10)

        rospy.Subscriber("/odom/kalman", Odometry, self.__odom_callback)
        rospy.Subscriber("/borders", Quadrants, self.__borders_callback)
        rospy.Subscriber("/detected/sound", Bool, self.__sound_callback)
        rospy.Subscriber("/detected/person", Bool, self.__person_callback)
        rospy.Subscriber("/ready/map", Bool, self.__ready_map_callback)
        rospy.Subscriber("/ready/control", Bool, self.__ready_control_callback)
        rospy.Subscriber("/manual_mode", Bool, self.__manual_callback)

    def __odom_callback(self, msg:Odometry) -> None:
        self.__pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def __borders_callback(self, msg:Quadrants) -> None:
        self.__quadrants = msg

    def __manual_callback(self, msg:Bool) -> None:
        self.__manual_mode = msg.data

    def __sound_callback(self, msg:Bool) -> None:
        # self.__sound = msg.data
        pass

    def __person_callback(self, msg:Bool) -> None:
        # self.__person = msg.data
        pass

    def __ready_map_callback(self, msg:Bool) -> None:
        self.__map = msg.data
    
    def __ready_control_callback(self, msg:Bool) -> None:
        self.__control_ready = msg.data

    def run(self):
        if self.__state == "EXPLORATION":
            if self.__map:
                self.__cmd_vel.publish(Twist(linear=Point(x=0.0), angular=Point(z=0.0)))
                self.__initiate_operation()

        elif self.__state == "CONTROL":
            flag = False
            if self.__manual_mode:
                self.__move_to_manual()
            elif self.__control_ready:
                self.__move_to_nav()
            elif self.__person:
                self.__move_to_alert()
            elif self.__sound and not self.__active_investigation:
                self.__start_investigation()
            else:
                flag = True
            self.__control_pub.publish(flag)

        elif self.__state == "NAVIGATION":
            flag = False
            if self.__manual_mode:
                self.__move_to_manual()
            elif self.__person:
                self.__move_to_alert()
            elif self.__sound and not self.__active_investigation:
                self.__start_investigation()
            else:
                flag = True
            self.__nav_pub.publish(flag)
            self.__check_time()

        elif self.__state == "MANUAL":
            if not self.__manual_mode:
                if self.__control_ready:
                    self.__approximate_new_quadrant()
                else:
                    self.__state = "CONTROL"
                    self.__kalman_pub.publish("Control")
                    rospy.loginfo("State: CONTROL - Moving to a quadrant...")
        
        elif self.__state == "ALERT":
            self.__cmd_vel.publish(Twist(linear=Point(x=0.0), angular=Point(z=0.0)))
            # self.__buzzer_pub.publish(1)
            print("Buzzer")

    def __move_to_manual(self) -> None:
        self.__cmd_vel.publish(Twist(linear=Point(x=0.0), angular=Point(z=0.0)))
        self.__kalman_pub.publish("Manual")
        self.__state = "MANUAL"
        rospy.loginfo("State: MANUAL - Use the joystick for moving the rover...")
    
    def __move_to_nav(self) -> None:
        self.__border_pub.publish(self.__curr_quadrant)
        self.__kalman_pub.publish("Navigation")
        self.__last_quadrant_time = rospy.Time.now().to_sec()
        self.__state = "NAVIGATION"
        rospy.loginfo("State: NAVIGATION - Navigating...")
    
    def __move_to_alert(self) -> None:
        self.__kalman_pub.publish("")
        self.__state = "ALERT"
        rospy.loginfo("State: ALERT - Person detected")
        self.__send_email()

    def __start_investigation(self) -> None:
        self.__active_investigation = True
        self.__buzzer_pub.publish(1)
        rospy.loginfo("Sound detected, investigating...")
        self.__nav_time = 15
        self.__visited_quadrants = []
        self.__planning()

    def __approximate_new_quadrant(self) -> None:
        for quad in self.__quadrants.borders:
            if (quad.upper.x <= self.__pose[0] <= quad.lower.x) and (quad.upper.y <= self.__pose[1] <= quad.lower.y):
                break
        if quad in self.__visited_quadrants:
            rospy.loginfo("Current quadrant already visited, selecting a new one...")
            self.__planning()
        else:
            rospy.loginfo("Restarting navigation in the current quadrant...")
            self.__curr_quadrant = quad
            self.__move_to_nav()

    def __select_quadrant(self) -> None:
        num_of_quadrants = len(self.__quadrants.borders)
        if num_of_quadrants == len(self.__visited_quadrants):
            if self.__active_investigation:
                rospy.loginfo("All quadrants have been investigated. Returning to normal operation...")
                self.__active_investigation = False
                self.__sound = False
                self.__nav_time = 15
            self.__visited_quadrants = [self.__curr_quadrant]
        while True:
            self.__curr_quadrant = self.__quadrants.borders[np.random.randint(0, num_of_quadrants)]
            if not self.__curr_quadrant in self.__visited_quadrants:
                break
        self.__print_borders()

    def __planning(self) -> None:
        while True:
            self.__select_quadrant()
            path = self.__planner.calculate_dijkstra(self.__curr_quadrant, self.__quadrants.offset)
            if (self.__curr_quadrant.upper.x <= path[-1][0] <= self.__curr_quadrant.lower.x 
                and self.__curr_quadrant.upper.y <= path[-1][1] <= self.__curr_quadrant.lower.y):
                self.__path.header.stamp = rospy.Time.now()
                self.__path.poses = [PoseStamped(pose=Pose(position=Point(x=pos[0], y=pos[1]))) for pos in path]
                self.__path_pub.publish(self.__path)
                rospy.sleep(1)
                break
            rospy.loginfo("Unable to create a path to the current quadrant, selecting another quadrant...")
        self.__kalman_pub.publish("Control")
        self.__state = "CONTROL"
        rospy.loginfo("State: CONTROL - Moving to a quadrant...")

    def __check_time(self) -> None:
        current_time = rospy.Time.now().to_sec()
        elapsed_time = current_time - self.__last_quadrant_time
        if elapsed_time >= self.__nav_time:
            rospy.loginfo("Time elapsed, selecting another quadrant...")
            self.__nav_pub.publish(False)
            self.__kalman_pub.publish("")
            time = rospy.Time.now().to_sec()
            while rospy.Time.now().to_sec() - time < 0.1:
                self.__cmd_vel.publish(Twist(linear=Point(x=0.0), angular=Point(z=0.0)))
                rospy.sleep(0.1)
            self.__visited_quadrants.append(self.__curr_quadrant)
            self.__planning()

    def __manage_roslaunchs(self) -> None:
        self.__process.send_signal(signal.SIGINT)
        self.__process.wait()
        self.__process = subprocess.Popen(["roslaunch", "rover_sentinel", "patrol.launch"], 
                            stdout = subprocess.PIPE, stderr = subprocess.PIPE)
        rospy.wait_for_message("/prediction/compressed", CompressedImage, timeout = 30)

    def __initiate_operation(self) -> None:
        shape = Map_Sections().split(2, 2)
        self.__print_borders()
        rospy.loginfo("Map splited into quadrants, calculating PRM...")
        graph = PRM().calculate_prm()
        rospy.loginfo("PRM calculated, creating a path to a quadrant...")
        self.__manage_roslaunchs()
        self.__planner = Dijkstra_Path(graph, shape)
        self.__planning()

    def __send_email(self) -> None:
        date = datetime.datetime.now(pytz.timezone('America/Mexico_City'))
        body = "ALERT: Person detected at " + date.strftime("Day: %m-%d-%Y, Hour: %H:%M:%S")
        self.__email_msg.attach(MIMEText(body, 'plain'))
        # Attach image
        with open("/home/puzzlebot/person.jpeg", 'rb') as img:
            img_data = img.read()
            image = MIMEImage(img_data)
            image.add_header('Content-ID', '<image1>')
            image.add_header('Content-Disposition', 'inline', filename = "detection.jpeg")
            self.__email_msg.attach(image)
        # Send to server
        with smtplib.SMTP(self.__smtp_server, self.__smtp_port) as server:
            server.starttls()
            server.login("puzzlebot2@hotmail.com", "Puzzlebot#72")
            server.sendmail("puzzlebot2@hotmail.com", self.__send_to, self.__email_msg.as_string())

    def __print_borders(self) -> None:
        self.__markers.markers = []
        for i, type in enumerate(self.__types):
            marker = Marker()
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = "odom"
            marker.ns = type
            marker.id = i
            marker.type = Marker.LINE_LIST
            marker.action = Marker.ADD
            marker.scale.x = 0.03
            marker.color.a = 1.0

            if type == "Current":
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                border = self.__curr_quadrant
                if border is not None:
                    points = [Point(x=border.upper.x, y=border.upper.y), Point(x=border.upper.x, y=border.lower.y),
                            Point(x=border.upper.x, y=border.lower.y), Point(x=border.lower.x, y=border.lower.y),
                            Point(x=border.lower.x, y=border.lower.y), Point(x=border.lower.x, y=border.upper.y),
                            Point(x=border.lower.x, y=border.upper.y), Point(x=border.upper.x, y=border.upper.y)]
                    marker.points = points
            elif type == "Visited":
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
                for border in self.__visited_quadrants:
                    points = [Point(x=border.upper.x, y=border.upper.y), Point(x=border.upper.x, y=border.lower.y),
                              Point(x=border.upper.x, y=border.lower.y), Point(x=border.lower.x, y=border.lower.y),
                              Point(x=border.lower.x, y=border.lower.y), Point(x=border.lower.x, y=border.upper.y),
                              Point(x=border.lower.x, y=border.upper.y), Point(x=border.upper.x, y=border.upper.y)]
                    marker.points.extend(points)
            else:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                for border in self.__quadrants.borders:
                    if border not in self.__visited_quadrants and border != self.__curr_quadrant:
                        points = [Point(x=border.upper.x, y=border.upper.y), Point(x=border.upper.x, y=border.lower.y),
                                Point(x=border.upper.x, y=border.lower.y), Point(x=border.lower.x, y=border.lower.y),
                                Point(x=border.lower.x, y=border.lower.y), Point(x=border.lower.x, y=border.upper.y),
                                Point(x=border.lower.x, y=border.upper.y), Point(x=border.upper.x, y=border.upper.y)]
                        marker.points.extend(points)
            self.__markers.markers.append(marker)
        self.__markers_pub.publish(self.__markers)

    def stop(self) -> None:
        self.__cmd_vel.publish(Twist(linear=Point(x=0.0), angular=Point(z=0.0)))
        self.__process.send_signal(signal.SIGINT)
        self.__process.wait()
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

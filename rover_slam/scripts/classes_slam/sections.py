#!/usr/bin/env python3

# Python libraries
import rospy
import numpy as np
from PIL import Image, ImageDraw

# ROS messages
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point32
from rover_slam.msg import Quadrants, Border

class Map_Sections:
    def __init__(self) -> None:
        self.__map = None
        self.__borders = []

        self.__borders_pub = rospy.Publisher("/borders", Quadrants, queue_size = 10)
        rospy.Subscriber("/map", OccupancyGrid, self.__map_callback)
        rospy.wait_for_message("/map", OccupancyGrid, timeout = 30)

    def __map_callback(self, data:OccupancyGrid) -> None:
        self.__map = np.array(data.data, dtype = np.uint8).reshape((data.info.height, data.info.width))

    def __crop_img(self) -> tuple:
        # Create a new image and crop the unknown areas
        img = Image.fromarray(self.__map).point(lambda p: 255 - p)
        points = img.getbbox()
        img_crop = img.crop(points).point(lambda p: 255 - p)
        img_crop.save("/home/puzzlebot/map.png")
        return points

    def split(self, rows:int, columns:int) -> None:
        points = self.__crop_img()
        img = Image.open("/home/puzzlebot/map.png")

        width, height = img.size
        section_width = width // columns
        section_height = height // rows

        # Get borders from each section
        for i in range(rows):
            for j in range(columns):
                upper = Point32(x = (j * section_width + points[0] - self.__map.shape[1]/2)/40,
                                y = (i * section_height + points[1]  - self.__map.shape[0]/2)/40)
                lower = Point32(x = ((j + 1) * section_width + points[0] - self.__map.shape[1]/2)/40, 
                                y = ((i + 1) * section_height + points[1]- self.__map.shape[0]/2)/40)
                self.__borders.append(Border(upper = upper, lower = lower))
        self.__borders_pub.publish(self.__borders)

        # Draw the lines to look at the sections
        draw = ImageDraw.Draw(img)
        for i in range(1, columns):
            x = i * section_width
            draw.line((x, 0, x, height))
        for i in range(1, rows):
            y = i * section_height
            draw.line((0, y, width, y))
        img.save("/home/puzzlebot/map_sections.png")

    def stop(self) -> None:
        rospy.loginfo("The split map node is stopping")

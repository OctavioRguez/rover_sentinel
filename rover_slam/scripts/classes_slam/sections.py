#!/usr/bin/env python3

# Import python libraries
import rospy
import numpy as np
from PIL import Image, ImageDraw

# Import ROS messages
from nav_msgs.msg import OccupancyGrid

class Map_Sections:
    def __init__(self) -> None:
        # Initialize variables
        self.__map = None
        self.__borders = []
        
        # Subscribe to the odometry and scan topics
        rospy.Subscriber("/map", OccupancyGrid, self.__map_callback)
        rospy.wait_for_message("/map", OccupancyGrid, timeout = 30)

    # Callback function for the SLAM map
    def __map_callback(self, data:OccupancyGrid) -> None:
        # Convert the map data to a numpy array
        self.__map = np.array(data.data, dtype = np.uint8).reshape((data.info.height, data.info.width))

    def __crop_img(self) -> tuple:
        # Create a new image and crop the unknown areas
        img = Image.fromarray(self.__map).point(lambda p: 255 - p)
        points = img.getbbox()
        img_crop = img.crop(points).point(lambda p: 255 - p)
        # Save the image and return points from original map
        img_crop.save("/home/puzzlebot/map.png")
        return points

    def split(self, rows:int, columns:int) -> None:
        # Get crop image
        points = self.__crop_img()
        img = Image.open("/home/puzzlebot/map.png")

        # Get sections size
        width, height = img.size
        section_width = width // columns
        section_height = height // rows

        # Get borders from each section
        for i in range(rows):
            for j in range(columns):
                self.__borders.append({"x" : [(j * section_width + points[0] - self.__map.shape[1]/2)/40,
                                              ((j + 1) * section_width + points[0] - self.__map.shape[1]/2)/40],
                                        "y" : [(i * section_height + points[1]  - self.__map.shape[0]/2)/40,
                                               ((i + 1) * section_height + points[1]- self.__map.shape[0]/2)/40]})
        rospy.loginfo(self.__borders)

        # Draw the lines to separate the sections
        draw = ImageDraw.Draw(img)
        for i in range(1, columns):
            x = i * section_width
            draw.line((x, 0, x, height))
        for i in range(1, rows):
            y = i * section_height
            draw.line((0, y, width, y))

        # Save the image
        img.save("/home/puzzlebot/map_sections.png")

    def stop(self) -> None:
        # Stop the node
        rospy.loginfo("The split map node is stopping")

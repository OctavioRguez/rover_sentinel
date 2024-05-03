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
        self.__height = 0
        self.__width = 0
        self.__borders = []
        
        # Subscribe to the odometry and scan topics
        rospy.Subscriber("/map", OccupancyGrid, self.__map_callback)
        rospy.wait_for_message("/map", OccupancyGrid, timeout = 30)

    # Callback function for the SLAM map
    def __map_callback(self, data:OccupancyGrid) -> None:
        # Convert the map data to a numpy array
        self.__height = data.info.height
        self.__width = data.info.width
        self.__map = np.array(data.data, dtype=np.uint8).reshape((self.__height, self.__width))

    def map_to_image(self) -> None:
        # Create a new image in grayscale mode
        img = Image.new('L', (self.__width, self.__height))
        img.putdata(self.__map .flatten())

        # Save the image
        img.save("/home/puzzlebot/map_image.png")

    def split(self, rows:int, columns:int) -> None:
        # Get sections size
        section_width = self.__width // rows
        section_height = self.__height // columns

        # Get borders from each section
        for i in range(columns):
            for j in range(rows):
                self.__borders.append({"x" : [j * section_width, (j + 1) * section_width],
                                        "y" : [i * section_height, (i + 1) * section_height]})

        # Create a new image to show the section
        img = Image.new('RGB', (self.__width, self.__height), color = (255, 255, 255))
        img.putdata(self.__map.flatten())

        # Draw the lines to separate the sections
        draw = ImageDraw.Draw(img)
        for i in range(1, rows):
            x = i * section_width
            draw.line((x, 0, x, self.__height), fill=(0, 0, 255))
        for i in range(1, columns):
            y = i * section_height
            draw.line((0, y, self.__width, y), fill=(0, 0, 255))

        # Save the image
        img.save("/home/puzzlebot/map_image_sections.png")

    def stop(self) -> None:
        # Stop the node
        rospy.loginfo("The split map node is stopping")
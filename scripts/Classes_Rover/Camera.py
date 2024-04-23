#!/usr/bin/env python

# Import python libraries
import rospy
import cv2 as cv

# Import ROS messages
from sensor_msgs.msg import CompressedImage

class Camera:
    def __init__(self):
        # Variables for the camera
        self.__cam_port =  'nvarguscamerasrc !  video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=10/1 ! nvvidconv flip-method=0 ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink drop=true'
        self.__cam = cv.VideoCapture(self.__cam_port)

        # Publish messages
        self.__img = CompressedImage()
        self.__img.format = "jpeg"

        # Publish for the camera image
        self.__cam_pub = rospy.Publisher("/image/compressed", CompressedImage, queue_size = 10)

    # Stop function
    def _stop(self):    
        self.__cam.release()
        print("Stopping") # Stop message

    # Function for starting the camera
    def getImage(self):
        ret, frame = self.__cam.read() # Get actual frame
        if ret:
            _, compressed_image = cv.imencode(".jpg", frame, [int(cv.IMWRITE_JPEG_QUALITY), 90])
            self.__img.data = compressed_image.tobytes()

        # Compressed image
        self.__img.header.stamp = rospy.Time.now()
        self.__cam_pub.publish(self.__img)

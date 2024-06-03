#!/usr/bin/env python3

# Python libraries
import rospy
import cv2 as cv

# ROS messages
from sensor_msgs.msg import CompressedImage

class Camera:
    def __init__(self) -> None:
        self.__cam_port =  'nvarguscamerasrc ! video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=10/1 ! nvvidconv flip-method=0 ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink drop=true'
        self.__cam = cv.VideoCapture(self.__cam_port)

        self.__img = CompressedImage()
        self.__img.format = "jpeg"

        self.__cam_pub = rospy.Publisher("/image/compressed", CompressedImage, queue_size = 1)

    def get_image(self) -> None:
        ret, frame = self.__cam.read() # Get actual frame
        if ret:
            _, compressed_image = cv.imencode(".jpeg", frame, [int(cv.IMWRITE_JPEG_QUALITY), 90])

            self.__img.header.stamp = rospy.Time.now()
            self.__img.data = compressed_image.tobytes()
            self.__cam_pub.publish(self.__img)
        else:
            rospy.loginfo("Unable to get frame")

    def stop(self) -> None:
        rospy.loginfo("Stopping camera")
        self.__cam.release()

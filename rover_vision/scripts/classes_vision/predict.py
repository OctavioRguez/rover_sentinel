#!/usr/bin/env python3

# Python libraries
import rospy
import cv2 as cv
import numpy as np
import onnxruntime as ort
import torch

# ROS messages
from std_msgs.msg import Bool
from sensor_msgs.msg import CompressedImage

class modelPredict:
    def __init__(self):
        self.__model = rospy.get_param("/classification/model/path", default = "best.onnx")
        self.__conf = rospy.get_param("/classification/confidence/value", default = 0.5)

        self.__color = np.random.uniform(0, 255, 3)
        self.__iou_threshold = 0.5

        self.__build_model(rospy.get_param("/classification/isCuda/value", default = False))
        self.__img = None

        self.__compressedImg = CompressedImage()
        self.__compressedImg.format = "jpeg"

        self.__detection_pub = rospy.Publisher("/prediction/compressed", CompressedImage, queue_size = 10)
        self.__person_pub = rospy.Publisher("/detected/person", Bool, queue_size = 10)
        rospy.Subscriber("/image/compressed", CompressedImage, self.__image_callback)
        rospy.wait_for_message("/image/compressed", CompressedImage, timeout = 30)

    def __image_callback(self, msg:CompressedImage):
        self.__img = msg.data

    def __build_model(self, is_cuda:bool) -> None:
        if is_cuda:
            rospy.loginfo("Attempting to use CUDA")
            self.__session = ort.InferenceSession(self.__model, providers = ['TensorrtExecutionProvider', 'CUDAExecutionProvider'])
        else:
            rospy.loginfo("Running on CPU")
            self.__session = ort.InferenceSession(self.__model, providers = ['CPUExecutionProvider'])
        shape = self.__session.get_inputs()[0].shape
        self.__inputWidth, self.__inputHeight = shape[2:4]

    def __format_img(self, img) -> np.ndarray:
        image = cv.cvtColor(img, cv.COLOR_BGR2RGB)
        image = np.array(cv.resize(image, (self.__inputWidth, self.__inputHeight))) / 255.0 # Resize and normalize (0-1)
        image = np.transpose(image, (2, 0, 1)) # Transpose to have: (channels, width, height)
        return np.expand_dims(image, axis=0).astype(np.float16) # Add batch dimension to create tensor (b, c, w, h)

    def __detect(self, img) -> np.ndarray:
        inputs = {self.__session.get_inputs()[0].name: img} # Prepare the input for the model
        preds = self.__session.run(None, inputs)
        return np.transpose(np.squeeze(preds[0])) # Remove batch dimension

    def __calculate_iou(self, box1:tuple, box2:tuple) -> float:
        x1, y1, w1, h1 = box1
        x2, y2, w2, h2 = box2

        x1_min, y1_min, x1_max, y1_max = x1, y1, x1 + w1, y1 + h1
        x2_min, y2_min, x2_max, y2_max = x2, y2, x2 + w2, y2 + h2

        inter_x_min = max(x1_min, x2_min)
        inter_y_min = max(y1_min, y2_min)
        inter_x_max = min(x1_max, x2_max)
        inter_y_max = min(y1_max, y2_max)

        inter_area = max(0, inter_x_max - inter_x_min) * max(0, inter_y_max - inter_y_min)
        union_area = (w1 * h1) + (w2 * h2) - inter_area
        return inter_area / union_area

    def __non_maximum_suppression(self, boxes:list, scores:list) -> list:
        indices = np.argsort(scores)[::-1]
        selected_indices = []
        while len(indices) > 0:
            current_index = indices[0]
            selected_indices.append(current_index)
            
            remaining_indices = indices[1:]
            ious = np.array([self.__calculate_iou(boxes[current_index], boxes[i]) for i in remaining_indices])

            indices = remaining_indices[ious < self.__iou_threshold]
        return selected_indices

    def __wrap_detection(self, modelOutput:np.ndarray) -> list:
        boxes, scores = [], []

        x_factor = self.__imgWidth / self.__inputWidth
        y_factor = self.__imgHeight / self.__inputHeight

        rows = modelOutput.shape[0]
        for r in range(rows):
            row = modelOutput[r]
            if (row[4] > self.__conf):
                x, y, w, h = row[0].item(), row[1].item(), row[2].item(), row[3].item() # Get the bounding box coordinates

                # Scale the bounding box coordinates
                left = (x - 0.5 * w) * x_factor
                top = (y - 0.5 * h) * y_factor
                width, height = w*x_factor, h*y_factor
                scores.append(row[4])
                boxes.append(np.array([left, top, width, height]))

        indices = self.__non_maximum_suppression(boxes, scores)
        return [boxes[i] for i in indices]

    def __start_detection(self, imgData:list) -> None:
        img = cv.imdecode(np.frombuffer(imgData, np.uint8), cv.IMREAD_COLOR)
        self.__imgHeight, self.__imgWidth = img.shape[:2]

        formatImg = self.__format_img(img)
        outs = self.__detect(formatImg)
        boxes = self.__wrap_detection(outs)

        if boxes:
            self.__person_pub.publish(True)
            for i in range(len(boxes)):
                x, y, w, h = tuple(map(int, boxes[i]))
                cv.rectangle(img, (x, y), (x + w, y + h), self.__color, 2) # Bounding box
            cv.imwrite("/home/puzzlebot/person.jpeg", img)
        return cv.imencode(".jpeg", img)[1].tobytes()

    def predict(self) -> None:
        img = self.__start_detection(self.__img)
        self.__compressedImg.header.stamp = rospy.Time.now()
        self.__compressedImg.data = img
        self.__detection_pub.publish(self.__compressedImg)

    def stop(self) -> None:
        rospy.loginfo("Stoping the Classificator Node")

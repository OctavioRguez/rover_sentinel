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
        self.__class_list = rospy.get_param("/classification/classes/list", default = [])
        self.__colors = np.random.uniform(0, 255, size=(len(self.__class_list), 3))
        self.__conf = rospy.get_param("/classification/confidence/value", default = 0.7)

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
        return np.squeeze(preds[0]) # Remove batch dimension

    def __wrap_detection(self, modelOutput:np.ndarray) -> list:
        class_ids, boxes, scores = [], [], []

        x_factor = self.__imgWidth / self.__inputWidth
        y_factor = self.__imgHeight / self.__inputHeight

        rows = modelOutput.shape[0]
        for r in range(rows):
            row = modelOutput[r]
            
            if (row[4] > self.__conf):
                classes_scores = row[5:]
                class_id = np.argmax(classes_scores)
                max_score = classes_scores[class_id]
                
                if (max_score > self.__conf):
                    x, y, w, h = row[0].item(), row[1].item(), row[2].item(), row[3].item() # Get the bounding box coordinates

                    # Scale the bounding box coordinates
                    left = (x - 0.5 * w) * x_factor
                    top = (y - 0.5 * h) * y_factor
                    width, height = w*x_factor, h*y_factor

                    class_ids.append(class_id)
                    scores.append(max_score)
                    boxes.append(np.array([left, top, width, height]))

        final_class_ids, final_boxes, final_scores = [], [], []
        for i in range(len(boxes)):
            final_class_ids.append(class_ids[i])
            final_boxes.append(boxes[i])
            final_scores.append(scores[i])
        return final_class_ids, final_boxes, final_scores

    def __start_detection(self, imgData:list):
        img = cv.imdecode(np.frombuffer(imgData, np.uint8), cv.IMREAD_COLOR)
        self.__imgHeight, self.__imgWidth = img.shape[:2]

        formatImg = self.__format_img(img)
        outs = self.__detect(formatImg)
        class_ids, boxes, scores = self.__wrap_detection(outs)

        if class_ids:
            # Get the detected object with the highest score
            index = np.argmax(scores)
            self.__person_pub.publish(True)

            x, y, w, h = tuple(map(int, boxes[index]))
            color = self.__colors[class_ids[index]]
            cv.rectangle(img, (x, y), (x + w, y + h), color, 2) # Bounding box
        return cv.imencode('.jpeg', img)[1].tobytes()

    def predict(self) -> None:
        img = self.__start_detection(self.__img)
        self.__compressedImg.header.stamp = rospy.Time.now()
        self.__compressedImg.data = img
        self.__detection_pub.publish(self.__compressedImg)

    def stop(self) -> None:
        rospy.loginfo("Stoping the Classificator Node")

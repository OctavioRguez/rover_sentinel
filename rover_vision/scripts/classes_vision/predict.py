#!/usr/bin/env python3

# Import python libraries
import rospy
import cv2 as cv
import numpy as np
# import torch
import onnxruntime as ort

# Import ROS messages
from sensor_msgs.msg import CompressedImage

class modelPredict:
    def __init__(self):
        # Initialize variables
        self.__model = rospy.get_param("/classification/model/path", default = "best.onnx")
        self.__class_list = rospy.get_param("/classification/classes/list", default = [])
        self.__colors = np.random.uniform(0, 255, size=(len(self.__class_list), 3))
        self.__conf = rospy.get_param("/classification/confidence/value", default = 0.7)

        # Build model
        self.__buildModel(rospy.get_param("/classification/isCuda/value", default = False))
        self.__img = None

        # Publish messages
        self.__compressedImg = CompressedImage()
        self.__compressedImg.format = "jpeg"

        # Publisher for classificated image
        self.__detection_pub = rospy.Publisher("/prediction/compressed", CompressedImage, queue_size = 1)
        
        # Subscribe to compressed image topic
        rospy.Subscriber("/image/compressed", CompressedImage, self.__imageCallback)
        rospy.wait_for_message("/image/compressed", CompressedImage, timeout = 30)

    def __imageCallback(self, msg:CompressedImage):
        # Start the detection process
        self.__img = msg.data

    # Define if opencv runs with CUDA or CPU (False = CPU, True = CUDA)
    def __buildModel(self, is_cuda:bool) -> None:
        if is_cuda:
            rospy.loginfo("Attempting to use CUDA")
            self.__session = ort.InferenceSession(self.__model, providers = ['CUDAExecutionProvider'])
        else:
            rospy.loginfo("Running on CPU")
            self.__session = ort.InferenceSession(self.__model, providers = ['CPUExecutionProvider'])
        # Get the input image shape for the model (width, height)
        shape = self.__session.get_inputs()[0].shape
        self.__inputWidth, self.__inputHeight = shape[2:4]

    def __formatImg(self, img) -> np.ndarray:
        image = cv.cvtColor(img, cv.COLOR_BGR2RGB)
        image = np.array(cv.resize(image, (self.__inputWidth, self.__inputHeight))) / 255.0 # Resize (input shape) and normalize (0-1)
        image = np.transpose(image, (2, 0, 1)) # Transpose to have: (channels, width, height)
        return np.expand_dims(image, axis=0).astype(np.float32) # Add batch dimension to create tensor (b, c, w, h)

    def __detect(self, img) -> np.ndarray:
        inputs = {self.__session.get_inputs()[0].name: img} # Prepare the input for the model
        preds = self.__session.run(None, inputs) # Perform inference
        return np.squeeze(preds[0]) # Remove batch dimension

    def __wrapDetection(self, modelOutput:np.ndarray, object:str) -> list:
        # Initialize lists
        class_ids, boxes, scores = [], [], []

        # Calculate the scaling factor
        x_factor = self.__imgWidth / self.__inputWidth
        y_factor = self.__imgHeight / self.__inputHeight

        # Iterate over the model output
        rows = modelOutput.shape[0]
        for r in range(rows):
            row = modelOutput[r]
            
            # Check if the object confidence is greater than the threshold
            if row[4] > self.__conf:
                classes_scores = row[5:]
                class_id = np.argmax(classes_scores)
                max_score = classes_scores[class_id]
                
                # Check if the score is greater than the threshold and if the detected object is the desired one
                if (max_score > self.__conf) and (self.__class_list[class_id] == object):
                    x, y, w, h = row[0].item(), row[1].item(), row[2].item(), row[3].item() # Get the bounding box coordinates

                    # Scale the bounding box coordinates
                    left = (x - 0.5 * w) * x_factor
                    top = (y - 0.5 * h) * y_factor
                    width, height = w*x_factor, h*y_factor

                    # Append the results to the lists
                    class_ids.append(class_id)
                    scores.append(max_score)
                    boxes.append(np.array([left, top, width, height]))

        # Get the final results
        final_class_ids, final_boxes, final_scores = [], [], []
        for i in range(len(boxes)):
            final_class_ids.append(class_ids[i])
            final_boxes.append(boxes[i])
            final_scores.append(scores[i])
        return final_class_ids, final_boxes, final_scores

    def __startDetection(self, imgData:list, object:str):
        # Decode the image
        img = cv.imdecode(np.frombuffer(imgData, np.uint8), cv.IMREAD_COLOR)
        # Get the image shapes
        self.__imgHeight, self.__imgWidth = img.shape[:2]

        # Perform the detection
        formatImg = self.__formatImg(img)
        outs = self.__detect(formatImg)
        class_ids, boxes, scores = self.__wrapDetection(outs, object)

        if class_ids:
            # Get the detected object with the highest score
            index = np.argmax(scores)
            # Decompress the bounding box coordinates
            x, y, w, h = tuple(map(int, boxes[index]))
            color = self.__colors[class_ids[index]]
            # Draw the bounding box for the object
            cv.rectangle(img, (x, y), (x + w, y + h), color, 2)
            # Draw the label background
            cv.rectangle(img, (x, y - 15), (x + w, y + 15), color, -1)
            # Draw the label and confidence of the object
            cv.putText(img, f"{object}", (x, y + 10), cv.FONT_HERSHEY_SIMPLEX, 1, (0,0,0), 1, cv.LINE_AA)
        return cv.imencode('.jpeg', img)[1].tobytes()

    def start(self) -> None:
        img = self.__startDetection(self.__img, "Persona")
        self.__compressedImg.header.stamp = rospy.Time.now()
        self.__compressedImg.data = img
        self.__detection_pub.publish(self.__compressedImg)

    def stop(self) -> None:
        rospy.loginfo("Stoping the Classificator Node")
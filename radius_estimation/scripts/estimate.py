#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pyrealsense2 import pyrealsense2 as rs
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Float32MultiArray
import cv2
import numpy as np
import os
# Modeli yükleme
import joblib



class Estimator:
    def __init__(self):
        self.model = joblib.load("/home/cenk/regression_model2.joblib")  #import pretrained model
        self.bridge = CvBridge()
        self.depth_scale = 0.001
        rospy.init_node('regression_node')
        rospy.Subscriber('/yolov7/yolov7', Detection2DArray, self.image_callback)
        rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.depth_callback)
        self.pub = rospy.Publisher("/radius_estimator", Float32MultiArray, queue_size=10)


    def image_callback(self, detection_array):
        # Bbox to numpy
        predictions=[]
        for detection in detection_array.detections:
            for result in detection.results:
                if result.id == 47 or result.id == 49 or result.id == 80:  # Nesne ID'si 32 ise
                    bbox = detection.bbox
                    depth = (self.depth_image[int(bbox.center.y), int(bbox.center.x)]) * self.depth_scale
                    features = np.array([depth, bbox.size_x, bbox.size_y])
                    features = features.reshape(1, -1)  # Ekstra bir boyut ekleniyor
                    predict = self.model.predict(features)
                    predictions.append(predict)
                    print("Radius of ball predicted as %f cm" % predict)
        msg = Float32MultiArray()
        msg.data = np.array(predictions).flatten().tolist()
        self.pub.publish(msg)  # Kontrol et

                

    def depth_callback(self, depth_image):
        self.depth_image = self.bridge.imgmsg_to_cv2(depth_image, desired_encoding="passthrough")



if __name__ == '__main__':
    estimator=Estimator()
    rospy.spin()

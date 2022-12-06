#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from yolov4 import Detector
from object_search.srv import ObjectDetection, ObjectDetectionRequest, ObjectDetectionResponse
import cv2

class ObjectDetector:
    
    def __init__(self):
        self.image = None
        self.bridge = CvBridge()
        self.detector = Detector(gpu_id=0, config_path='/opt/darknet/cfg/yolov4.cfg',
                                weights_path='/opt/darknet/yolov4.weights',
                                lib_darknet_path='/opt/darknet/libdarknet.so',
                                meta_path='/opt/darknet/cfg/coco.data')

    def imageFrameCB(self, msg) -> None:
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def objectDetectionCB(self, msg: ObjectDetectionRequest) -> ObjectDetectionResponse:
        if self.image is not None:
            cv_copy = self.image.copy()

            img_arr = cv2.resize(cv_copy, (self.detector.network_width(), self.detector.network_height()))
            detections = self.detector.perform_detect(image_path_or_buf=img_arr, show_image=False)

            for detectionEntry in detections:
                if detectionEntry.class_name == msg.searchFor:
                    return ObjectDetectionResponse(True)
        else:
            return ObjectDetectionResponse(False)

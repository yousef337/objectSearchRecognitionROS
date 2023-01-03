#!/usr/bin/env python3
import rospy
from object_search.srv import (
    LocationSampling,
    Rasterization,
    ObjectDetection,
    ObjectFinder,
)
from RasterizationSrv import rasterizationCB
from PointsSamplingSrv import sampleRoomCB
from ObjectDetection import ObjectDetector
from sensor_msgs.msg import Image
from main import objectFinderCB

rospy.init_node('ServicesNode')

yoloDetector = ObjectDetector()
rospy.Subscriber('/xtion/rgb/image_raw', Image, yoloDetector.imageFrameCB)
rospy.Service(
    'objectDetection', ObjectDetection, yoloDetector.objectDetectionCB
)

rospy.Service('Rasterization', Rasterization, rasterizationCB)
rospy.Service('LocationSampling', LocationSampling, sampleRoomCB)
rospy.Service('Search', ObjectFinder, objectFinderCB)
rospy.spin()

#!/usr/bin/env python3
import rospy
from object_search.srv import LocationSampling, Rasterization
from RasterizationSrv import rasterizationCB
from PointsSamplingSrv import sampleRoomCB


rospy.init_node('ServicesNode')
rospy.Service('Rasterization', Rasterization, rasterizationCB)
rospy.Service('LocationSampling', LocationSampling, sampleRoomCB)
rospy.spin()

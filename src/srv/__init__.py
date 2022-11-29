import rospy
from .RasterizationSrv import rasterizationCB


rospy.init_node('ServicesNode')
rospy.Service('Rasterization', Rasterization, rasterizationCB)
rospy.spin()
#!/usr/bin/env python

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from kinect_msgs.msg import Kinect


class KinectDisplay(object):
	def __init__(self):
		rospy.init_node('kinect_display')
		self.bridge = CvBridge()
		kinect_color_sub = rospy.Subscriber('/camera/left', Image, self.image_cb)
		rospy.spin()

	def image_cb(self, message):

		cv_image = self.bridge.imgmsg_to_cv2(message)
		print message.header.stamp
		cv2.imshow("Color", cv_image)
		cv2.waitKey(5)

if __name__ == '__main__':
	try:
		KinectDisplay()
	except rospy.ROSInterruptException:
		rospy.logerr('Could not start kinect publisher node.')

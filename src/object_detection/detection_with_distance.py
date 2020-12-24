#!/usr/bin/env python

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from kinect_msgs.msg import Kinect
from darkflow.net.build import TFNet
import os
import rospkg
from rospy_message_converter import message_converter
import message_filters
from kinect_msgs.msg import Detection
from kinect_msgs.msg import SingleDetection
from sensor_msgs.msg import Image
from std_msgs.msg import Int16


class DetectionWithDistance(object):
	def __init__(self):
		rospy.init_node('detection_with_distance')
		self.detection_pub = rospy.Publisher('/detection', Detection, queue_size=1)
		self.nearest_object_pub = rospy.Publisher('/nearest_object', Int16, queue_size=1)
		rp = rospkg.RosPack()
		options = {"model": os.path.join(rp.get_path("object_detection"), "cfg", "yolo-voc-tiny.cfg"), "load": os.path.join(rp.get_path("object_detection"), "bin", "tiny-yolo-voc.weights"), "labels": os.path.join(rp.get_path("object_detection"), "labels", "labels.txt"), "threshold": 0.3, "gpu": 0.4}
		self.tfnet = TFNet(options)
		self.bridge = CvBridge()
		self.message = Kinect()

		self.image_color_sub = message_filters.Subscriber('/kinect2/qhd/image_color_rect', Image)
		self.image_depth_sub = message_filters.Subscriber('/kinect2/qhd/image_depth_rect', Image)

		self.ts = message_filters.ApproximateTimeSynchronizer([self.image_color_sub, self.image_depth_sub], queue_size=10, slop=0.1)
		self.ts.registerCallback(self.images_cb)

		rospy.spin()

	def images_cb(self, message1, message2):
		mode = rospy.get_param("mode")
		if mode == "manual":
			self.nearest_object_pub.publish(9999)
		mode = rospy.get_param("mode")
		if mode == "autonomous":
			detection_msg = Detection()

			single_detection = SingleDetection()
			self.color = message1
			self.depth = message2

			self.color_array = self.bridge.imgmsg_to_cv2(self.color)
			self.color_array = self.color_array[:, :, 0:3]
			self.depth_array = self.bridge.imgmsg_to_cv2(self.depth)

			result = self.tfnet.return_predict(self.color_array)

			detection_dist_arr = []

			for rows in result:
				detection_dist_arr = []

				x1 = rows['topleft']['x']
				y1 = rows['topleft']['y']
				x2 = rows['bottomright']['x']
				y2 = rows['bottomright']['y']
				label = rows['label']
				confidence = rows['confidence']

				single_detection.topleft.x = x1
				single_detection.topleft.y = y1
				single_detection.bottomright.x = x2
				single_detection.bottomright.y = y2
				single_detection.label = label
				single_detection.confidence = confidence

				if label == "person":

					area = np.array(self.depth_array[y1:y2, x1:x2])
					area[area == 0] = 999999
					dist = np.min(area)
					single_detection.distance = dist
					detection_dist_arr.append(dist)
					detection_msg.detection.append(single_detection)

			self.detection_pub.publish(detection_msg)

			detection_dist_arr = np.array(detection_dist_arr)
			if detection_dist_arr.size > 0:
				nearest_obj_dist = np.min(detection_dist_arr)
			else:
				nearest_obj_dist = 9999
			self.nearest_object_pub.publish(nearest_obj_dist)


if __name__ == '__main__':
	try:
		DetectionWithDistance()
	except rospy.ROSInterruptException:
		rospy.logerr('Could not start detection with distance node.')

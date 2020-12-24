#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from kinect_msgs.msg import Kinect
from keras.models import load_model
import h5py
import csv
import cv2
from drive_by_wire_msgs.msg import DriveByWire
from keras.models import model_from_json
from behavior_cloning_srv.srv import *


class BehaviorCloningClass(object):
    def __init__(self):
        rospy.init_node('behavior_cloning')
        self.bridge = CvBridge()
        self.pred = DriveByWire()
        rospy.wait_for_service('behavior_cloning')
        self.behavior_cloning = rospy.ServiceProxy('behavior_cloning', BehaviorCloning)
        rospy.Subscriber('/camera/left', Image, self.cloning_cb)
        self.auto_pub = rospy.Publisher('/dbw/autonomous', DriveByWire, queue_size=1)
        rospy.spin()

    def cloning_cb(self, message):
        resp = self.behavior_cloning(message)
        self.pred.steering = resp.steering
        self.pred.accelerator = resp.accelerator
        self.pred.brake = resp.brake
        self.auto_pub.publish(self.pred)

if __name__ == '__main__':
	try:
		BehaviorCloningClass()
	except rospy.ROSInterruptException:
		rospy.logerr('Could not start behavior cloning node.')

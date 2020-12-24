#!/usr/bin/env python

import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import cv2


class LeftCamera(object):
    def __init__(self):
        rospy.init_node('left_camera')
        self.bridge = CvBridge()
        self.left_camera_pub = rospy.Publisher('/camera/left', Image, queue_size=1)

        self.message_image = Image()
        self.message_header = Header()

        self.cap = cv2.VideoCapture(0)
        while not rospy.is_shutdown():
            self.ret, self.frame = self.cap.read()
            if self.ret is not False:
                # self.small_frame = cv2.resize(self.frame, (320, 160))
                self.message_image = self.bridge.cv2_to_imgmsg(self.frame, encoding="bgr8")
                self.message_image.header.stamp = rospy.Time.now()
                self.left_camera_pub.publish(self.message_image)

if __name__ == '__main__':
    try:
        LeftCamera()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start left camera node.')
        self.cap.release()

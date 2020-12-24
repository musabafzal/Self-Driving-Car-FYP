#!/usr/bin/env python

import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import cv2


class RightCamera(object):
    def __init__(self):
        rospy.init_node('right_camera')
        self.bridge = CvBridge()
        self.right_camera_pub = rospy.Publisher('/camera/right', Image, queue_size=1)

        self.message_image = Image()
        self.message_header = Header()

        self.cap = cv2.VideoCapture(3)
        while not rospy.is_shutdown():
            self.ret, self.frame = self.cap.read()
            if self.ret is not False:
                # self.small_frame = cv2.resize(self.frame, (320, 160))
                self.message_image = self.bridge.cv2_to_imgmsg(self.frame, encoding="bgr8")
                self.message_image.header.stamp = rospy.Time.now()
                self.right_camera_pub.publish(self.message_image)

if __name__ == '__main__':
    try:
        RightCamera()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start right camera node.')
        self.cap.release()

#!/usr/bin/env python

import numpy as np
import rospy
import os
import rospkg
import message_filters
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from drive_by_wire_msgs.msg import DriveByWire
import datetime
import csv
import cv2
from cv_bridge import CvBridge, CvBridgeError


class DataExtractor(object):
    def __init__(self):
        rospy.init_node('data_extractor')
        self.extractor_pub = rospy.Publisher('/extractor', Int16, queue_size=1)
        self.bridge = CvBridge()
        self.kinect_sub = message_filters.Subscriber('/kinect2/qhd/image_color_rect', Image)
        self.left_camera_sub = message_filters.Subscriber('/camera/left', Image)
        self.right_camera_sub = message_filters.Subscriber('/camera/right', Image)
        self.dbw_input_sub = message_filters.Subscriber('/dbw/input', DriveByWire)

        self.csv_file = open('/home/musab/data/driving_log.csv', 'w')
        fieldnames = ['image', 'steering', 'accelerator', 'brake']
        self.writer = csv.DictWriter(self.csv_file, fieldnames=fieldnames)
        self.writer.writeheader()

        self.count = 0

        self.ts = message_filters.ApproximateTimeSynchronizer([self.left_camera_sub, self.dbw_input_sub], queue_size=10, slop=0.05)
        self.ts.registerCallback(self.data_cb)

        rospy.spin()

    def data_cb(self, left_camera_msg, dbw_input_msg):
        time_stamp = dbw_input_msg.header.stamp.to_sec()
        date = datetime.datetime.fromtimestamp(time_stamp)
        image = self.bridge.imgmsg_to_cv2(left_camera_msg)
        image_path = "/home/musab/data/images"
        image_name = image_path + "/left_" + str(date) + ".jpg"
        cv2.imwrite(image_name, image)

        csv_image_name = "images/left_" + str(date) + ".jpg"
        self.writer.writerow({'image': csv_image_name, 'steering': dbw_input_msg.steering, 'accelerator': dbw_input_msg.accelerator, 'brake': dbw_input_msg.brake})
        self.count = self.count + 1
        print self.count
        self.extractor_pub.publish(self.count)

if __name__ == '__main__':
    try:
        DataExtractor()
    except rospy.ROSInterruptException:
        self.csv_file.close()
        rospy.logerr('Could not start data extractor node.')

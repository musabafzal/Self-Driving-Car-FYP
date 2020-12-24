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
import tensorflow as tf
from keras.backend.tensorflow_backend import set_session


class BehaviorCloningServer(object):
    def __init__(self):
        rospy.init_node('behavior_cloning_server')
        self.bridge = CvBridge()
        # config = tf.ConfigProto()
        # config.gpu_options.per_process_gpu_memory_fraction = 0.4
        # set_session(tf.Session(config=config))
        self.model = load_model("/home/musab/data/model.h5")
        self.model._make_predict_function()
        self.graph = tf.get_default_graph()
        rospy.Service('behavior_cloning', BehaviorCloning, self.handle_behavior_cloning)
        rospy.spin()

    def handle_behavior_cloning(self, req):
        cv_image = self.bridge.imgmsg_to_cv2(req.input)
        cv_image = cv2.resize(cv_image, (320, 240))
        cv_image = np.reshape(cv_image, [1, 240, 320, 3])
        with self.graph.as_default():
            prediction = self.model.predict(cv_image)

        steering = int(prediction[0][0] * 1012)
        accelerator = int(prediction[0][1] * 600)
        brake = int(prediction[0][2] * 600)
        return BehaviorCloningResponse(steering, accelerator, brake)


if __name__ == '__main__':
	try:
		BehaviorCloningServer()
	except rospy.ROSInterruptException:
		rospy.logerr('Could not start behavior cloning node.')

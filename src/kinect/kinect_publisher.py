#!/usr/bin/env python

from pylibfreenect2.libfreenect2 import Freenect2, SyncMultiFrameListener, Registration, FrameType, FrameMap, Frame
import numpy as np
import rospy
from cv_bridge import CvBridge
from kinect_msgs.msg import Kinect
import cv2


class KinectPublisher(object):
    def __init__(self):
        rospy.init_node('kinect_publisher')
        self.kinect_pub = rospy.Publisher('/kinect/images', Kinect, queue_size=20)
        self.bridge = CvBridge()

        self.message = Kinect()

        fn = Freenect2()
        assert fn.enumerateDevices() > 0

        device = fn.openDefaultDevice()
        self.listener = SyncMultiFrameListener(FrameType.Color | FrameType.Ir | FrameType.Depth)
        device.setColorFrameListener(self.listener)
        device.setIrAndDepthFrameListener(self.listener)
        device.startStreams(True, True)

        self.frames = FrameMap()

        while not rospy.is_shutdown():
            self.listener.waitForNewFrame(self.frames)

            self.color = self.frames["color"]
            self.ir = self.frames["ir"]
            self.depth = self.frames["depth"]
            self.color_array = np.asarray(self.color.asarray(dtype=np.uint8))
            # self.color_array_small = np.asarray(self.color_array[:, 240:1680, :])

            # self.color_array = cv2.resize(self.color_array, (1920, 480))
            # self.color_array_small = cv2.resize(self.color_array_small, (640, 480))
            self.message.color = self.bridge.cv2_to_imgmsg(self.color_array, encoding="bgra8")
            self.message.depth = self.bridge.cv2_to_imgmsg(self.depth.asarray(dtype=np.float32), encoding="passthrough")
            # self.message.color_small = self.bridge.cv2_to_imgmsg(self.color_array_small, encoding="bgra8")

            # self.registration = Registration(device.getIrCameraParams(), device.getColorCameraParams())

            # self.undistorted = Frame(512, 424, 4)
            # self.registered = Frame(512, 424, 4)
            # self.bigdepth = Frame(1920, 1082, 4)

            # self.registration.apply(self.color, self.depth, self.undistorted, self.registered, bigdepth=self.bigdepth)
            # self.bigdepth = np.array(self.bigdepth.asarray(dtype=np.float32))
            # self.bigdepth = self.bigdepth[1:1081, :]
            # self.bigdepth = cv2.resize(self.bigdepth, (640, 480))
            # self.message.depth = self.bridge.cv2_to_imgmsg(self.bigdepth, encoding="passthrough")
            self.kinect_pub.publish(self.message)

            self.listener.release(self.frames)

if __name__ == '__main__':
    try:
        KinectPublisher()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start kinect publisher node.')
        device.stop()
        device.close()

#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16


class ModeSetter(object):
    def __init__(self):
		rospy.init_node('mode_setter')
		kinect_color_sub = rospy.Subscriber('/mode', Int16, self.mode_cb)
		rospy.spin()

    def mode_cb(self, message):
        mode = message.data
        if mode == 1:
            rospy.set_param('mode', 'autonomous')
        elif mode == 0:
            rospy.set_param('mode', 'manual')

if __name__ == '__main__':
	try:
		ModeSetter()
	except rospy.ROSInterruptException:
		rospy.logerr('Could not start mode setter node.')

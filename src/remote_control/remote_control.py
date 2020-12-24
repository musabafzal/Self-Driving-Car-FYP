#!/usr/bin/env python

import rospy
from pynput.keyboard import Key, Listener
import getch
from std_msgs.msg import Int16


class RemoteControl(object):
	def __init__(self):
		rospy.init_node('remote_control')
		dbw_steer_out_pub = rospy.Publisher('/dbw/steer_output', Int16, queue_size=1)
		dbw_accelerator_out_pub = rospy.Publisher('/dbw/accelerator_output', Int16, queue_size=1)
		dbw_brake_out_pub = rospy.Publisher('/dbw/brake_output', Int16, queue_size=1)
		accelerator = 0
		brake = 170
		steering = 3

		while not rospy.is_shutdown():
			mode = rospy.get_param("mode")
			if mode == "remote":
				accelerator = 0
				while True:
					key = getch.getch()
					if key == "w":
						accelerator = accelerator + 5
						brake = 170
						if accelerator > 255:
							accelerator = 255

					if key == "e":
						accelerator = accelerator - 5
						brake = 170
						if accelerator < 0:
							accelerator = 0
					if key == "r":
						accelerator = 0
						brake = 90
					if key == "a":
						steering = 1
					if key == "s":
						steering = 3
					if key == "d":
						steering = 2
					count1 = 0
					while count1 < 20:
						dbw_accelerator_out_pub.publish(accelerator)
						count1 = count1 + 1
					count2 = 0
					while count2 < 20:
						dbw_brake_out_pub.publish(brake)
						count2 = count2 + 1
					count3 = 0
					while count3 < 20:
						dbw_steer_out_pub.publish(steering)
						count3 = count3 + 1
					print "accelerator: " + str(accelerator)
					print "brake: " + str(brake)
					print "steering: " + str(steering)


if __name__ == '__main__':
	try:
		RemoteControl()
	except rospy.ROSInterruptException:
		rospy.logerr('Could not start remote controller node.')

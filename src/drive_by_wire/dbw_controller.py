#!/usr/bin/env python

import serial
import rospy
from drive_by_wire_msgs.msg import DriveByWire
from std_msgs.msg import Int16
import thread as thread


class DriveByWireController(object):
    def __init__(self):
        rospy.init_node('dbw_controller')

        self.dbw_steer_out_pub = rospy.Publisher('/dbw/steer_output', Int16, queue_size=1)
        self.dbw_accelerator_out_pub = rospy.Publisher('/dbw/accelerator_output', Int16, queue_size=1)
        self.dbw_brake_out_pub = rospy.Publisher('/dbw/brake_output', Int16, queue_size=1)

        self.input = DriveByWire()
        self.input.steering = 0
        self.front = 0
        self.nearest_object = 9999

        mode = rospy.get_param("mode")
        rospy.Subscriber('/dbw/input', DriveByWire, self.input_cb)
        rospy.Subscriber('/dbw/autonomous', DriveByWire, self.autonomous_cb)
        rospy.Subscriber('/dbw/front', Int16, self.front_cb)
        rospy.Subscriber('/nearest_object', Int16, self.nearest_object_cb)

        rospy.spin()

    def nearest_object_cb(self, message):
        self.nearest_object = message.data

    def input_cb(self, message):
        mode = rospy.get_param("mode")
        # print mode
        if mode == "manual":
            self.input.steering = message.steering
            self.input.accelerator = message.accelerator
            self.input.brake = message.brake

            self.steering_control()
            self.accelerator_control()
            self.brake_control()
        elif mode == "remote":
            pass
        else:
            self.dbw_accelerator_out_pub.publish(0)
            self.dbw_brake_out_pub.publish(170)
            self.dbw_steer_out_pub.publish(3)

    def autonomous_cb(self, message):
        mode = rospy.get_param("mode")
        if mode == "autonomous":
            self.input.steering = message.steering
            self.input.accelerator = message.accelerator
            self.input.brake = message.brake

            self.steering_control()
            self.accelerator_control()
            self.brake_control()
        else:
            pass

    def front_cb(self, message):
        self.front = message.data

    def steering_control(self):
        tolerance = 35

        target = self.map(self.input.steering, 0, 1012, 0, 525)

        if target < self.front - tolerance:
            command = 1

        elif target > self.front + tolerance:
            command = 2
        else:
            command = 3
        self.dbw_steer_out_pub.publish(command)

    def accelerator_control(self):
        command = self.map(self.input.accelerator, 250, 600, 0, 255)
        if command < 0:
            command = 0
        if command > 255:
            command = 255
        if self.nearest_object < 1500:
            command = 0
        self.dbw_accelerator_out_pub.publish(command)

    def brake_control(self):
        command = self.map(self.input.brake, 250, 600, 170, 90)
        if command < 90:
            command = 90
        if command > 170:
            command = 170
        if self.nearest_object < 1500:
            command = 90
        self.dbw_brake_out_pub.publish(command)

    def map(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

if __name__ == '__main__':
    try:
        DriveByWireController()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start drive by wire controller node.')

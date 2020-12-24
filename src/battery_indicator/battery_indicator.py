#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16


class BatteryIndicator(object):
    def __init__(self):
        rospy.init_node('battery_indicator')
        self.battery_level_pub = rospy.Publisher('/battery/level', Int16, queue_size=1)

        while not rospy.is_shutdown():
            try:  
                power_now = open("/sys/class/power_supply/BAT0/charge_now", "r").readline()
            except IOError:
                pass
            try:
                power_full = open("/sys/class/power_supply/BAT0/charge_full", "r").readline()
            except IOError:
                pass
            level = float(power_now) / float(power_full) * 100

            if (level > 0 and level <= 20):
                command = 1
            elif(level > 20 and level <= 50):
                command = 2
            elif(level > 50 and level <= 75):
                command = 3
            elif (level > 75 and level <= 100):
                command = 4
            self.battery_level_pub.publish(command)

if __name__ == '__main__':
    try:
        BatteryIndicator()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start battery indicator node.')
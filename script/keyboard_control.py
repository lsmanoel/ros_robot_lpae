#!/usr/bin/env python3
from __future__ import print_function
import sys
print(sys.version)
import pygame
import time
import roslib
import rospy
from std_msgs.msg import Int8

def keyboard_control():
    pub = rospy.Publisher('driver_L_command', Int8, queue_size=10)
    pub = rospy.Publisher('driver_R_command', Int8, queue_size=10)

    rospy.init_node('keyboard_control', anonymous=True)

    rate = rospy.Rate(25)

    while kb.is_pressed('esc') is not True:
        dutycycle_L = 0
        dutycycle_R = 0
        if kb.is_pressed('w') is True:
            dutycycle_L = 63
            dutycycle_R = 63
        if kb.is_pressed('a') is True:
            dutycycle_L = dutycycle_L + 64
        if kb.is_pressed('d') is True:
            dutycycle_R = dutycycle_R + 64
        if kb.is_pressed('s') is True:
            dutycycle_L = dutycycle_L - 64
            dutycycle_R = dutycycle_R - 64

        print(dutycycle_L, dutycycle_R)
        rate.sleep()

print("end")

if __name__ == '__main__':
    try:
        keyboard_control()
    except rospy.ROSInterruptException:
        pass

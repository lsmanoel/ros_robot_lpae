#!/usr/bin/env python
from __future__ import print_function
import rospy
from std_msgs.msg import Int8

class DriverController(object):
    """docstring for DriverController"""
    def __init__(self):
        rospy.init_node('driver_controller', anonymous=True)

        self.rate = rospy.Rate(120)#Hz

        self.keyboard_L_command = 0
        self.keyboard_R_command = 0
        self.driver_L_command = 0
        self.driver_R_command = 0

    def signals_subscriber_init(self):
        rospy.Subscriber("keyboard_L_command", Int8, self.keyboard_L_command_callback)
        rospy.Subscriber("keyboard_R_command", Int8, self.keyboard_R_command_callback)

    def signals_publisher_init(self):
        self.pub_L_command = rospy.Publisher('driver_L_command', Int8, queue_size=10)
        self.pub_R_command = rospy.Publisher('driver_R_command', Int8, queue_size=10)

    def main_loop(self):
        self.signals_subscriber_init()
        self.signals_publisher_init()

        while not rospy.is_shutdown():
            self.rate.sleep()

            if self.driver_L_command < self.keyboard_L_command:
                self.driver_L_command += 1;
            elif self.driver_L_command > self.keyboard_L_command:
                self.driver_L_command -= 1;

            if self.driver_R_command < self.keyboard_R_command:
                self.driver_R_command += 1;
            elif self.driver_R_command > self.keyboard_R_command:
                self.driver_R_command -= 1;

            self.pub_L_command.publish(int(self.driver_L_command))
            self.pub_R_command.publish(int(self.driver_R_command))

            print(self.driver_L_command, self.driver_R_command)
        
    def keyboard_L_command_callback(self, message):
        self.keyboard_L_command = message.data

    def keyboard_R_command_callback(self, message):
        self.keyboard_R_command = message.data

# ======================================================================================================================
def driver_controller():
    driver_controller = DriverController().main_loop()

if __name__ == '__main__':
    try:
        driver_controller()
    except rospy.ROSInterruptException:
        pass
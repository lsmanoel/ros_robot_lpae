#!/usr/bin/env python
from __future__ import print_function
import rospy
from std_msgs.msg import Int8
from std_msgs.msg import Bool
from geometry_msgs.msg import Point32

class Controller(object):
    """docstring for Controller"""
    def __init__(self):
        rospy.init_node('controller_py', anonymous=True)

        self.rate = rospy.Rate(40)#Hz

        self.ctrl = 0
        self.power_ref = 0
        self.power_dif = 0
        self.power_on_L = False
        self.power_on_R = False

        self.driver_L = 0
        self.driver_R = 0

    def signals_publisher_init(self):
        self.pub_motor_power_L = rospy.Publisher('motor_power_L', Int8, queue_size=10)
        self.pub_motor_power_R = rospy.Publisher('motor_power_R', Int8, queue_size=10)

    def signals_subscriber_init(self):
        rospy.Subscriber("ctrl", Int8, self.ctrl_callback)
        rospy.Subscriber("power_ref", Int8, self.power_ref_callback)
        rospy.Subscriber("power_dif", Int8, self.power_dif_callback)    
        rospy.Subscriber("svm_ifsc_detector_loc", Point32, self.svm_ifsc_detector_loc_callback)

    def ctrl_callback(self, message):
        self.ctrl = message.data

    def power_ref_callback(self, message):
        if self.ctrl == 1 or self.ctrl == 3:
            self.power_on_L = True
            self.power_on_R = True
            self.power_ref = message.data

    def power_dif_callback(self, message):
        if self.ctrl == 2 or self.ctrl == 3 or self.ctrl == 4 or self.ctrl == 8:
            self.power_on_L = True
            self.power_on_R = True
            self.power_dif = message.data

    def svm_ifsc_detector_loc_callback(self, message):
        if self.ctrl == 0:
            self.power_on_L = True
            self.power_on_R = True
            self.power_dif = (message.x-320)//4
            self.power_ref = (message.x-240)//4

    def main_loop(self):
        self.signals_subscriber_init()
        self.signals_publisher_init()

        while not rospy.is_shutdown():
            self.power_on_L = False
            self.power_on_R = False

            self.rate.sleep() 

            if self.power_on_L is True:
                if self.driver_L < self.power_ref + self.power_dif:
                    self.driver_L += 1
                    if self.driver_L > 126:
                        self.driver_L = 126
                if self.driver_L > self.power_ref + self.power_dif:
                    self.driver_L -= 1
                    if self.driver_L < -126:
                        self.driver_L = -126
            else:
                self.driver_L = 0

            if self.power_on_R is True:
                if self.driver_R < self.power_ref - self.power_dif:
                    self.driver_R += 1
                    if self.driver_R > 126:
                        self.driver_R = 126
                if self.driver_R > self.power_ref - self.power_dif:
                    self.driver_R -= 1
                    if self.driver_R < -126:
                        self.driver_R = -126
            else:
                self.driver_R = 0

            self.pub_motor_power_L.publish(int(self.driver_L))
            self.pub_motor_power_R.publish(int(self.driver_R))

            print(self.power_on_L, self.driver_L, self.driver_R, self.power_on_R)


# ======================================================================================================================
def controller():
    controller = Controller().main_loop()

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass

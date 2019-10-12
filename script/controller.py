#!/usr/bin/env python
from __future__ import print_function
import rospy
from std_msgs.msg import Int8
from std_msgs.msg import Bool

class Controller(object):
    """docstring for Controller"""
    def __init__(self):
        rospy.init_node('Controller', anonymous=True)

        self.rate = rospy.Rate(127)#Hz

        self.power_ref = 0
        self.power_dif = 0
        self.power_on_L = True
        self.power_on_R = True

        self.driver_L = 0
        self.driver_R = 0

    def signals_publisher_init(self):
        self.pub_motor_power_L = rospy.Publisher('motor_power_L', Int8, queue_size=10)
        self.pub_motor_power_R = rospy.Publisher('motor_power_R', Int8, queue_size=10)

    def signals_subscriber_init(self):
        rospy.Subscriber("power_ref", Int8, self.power_ref_callback)
        rospy.Subscriber("power_dif", Int8, self.power_dif_callback)    
        rospy.Subscriber("power_on_L", Bool, self.power_on_L_callback)
        rospy.Subscriber("power_on_R", Bool, self.power_on_R_callback)

    def power_ref_callback(self, message):
        self.power_ref = message.data

    def power_dif_callback(self, message):
        self.power_dif = message.data

    def power_on_L_callback(self, message):
        self.power_on_L = message.data

    def power_on_R_callback(self, message):
        self.power_on_R = message.data

    def main_loop(self):
        self.signals_subscriber_init()
        self.signals_publisher_init()

        while not rospy.is_shutdown():
            self.rate.sleep()

            if self.power_on_L is True:
                if self.driver_L < self.power_ref + self.power_dif:
                    self.driver_L += 1
                    if self.driver_L > 127:
                        self.driver_L = 127
                if self.driver_L > self.power_ref + self.power_dif:
                    self.driver_L -= 1
                    if self.driver_L < -127:
                        self.driver_L = -127
            else:
                self.driver_L = 0

            if self.power_on_R is True:
                if self.driver_R < self.power_ref - self.power_dif:
                    self.driver_R += 1
                    if self.driver_R > 127:
                        self.driver_R = 127
                if self.driver_R > self.power_ref - self.power_dif:
                    self.driver_R -= 1
                    if self.driver_R < -127:
                        self.driver_R = -127
            else:
                self.driver_R = 0

            self.pub_driver_L.publish(int(self.driver_L))
            self.pub_driver_R.publish(int(self.driver_R))

            print(self.driver_L, self.driver_R)


# ======================================================================================================================
def controller():
    controller = Controller().main_loop()

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass

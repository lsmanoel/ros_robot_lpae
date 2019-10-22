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
        self.power_ref_ctrl = 0
        self.power_dif_ctrl = 0
        
        self.svm = 0
        self.power_ref_svm = 0
        self.power_dif_svm = 0

        self.power_on_L = True
        self.power_on_R = True

        self.driver_L = 0
        self.driver_R = 0

    def signals_publisher_init(self):
        self.pub_motor_power_L = rospy.Publisher('motor_power_L', Int8, queue_size=10)
        self.pub_motor_power_R = rospy.Publisher('motor_power_R', Int8, queue_size=10)

    def signals_subscriber_init(self):
        rospy.Subscriber("ctrl", Int8, self.ctrl_callback)
        rospy.Subscriber("power_ref_ctrl", Int8, self.power_ref_ctrl_callback)
        rospy.Subscriber("power_dif_ctrl", Int8, self.power_dif_ctrl_callback)  

        rospy.Subscriber("svm", Int8, self.svm_callback)
        rospy.Subscriber("svm_ref_ctrl", Int8, self.power_ref_svm_callback)
        rospy.Subscriber("svm_dif_ctrl", Int8, self.power_dif_svm_callback) 

    def ctrl_callback(self, message):
        self.ctrl = message.data

    def power_ref_ctrl_callback(self, message):
        self.power_ref_ctrl = message.data

    def power_dif_ctrl_callback(self, message):
        self.power_dif_ctrl = message.data

    def svm_callback(self, message):
        self.svm = message.data

    def power_ref_svm_callback(self, message):
        self.power_ref_svm = message.data

    def power_dif_svm_callback(self, message):
        self.power_dif_svm = message.data

    def main_loop(self):
        self.signals_subscriber_init()
        self.signals_publisher_init()
        
        tn1 = rospy.get_rostime()
        while not rospy.is_shutdown():
            self.rate.sleep() 
            t0 = rospy.get_rostime()
            dt = t0.nsecs - tn1.nsecs              
            #print('t0:', t0.secs, t0.nsecs, 'dt:',  dt)
            tn1 = t0
            
            self.power_ref = 0 
            self.power_dif = 0

            if self.svm > 0:
                self.power_ref = self.power_ref_svm
                self.power_dif = self.power_dif_svm                   

            if self.ctrl > 0:
                self.power_ref = self.power_ref_ctrl
                self.power_dif = self.power_dif_ctrl           

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
                self.power_ref = 0
                self.power_dif = 0

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
                self.power_ref = 0
                self.power_dif = 0

            print(self.driver_L, self.driver_R)
            self.pub_motor_power_L.publish(int(self.driver_L))
            self.pub_motor_power_R.publish(int(self.driver_R))

            # print(self.power_on_L, self.driver_L, self.driver_R, self.power_on_R)


# ======================================================================================================================
def controller():
    controller = Controller().main_loop()

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass

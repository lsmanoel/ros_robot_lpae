#!/usr/bin/env python
from __future__ import print_function
import rospy
from std_msgs.msg import Int8
from std_msgs.msg import UInt8
from std_msgs.msg import Bool
import numpy as np
from geometry_msgs.msg import Point32

class Controller(object):
    """docstring for Controller"""
    def __init__(self, name=None):

        if name is None:
            self.name='radial_closed_loop'
        else:
            self.name=name

        rospy.init_node(self.name, anonymous=True)

        self.rate = rospy.Rate(40)#Hz

        self.ctrl_mode = 0
        self.power_ref = 0
        self.power_dif = 0
        self.power_comp = 0
        self.feedback_L = 0
        self.feedback_R = 0

        self.ctrl_mode_input = 0
        self.power_ref_input = 0
        self.power_dif_input = 0 
        self.power_comp_input = 0
        self.feedback_L_input = 0
        self.feedback_R_input = 0

        self.spin_L = True
        self.spin_R = True         
        self.power_L = 0
        self.power_R = 0

        self.d_L = 0
        self.d_R = 0
        self.d_C = 0
        self.d_L_input = 0
        self.d_R_input = 0
        self.d_C_input = 0

    def signals_publisher_init(self):
        self.pub_power_L = rospy.Publisher('/power_L', UInt8, queue_size=10)
        self.pub_power_R = rospy.Publisher('/power_R', UInt8, queue_size=10)
        self.pub_spin_L = rospy.Publisher('/spin_L', Bool, queue_size=10)
        self.pub_spin_R = rospy.Publisher('/spin_R', Bool, queue_size=10)

    def signals_subscriber_init(self):
        rospy.Subscriber("/ctrl_mode", UInt8, self.ctrl_mode_callback)
        rospy.Subscriber("/power_ref", Int8, self.power_ref_callback)
        rospy.Subscriber("/power_dif", Int8, self.power_dif_callback)
        rospy.Subscriber("/power_comp", Int8, self.power_comp_callback)
        rospy.Subscriber("/feedback_L", UInt8, self.feedback_L_callback)
        rospy.Subscriber("/feedback_R", UInt8, self.feedback_R_callback)   
        rospy.Subscriber("/d_L", UInt8, self.d_L_callback)
        rospy.Subscriber("/d_R", UInt8, self.d_R_callback)  
        rospy.Subscriber("/d_C", UInt8, self.d_C_callback)  

    def ctrl_mode_callback(self, message):
        self.ctrl_mode_input = message.data

    def power_ref_callback(self, message):
        self.power_ref_input = message.data

    def power_dif_callback(self, message):
        self.power_dif_input = message.data

    def power_comp_callback(self, message):
        self.power_comp_input = message.data

    def feedback_L_callback(self, message):
        self.feedback_input = message.data

    def feedback_R_callback(self, message):
        self.feedback_input = message.data

    def d_L_callback(self, message):
        self.d_L_input = message.data

    def d_R_callback(self, message):
        self.d_R_input = message.data

    def d_C_callback(self, message):
        self.d_C_input = message.data

    def main_loop(self):
        self.signals_subscriber_init()
        self.signals_publisher_init()
        
        while not rospy.is_shutdown():
            self.rate.sleep() 
            self.ctrl_mode = self.ctrl_mode_input
            self.power_comp = self.power_comp_input
            self.d_L = self.d_L_input
            self.d_R = self.d_R_input
            self.d_C = self.d_C_input
            self.power_L = 0 
            self.power_R = 0               

            # -------------------------------------------------------------------------
            if self.ctrl_mode == 1:# A
                self.power_ref = 3*self.power_ref_input
                dynamic_dutycycle = True
            elif self.ctrl_mode == 4:# C
                self.power_dif = 3*self.power_dif_input
                dynamic_dutycycle = True           
            elif self.ctrl_mode == 5:# A and C
                self.power_ref = 3*self.power_ref_input
                self.power_dif = 3*self.power_dif_input
                dynamic_dutycycle = True  
            elif self.ctrl_mode == 2:# B
                self.power_L = 192
                self.power_R = 0
                dynamic_dutycycle = False 
            elif self.ctrl_mode == 8:# D
                self.power_L = 0
                self.power_R = 192
                dynamic_dutycycle = False 
            elif self.ctrl_mode == 10:# B and D
                self.power_L = 192
                self.power_R = 192
                dynamic_dutycycle = False
            elif self.ctrl_mode == 3:# A and B
                self.power_L = -192
                self.power_R = 0
                dynamic_dutycycle = False 
            elif self.ctrl_mode == 12:# C and D
                self.power_L = 0
                self.power_R = -192
                dynamic_dutycycle = False  
            elif self.ctrl_mode == 15:# A and B and C and D
                self.power_L = -192
                self.power_R = -192
                dynamic_dutycycle = False
            else:
                dynamic_dutycycle = False 
            
            d_dif = self.d_L - self.d_R
            # -------------------------------------------------------------------------
            if dynamic_dutycycle is True:
                self.power_L = self.power_ref + self.power_dif + self.power_comp
                self.power_R = self.power_ref - self.power_dif - self.power_comp

            if self.power_L > 255:
                self.power_L = 255
            if self.power_L < -255:
                self.power_L = -255

            if self.power_L<0:
                self.power_L = np.uint8(-1*self.power_L)
                self.spin_L = False
            else:
                self.power_L = np.uint8(self.power_L)
                self.spin_L = True

            self.power_L = self.power_L - np.uint8(self.feedback_L)

            if self.power_R > 255:
                self.power_R = 255
            if self.power_R < -255:
                self.power_R = -255

            if self.power_R<0:
                self.power_R = np.uint8(-1*self.power_R)
                self.spin_R = False
            else:
                self.power_R = np.uint8(self.power_R)
                self.spin_R = True

            self.power_R = self.power_R - np.uint8(self.feedback_R)

            #if self.d_C > 215:
            #    print('break power!!!!')
            #    self.power_L = 0
            #    self.power_R = 0

            print('d_dif = self.d_L - self.d_R:', d_dif, self.d_L, self.d_R)

            print(self.power_L, self.power_R)
            self.pub_power_L.publish(self.power_L)
            self.pub_spin_L.publish(self.spin_L)
            self.pub_power_R.publish(self.power_R)
            self.pub_spin_R.publish(self.spin_R)


# ======================================================================================================================
def controller():
    controller = Controller().main_loop()

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass

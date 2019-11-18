#!/usr/bin/env python
from __future__ import print_function
import rospy
from std_msgs.msg import Int8
from std_msgs.msg import UInt8
from std_msgs.msg import Bool
import numpy as np
from geometry_msgs.msg import Point32
from controller import Controller

class RadialCLosedLoop(Controller):
    """docstring for Controller"""
    def __init__(self, 
                 name=None):

        if name is None:
            name='radial_closed_loop'

        super(RadialCLosedLoop, self).__init__(name=name)

    def main_loop(self):
        self.signals_subscriber_init()
        self.signals_publisher_init()
        
        while not rospy.is_shutdown():
            self.rate.sleep() 

            self.d_L = self.d_L_input
            self.d_R = self.d_R_input
            self.d_C = self.d_C_input
            
            d_dif = 5*(self.d_L - self.d_R)
            dynamic_dutycycle = True

            self.power_L = 100
            print("!!!")

            # -------------------------------------------------------------------------
            if dynamic_dutycycle is True:
                self.power_L = - d_dif + self.power_comp
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

                self.power_R = d_dif - self.power_comp
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

            print('d_dif = self.d_L - self.d_R:', d_dif, self.d_L, self.d_R)

            print(self.power_L, self.power_R)
            self.pub_power_L.publish(self.power_L)
            self.pub_spin_L.publish(self.spin_L)
            self.pub_power_R.publish(self.power_R)
            self.pub_spin_R.publish(self.spin_R)



# ======================================================================================================================
def radial_closed_loop():
    radial_closed_loop = RadialCLosedLoop().main_loop()

if __name__ == '__main__':
    try:
        radial_closed_loop()
    except rospy.ROSInterruptException:
        pass

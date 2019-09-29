#!/usr/bin/env python
from __future__ import print_function
import rospy
from std_msgs.msg import Int8
from std_msgs.msg import Bool

class ControladorDeMotor(object):
    """docstring for ControladorDeMotor"""
    def __init__(self):
        rospy.init_node('ControladorDeMotor', anonymous=True)

        self.rate = rospy.Rate(120)#Hz

        self.keyboard_L_command = 0
        self.keyboard_R_command = 0
        self.driver_L_command = 0
        self.driver_R_command = 0
        self.break_command = False


    def signals_publisher_init(self):
        self.pub_L_command = rospy.Publisher('driver_L_command', Int8, queue_size=10)
        self.pub_R_command = rospy.Publisher('driver_R_command', Int8, queue_size=10)

    def signals_subscriber_init(self):
        rospy.Subscriber("keyboard_L_command", Int8, self.keyboard_L_command_callback)
        rospy.Subscriber("keyboard_R_command", Int8, self.keyboard_R_command_callback)
        rospy.Subscriber("break_command", Bool, self.break_command_callback)
        
    def keyboard_L_command_callback(self, message):
        self.keyboard_L_command = message.data

    def keyboard_R_command_callback(self, message):
        self.keyboard_R_command = message.data

    def break_command_callback(self, message):
        self.break_command = message.data

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

            if self.break_command is True:
                self.driver_L_command = 0
                self.driver_R_command = 0

            self.pub_L_command.publish(int(self.driver_L_command))
            self.pub_R_command.publish(int(self.driver_R_command))

            print(self.driver_L_command, self.driver_R_command)


# ======================================================================================================================
def controlador_de_motor():
    controlador_de_motor = ControladorDeMotor().main_loop()

if __name__ == '__main__':
    try:
        controlador_de_motor()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

class ProcessadorDeDecisao(object):
    """docstring for ProcessadorDeDecisao"""
    def __init__(self):
        rospy.init_node('ProcessadorDeDecisao', anonymous=True)

    def signals_publisher_init(self, rostopic="break_command"):
        self.pub_signal = rospy.Publisher(rostopic, Bool, queue_size=10)

    def signals_subscriber_init(self):
        rospy.Subscriber("break_command", Bool, self.break_command_callback)

    def break_command_callback(self, message):
        self.break_command = message.data
        self.pub_signal.publish(self.break_command)

    def main_loop(self):
        self.signals_subscriber_init()
        self.signals_publisher_init()

        rospy.spin()
            

# ======================================================================================================================
def processador_de_decisao():
    processador_de_decisao = ProcessadorDeDecisao().main_loop()

if __name__ == '__main__':
    try:
        processador_de_decisao()
    except rospy.ROSInterruptException:
        pass
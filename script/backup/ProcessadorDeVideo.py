#!/usr/bin/env python
import numpy as np
import rospy
from sensor_msgs.msg import Image

class ProcessadorDeVideo(object):
    """docstring for VizualizadorDeVideo"""
    def __init__(self):
        rospy.init_node('ProcessadorDeVideo', anonymous=True)

        self.rate = rospy.Rate(120)#Hz
        self.frame_size = (320, 240)
        self.bridge = CvBridge()
        self.input_frame = np.zeros(self.frame_size);
        self.output_frame = np.zeros(self.frame_size);

    def signals_subscriber_init(self, rostopic="ProcessadorDeVideo_entrada"):
        rospy.Subscriber(rostopic, Image, self.input_callback)

    def signals_publisher_init(self, rostopic="ProcessadorDeVideo_saida"):
        self.pub_signal = rospy.Publisher(rostopic, Image, queue_size=10)

    def input_callback(frame):
        self.input_frame = bridge.imgmsg_to_cv2(frame)

    def main_loop(self):
        self.signals_subscriber_init()
        self.signals_publisher_init()
        
        while not rospy.is_shutdown():
            self.rate.sleep()
            
            self.output_frame = self.input_frame

            pub.publish(bridge.cv2_to_imgmsg(self.output_frame))


# ======================================================================================================================
def processador_de_video():
    processador_de_video = ProcessadorDeVideo().main_loop()

if __name__ == '__main__':
    try:
        vizualizador_de_video()
    except rospy.ROSInterruptException:
        pass
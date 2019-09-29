#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

class VizualizadorDeVideo(object):
    """docstring for VizualizadorDeVideo"""
    def __init__(self):
        rospy.init_node('VizualizadorDeVideo', anonymous=True)
        self.bridge = CvBridge()

    def signals_subscriber_init(self, rostopic="ControladorDeCamera_saida"):
        rospy.Subscriber(rostopic, Image, self.input_callback)

    def input_callback(self, frame):
        cv_frame = self.bridge.imgmsg_to_cv2(frame)
        cv2.imshow("Image window", cv_frame)
        cv2.waitKey(40)

    def main_loop(self):
        self.signals_subscriber_init()
        rospy.spin()


# ======================================================================================================================
def vizualizador_de_video():
    vizualizador_de_video = VizualizadorDeVideo().main_loop()

if __name__ == '__main__':
    try:
        vizualizador_de_video()
    except rospy.ROSInterruptException:
        pass

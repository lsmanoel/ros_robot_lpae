#!/usr/bin/env python
import numpy as np
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

class ControladorDeCamera(object):
    """docstring for ControladorDeCamera"""
    def __init__(self,
                 video_source=0):

        rospy.init_node('ControladorDeCamera', anonymous=True)
        self.rate = rospy.Rate(30)#Hz

        self.video_source = video_source
        self.bridge = CvBridge()

    def signals_publisher_init(self, rostopic="ControladorDeCamera_saida"):
        self.pub_signal = rospy.Publisher(rostopic, Image, queue_size=10)

    def main_loop(self):
        video_capture = cv2.VideoCapture(self.video_source)
        self.signals_publisher_init()

        while not rospy.is_shutdown():
            self.rate.sleep()

#            if not video_capture.grab():
#                print("No more frames")
#                break

            _, frame_input = video_capture.read()
 
            output_frame = cv2.resize(frame_input, (320, 240), interpolation = cv2.INTER_AREA)
            
            cv2.resize()
            self.pub_signal.publish(self.bridge.cv2_to_imgmsg(output_frame))


# ======================================================================================================================
def controlador_de_camera():
    controlador_de_camera = ControladorDeCamera().main_loop()

if __name__ == '__main__':
    try:
        controlador_de_camera()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python2
import os
import time
import random
import numpy as np
import cv2
import roslib
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class EdgeFilter(object):
    def __init__(self,
    			 name=None,
                 input_topic=None,
                 output_topic=None,
                 input_frame=None):

        if name is None:
        	self.name='edge_filter'
        else:
        	self.name=name

        if input_topic is None:
        	self.input_topic = self.name + '_input'
        else: 
        	self.input_topic = input_topic

		if output_topic is None:
			self.output_topic = self.name + '_output'
		else: 
			self.output_topic = output_topic

        rospy.init_node(self.name, anonymous=True)

        self.rate = rospy.Rate(30)#Hz

        self.bridge = CvBridge()
        self.input_frame = input_frame
        self.output_frame = None

    def signals_publisher_init(self, rostopic_name=None):
        if rostopic_name is None:
        	rostopic_name = self.output_topic
        self.pub_output = rospy.Publisher(rostopic_name, Image, queue_size=10)

    def signals_subscriber_init(self, rostopic_name=None):
    	if rostopic_name is None:
    		rostopic_name = self.input_topic 
        rospy.Subscriber(rostopic_name, Image, self.input_frame_callback)

    def input_frame_callback(self, frame):
        self.input_frame = self.bridge.imgmsg_to_cv2(frame)

    def main_loop(self):
        # ------------------------------------------
        while not rospy.is_shutdown():
            self.rate.sleep()
            if self.input_frame is not None:
                frame = self.input_frame.copy()
                # -----------------------------------
                frame = cv2.Canny(frame,100,200)
                # -----------------------------------
                # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                self.output_frame = frame.copy()
                self.pub_output.publish(self.bridge.cv2_to_imgmsg(self.output_frame))
        
        print("break")

# ======================================================================================================================
def edge_filter():
    edge_filter = EdgeFilter(input_topic="/webcam/image_raw")
    edge_filter.signals_publisher_init()
    edge_filter.signals_subscriber_init()
    edge_filter.main_loop()

if __name__ == '__main__':
    try:
        edge_filter()
    except rospy.ROSInterruptException:
        pass

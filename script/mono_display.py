#!/usr/bin/env python
import roslib
import cv2
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

FULL_FRAME_WIDTH = 640;
FULL_FRAME_HEIGHT = 480;
FULL_FRAME_SIZE = (FULL_FRAME_WIDTH, FULL_FRAME_HEIGHT)
FULL_FRAME_CENTER = (FULL_FRAME_WIDTH//2, FULL_FRAME_HEIGHT//2)

VIEW_FRAME_WIDTH = 320
VIEW_FRAME_HEIGHT = 240
VIEW_FRAME_SIZE = (VIEW_FRAME_WIDTH, VIEW_FRAME_HEIGHT)
VIEW_FRAME_CENTER = (VIEW_FRAME_WIDTH//2, VIEW_FRAME_HEIGHT//2)

M_rot_L = cv2.getRotationMatrix2D(FULL_FRAME_CENTER, 270, 1.0)
M_rot_R = cv2.getRotationMatrix2D(FULL_FRAME_CENTER, 90, 1.0)

def chatter_callback(frame):
    cv_frame = bridge.imgmsg_to_cv2(frame)
    cv2.imshow('mono_display', cv_frame)
    cv2.waitKey(5)
 
def mono_display():
    rospy.init_node('mono_display', anonymous=True)
    rospy.Subscriber('mono_vision_mono8', Image, chatter_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    mono_display()
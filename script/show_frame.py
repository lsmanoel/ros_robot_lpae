#!/usr/bin/env python
import numpy as np
import roslib
import cv2
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

def chatter_callback(frame):

    cv_frame = bridge.imgmsg_to_cv2(frame)
    frame_center = tuple(np.array(cv_frame.shape[1::-1]) / 2)
    rot_mat = cv2.getRotationMatrix2D(frame_center, 270, 1.0)
    cv_frame = cv2.warpAffine(cv_frame, rot_mat, cv_frame.shape[1::-1], flags=cv2.INTER_LINEAR)
    cv2.imshow("Image window", cv_frame)
    cv2.waitKey(40)

    
def show_frame():
    
    rospy.init_node('show_frame', anonymous=True)

    rospy.Subscriber("frame_chatter", Image, chatter_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    show_frame()

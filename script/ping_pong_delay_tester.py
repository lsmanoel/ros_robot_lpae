#!/usr/bin/env python
from __future__ import print_function
import sys
import cv2
import rospy
from ros_robot_lpae.srv import return_data, return_dataResponse
from std_msgs.msg import Int8
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import matplotlib.pyplot as plt

class PingPongDelayTester(object):
    """docstring for ping_pong_delay_tester"""
    def __init__(self, name='ping_pong_delay_tester', mode='ping_pong'):
        rospy.init_node(name, anonymous=True)
        self.name = name
        self.bridge = CvBridge()

        self.mode = mode

        self.time_ping_buffer_size = 1000
        self._time_ping = []
        self.time_ping = 0

        self.time_pong_buffer_size = 1000
        self._time_pong = []
        self.time_pong = 0

        self.time_bypass_buffer_size = 1000
        self._time_bypass = []
        self.time_bypass = 0

        self.input_frame = None
        self.output_frame = None
        
    @staticmethod
    def imshow(frame=None, name='frame'):
        if frame is not None:
            cv2.imshow(name, frame)
            cv2.waitKey(5)

    # ----------------------------------------------------------------------------------------
    @property
    def time_ping(self):
        return self._time_ping[-1]

    @time_ping.setter
    def time_ping(self, value):
        if len(self._time_ping) > self.time_ping_buffer_size:
            self._time_ping.remove(0)
        self._time_ping.append(value)

    @property
    def time_pong(self):
        return self._time_pong[-1]

    @time_pong.setter
    def time_pong(self, value):
        if len(self._time_pong) > self.time_pong_buffer_size:
            self._time_pong.remove(0)
        self._time_pong.append(value)

    @property
    def time_bypass(self):
        return self._time_bypass[-1]

    @time_bypass.setter
    def time_bypass(self, value):
        if len(self._time_bypass) > self.time_bypass_buffer_size:
            self._time_bypass.remove(0)
        self._time_bypass.append(value)

    # ----------------------------------------------------------------------------------------
    # rosservices
    def time_ping_data_service_init(self):
        self._time_ping_data_service = rospy.Service(self.name + '_time_ping_data_service', return_data, self.time_ping_data_service)

    def time_ping_data_service(self, msg):
        return return_dataResponse(self._time_ping)

    def time_pong_data_service_init(self):
        self._time_pong_data_service = rospy.Service(self.name + '_time_pong_data_service', return_data, self.time_pong_data_service)

    def time_pong_data_service(self, msg):
        return return_dataResponse(self._time_pong)

    def time_bypass_data_service_init(self):
        self._time_pong_data_service = rospy.Service(self.name + '_time_bypass_data_service', return_data, self.time_bypass_data_service)

    def time_bypass_data_service(self, msg):
        return return_dataResponse(self._time_bypass)

    # ----------------------------------------------------------------------------------------
    # rostopics
    def bypass_image_callback(self, frame):
        frame = self.bridge.imgmsg_to_cv2(frame)
        self.time_bypass = rospy.get_rostime().nsecs
        self.pub.publish(self.bridge.cv2_to_imgmsg(frame))
        # self.imshow(frame, name="bypass_frame")

    def pong_image_callback(self, frame):
        self.input_frame = self.bridge.imgmsg_to_cv2(frame)
        self.time_pong = rospy.get_rostime().nsecs
        # self.imshow(self.input_frame, name="pong_frame")

    def signals_publisher_init(self, 
                               topic=None):
        if topic is None:
            topic = self.name + '_output'
        self.pub = rospy.Publisher(topic, Image, queue_size=10)    

    def signals_subscriber_init(self, 
                                topic=None, 
                                callback=None):
        if topic is None:
            topic = self.name + '_output'
        if callback is None:
            callback=self.pong_image_callback           
        rospy.Subscriber(topic, Image, callback)

    def ping_image(self):
        if self.pub is not None:
            if self.output_frame is not None:
                self.pub.publish(self.bridge.cv2_to_imgmsg(self.output_frame))
                self.time_ping = rospy.get_rostime().nsecs
                # self.imshow(self.output_frame, name="pong_frame")
        else:
            print("publisher output not started!")

    # ----------------------------------------------------------------------------------------
    # Main Loop
    def main_loop(self):
        if self.mode == 'bypass':
            rospy.spin()

        elif self.mode == 'plotter':
            rospy.wait_for_service('bypass_time_bypass_data_service')
            bypass_time_bypass_data_service = rospy.ServiceProxy('bypass_time_bypass_data_service', return_data)
            bypass_time = bypass_time_bypass_data_service()
            #print(bypass_time.data)
            plt.plot(bypass_time.data)
            plt.show()
        else:
            self.rate = rospy.Rate(20)#Hz
            video_capture = cv2.VideoCapture(0)
            while not rospy.is_shutdown():
                for i in range(1000):
                    if rospy.is_shutdown():
                        break 
                    self.rate.sleep()
                    _, self.output_frame = video_capture.read()
                    self.ping_image()

                break
            video_capture.release()
            cv2.destroyAllWindows()

# ======================================================================================================================
def ping_pong_delay_tester():   
    if len(sys.argv)>=2:
        mode = sys.argv[1]
        if len(sys.argv)>=3:
            opition = sys.argv[2]
    else:
        mode = None

    if mode == 'plotter':
        plotter = PingPongDelayTester(name = 'plotter', mode='plotter')
        plotter.main_loop()
    elif mode == 'bypass':
        bypass = PingPongDelayTester(name='bypass', mode='bypass')
        bypass.signals_subscriber_init(topic='ping_pong_output', callback=bypass.bypass_image_callback)
        bypass.signals_publisher_init()
        bypass.time_bypass_data_service_init()
        bypass.main_loop()
    else:  
        ping_pong = PingPongDelayTester(name='ping_pong', mode=mode)
        ping_pong.signals_subscriber_init(topic='bypass_output', callback=ping_pong.pong_image_callback)
        ping_pong.signals_publisher_init()
        ping_pong.time_ping_data_service_init()
        ping_pong.time_pong_data_service_init()
        ping_pong.main_loop()

if __name__ == '__main__':
    try:
        ping_pong_delay_tester()
    except rospy.ROSInterruptException:
        pass

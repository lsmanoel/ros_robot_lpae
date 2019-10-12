#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::imshow("mono_display_mono8", cv_bridge::toCvShare(msg, "mono8")->image);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mono_display_mono8_cpp");
  ros::NodeHandle nh;
  cv::namedWindow("mono_display_mono8");
  // cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("mono_vision_mono8", 1, imageCallback);
  ros::spin();
  cv::destroyWindow("mono_display_mono8");
}
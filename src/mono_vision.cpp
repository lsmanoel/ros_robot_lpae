#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream> // for converting the command line parameter to integer

int main(int argc, char** argv)
{
  ROS_INFO("mono_vision_cpp start...");
  // Check if video source has been passed as a parameter
  if(argv[1] == NULL) 
    return 1;

  ros::init(argc, argv, "mono_vision_cpp");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("frame_chatter", 1);

  // Convert the passed as command line parameter index for the video device to an integer
  std::istringstream video_sourceCmd(argv[1]);
  int video_source;
  // Check if it is indeed a number
  if(!(video_sourceCmd >> video_source)) 
    return 1;

  cv::VideoCapture cap(video_source);
  // Check if video device can be opened with the given index
  if(!cap.isOpened()) 
    return 1;
  cv::Mat input_frame;
  cv::Mat ping_frame, pong_frame;
  sensor_msgs::ImagePtr msg;

  ros::Rate loop_rate(5);
  while (nh.ok()) {
    cap >> input_frame;  
    // Check if grabbed frame is actually full with some content
    if(!input_frame.empty()) {
      cv::cvtColor(input_frame, ping_frame, CV_BGR2GRAY);
      msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", ping_frame).toImageMsg();
      pub.publish(msg);
      cv::waitKey(1);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}
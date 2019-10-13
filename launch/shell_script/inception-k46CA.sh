#!/bin/bash
rosrun ros_robot_lpae mono_vision_mono8 0 0 &
rosrun ros_robot_lpae mono_display.py -i mono_vision_mono8

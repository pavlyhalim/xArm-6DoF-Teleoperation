#!/bin/bash

#cd ~/catkin_ws/
#catkin_make
#read
roslaunch openni_launch openni.launch &
rosrun openni_tracker openni_tracker &
rosrun telerobot arm_tracker &
rosrun telerobot grabber

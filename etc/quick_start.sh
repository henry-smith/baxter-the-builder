#!/bin/bash
rosrun baxter_tools enable_robot.py -e
rosrun baxter_tools tuck_arms.py -u
rosrun baxter_tools camera_control.py -c right_hand_camera
rosrun baxter_tools camera_control.py -o left_hand_camera -r 1280x800
roslaunch baxter_the_builder left_btb_begin.launch 



#!/usr/bin/env python
"""
EE106b Final Project
Authors: Henry Smith Andrew Chan
"""
import copy
import rospy
import sys

import baxter_interface
import moveit_commander
from baxter_core_msgs.msg import EndpointState
from moveit_msgs.msg import OrientationConstraint, Constraints
from geometry_msgs.msg import PoseStamped
from baxter_interface import gripper as baxter_gripper
from std_msgs.msg import (
    UInt16,
)
from ee106b_lab1.srv import *

import IPython
import tf
import time
import pickle
import matplotlib.pyplot as plt

with open('test3.pickle', 'rb') as f:
    my_list = pickle.load(f)

with open('test2.pickle', 'rb') as f:
    my_list2 = pickle.load(f)

poses = my_list[0]
x = []
y = []
for pose in poses:
	x.append(pose.position.x)
	y.append(pose.position.y)

poses2 = my_list2[0]
x2 = []
y2 = []
for pose in poses2:
	x2.append(pose.position.x)
	y2.append(pose.position.y)

#plt.plot(x,y,x2,y2)
plt.plot(x,y)
plt.show()
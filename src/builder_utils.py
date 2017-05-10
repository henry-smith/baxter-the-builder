#!/usr/bin/env python
"""
Baxter_the_builder regrasping node
Author: Henry Smith & Andrew Chan
"""
import numpy as np
import time

import rospy
from baxter_interface import gripper as baxter_gripper
import baxter_interface
import actionlib
import tf

import transformations as trans
from ee106b_lab1.srv import *
from std_msgs.msg import (
    UInt16,
)
import copy

 
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header
 
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

PI = np.pi


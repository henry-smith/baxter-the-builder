#!/usr/bin/env python
"""
Baxter_the_builder main run script
Author: Henry Smith & Andrew Chan
"""
import numpy as np
import time

import rospy
import moveit_commander
from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal, MoveGroupFeedback, MoveGroupResult, JointConstraint, OrientationConstraint, Constraints
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from baxter_interface import gripper as baxter_gripper
import actionlib
import tf

from ee106b_lab1.srv import *


if __name__ == '__main__':
    rospy.init_node('baxter_builds')
    # For recording Trajectory
    # rospy.wait_for_service('endpoint_info')
    # endpointSave = rospy.ServiceProxy('endpoint_save', endpoint_save)
    # endpointInfo = rospy.ServiceProxy('endpoint_info', endpoint_service)
    # endpointLoad = rospy.ServiceProxy('endpoint_load', endpoint_load)





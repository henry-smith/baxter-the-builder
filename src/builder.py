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

import regrasping
import grasp_utils as utils

PI = np.pi

class Builder:
    def __init__(self, limb, gripper):
        rospy.init_node('buliding' + limb.name)

        # Set up gripper
        self.gripper = gripper

        # Set up limb
        self.limb = limb
        self.limb_name = limb.name

        # Set up tf stuff
        self.parent_frame = 'base'
        self.listener = tf.TransformListener()

        # Set up IK solver
        self.ns = "ExternalTools/" + self.limb_name + "/PositionKinematicsNode/IKService"
        self.iksvc = rospy.ServiceProxy(self.ns, SolvePositionIK)

        self.rate = rospy.Rate(100)
        rospy.sleep(1.0)

    def stack_bricks(self, stacking_block, base_block):
        if not self.listener.frameExists(base_block):
            print 'block not found'
            exit(0)
        utils.align_with_top(self.limb, stacking_block)

        # Moves to above next brick
        self.listener.waitForTransform(self.parent_frame, base_block, rospy.Time(), rospy.Duration(4.0))
        t = self.listener.getLatestCommonTime(self.parent_frame, base_block)
        b_pos, b_quat = self.listener.lookupTransform(self.parent_frame, base_block, t)
        new_quat = np.array([1,0,0,0])
        new_pos = b_pos + np.array([0,0,.2])
        utils.move_to_point(self.limb, new_pos, new_quat)

        # Rotates until aligned
        velocities = self.limb.joint_velocities()
        cmd = copy.deepcopy(velocities)
        for joint in cmd.keys():
            cmd[joint] = 0
        cmd['right_w2'] = .5
        while not rospy.is_shutdown():
            self.limb.set_joint_velocities(cmd)
            t = self.listener.getLatestCommonTime(self.parent_frame, base_block)
            position, quaternion = self.listener.lookupTransform(base_block, 'right_gripper', t)
            rot = np.array(trans.quaternion_matrix(quaternion)[:3,:3])
            count = 0
            for i in range(3):
                for j in range(3):
                    if abs(abs(rot[i,j])-1) < .03:
                        count += 1
            if count >= 3:
                break

        # Stacks 'em
        utils.move_to_point(self.limb, b_pos + np.array([0,0,.055]), quaternion)
        print("Move Succesful")
        self.gripper.open()

        utils.move_to_point(self.limb, b_pos + np.array([0,0,.09]), quaternion)
        return b_pos

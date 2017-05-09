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

# Set up IK solver
ns = "ExternalTools/" + 'right' + "/PositionKinematicsNode/IKService"
iksvc = rospy.ServiceProxy(ns, SolvePositionIK)

parent_frame = 'base'
listener = tf.TransformListener()

def move_to_point(limb, position, orientation, threshold=0.008726646, timeout = 15.0):
    goal = PoseStamped()
    goal.header = Header(stamp=rospy.Time.now(), frame_id='base')

    print "\nMoving to:"
    print(position)
    print(orientation)
    #x, y, and z position
    goal.pose.position.x =  position[0]
    goal.pose.position.y =  position[1]
    goal.pose.position.z =  position[2]
    
    #Orientation as a quaternion
    goal.pose.orientation.x = orientation[0] 
    goal.pose.orientation.y = orientation[1]
    goal.pose.orientation.z = orientation[2]
    goal.pose.orientation.w = orientation[3]

    # Solves IK
    ikreq = SolvePositionIKRequest()
    ikreq.pose_stamp.append(goal)
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1

    if (resp.isValid[0]):
        print("SUCCESS - Valid Joint Solution Found:")
        # Format solution into limb compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        # Moves to solution
        limb.move_to_joint_positions(limb_joints, threshold=threshold, timeout=timeout)
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")

def align_with_top(limb, block, realign_with_base = False):
    if not listener.frameExists(block):
        print block + ' not found'
        exit(0)

    # Finds block position and orientation
    listener.waitForTransform(parent_frame, block, rospy.Time(), rospy.Duration(4.0))
    t = listener.getLatestCommonTime(parent_frame, block)
    b_pos, b_quat = listener.lookupTransform(parent_frame, block, t)
    if abs(1-abs(trans.quaternion_matrix(b_quat)[0,0])) < .01:
        on_side = False
    else: 
        on_side = True
    # Throws away original orientation
    new_quat = np.array([1,0,0,0])
    new_pos = b_pos + np.array([0,0,.1])

    # Moves to above brick, then rotates until alignment
    move_to_point(limb, new_pos, new_quat, threshold = .005)
    velocities = limb.joint_velocities()
    cmd = copy.deepcopy(velocities)
    for joint in cmd.keys():
        cmd[joint] = 0
    cmd['right_w2'] = .5
    while not rospy.is_shutdown():
        limb.set_joint_velocities(cmd)
        listener.waitForTransform('base','right_gripper', rospy.Time(), rospy.Duration(4.0))
        t = listener.getLatestCommonTime('base', 'right_gripper')
        position, quaternion = listener.lookupTransform(block, 'right_gripper', t)
        rot = np.array(trans.quaternion_matrix(quaternion)[:3,:3])
        # This is very hacky, forgive me

        if on_side:
            count = 0
            for i in range(2):
                for j in range(1,3):
                    if abs(abs(rot[i,j])-1) < .02:
                        count += 1
            if count >= 2:
                break
        else:
            count = 0
            for i in range(3):
                for j in range(3):
                    if abs(abs(rot[i,j])-1) < .03:
                        count += 1
            if count >= 3:
                break

    # Picks up brick
    t = listener.getLatestCommonTime('base', 'right_gripper')
    position, quaternion = listener.lookupTransform('base', 'right_gripper', t)
    move_to_point(limb, b_pos - np.array([0,0,.02]), quaternion, timeout = 2.0)
    gripper.close()

    # Keeps or throws away current orientation
    if realign_with_base:
        print('realigning')
        move_to_point(limb, b_pos + np.array([0,0,.03]), new_quat)
    else:
        move_to_point(limb, b_pos + np.array([0,0,.05]), quaternion)
    # Returns original position for reference
    return b_pos
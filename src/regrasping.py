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

class Regrasper:
    def __init__(self, limb):
        rospy.init_node('regrasping_' + limb)

        # Set up gripper
        self.gripper = baxter_gripper.Gripper(limb)
        self.gripper.calibrate()
        self.limb = baxter_interface.Limb(limb)

        self.limb_name = limb
        self.parent_frame = 'base'
        self.listener = tf.TransformListener()
        self.rate = rospy.Rate(100)

        self.ns = "ExternalTools/" + self.limb_name + "/PositionKinematicsNode/IKService"
        self.iksvc = rospy.ServiceProxy(self.ns, SolvePositionIK)
        time.sleep(1)

    def move_to_point(self, position, orientation):
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

        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(goal)
        try:
            rospy.wait_for_service(self.ns, 5.0)
            resp = self.iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return 1

        if (resp.isValid[0]):
            print("SUCCESS - Valid Joint Solution Found:")
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        else:
            print("INVALID POSE - No Valid Joint Solution Found.")
        
        self.limb.move_to_joint_positions(limb_joints)


    def reorient_block(self, arm, block, goal):
        # Decide to come from top or side
        pass

    def adjust_yaw(self, block, angle=1.2):
        b_pos = self.align_with_top(block)
        positions = self.limb.joint_angles()
        cmd = copy.deepcopy(positions)
        print cmd
        cmd['right_w2'] += angle
        self.limb.move_to_joint_positions(cmd)
        t = self.listener.getLatestCommonTime(self.parent_frame, 'right_gripper')
        position, quaternion = self.listener.lookupTransform(self.parent_frame, 'right_gripper', t)
        self.move_to_point(b_pos - np.array([0,0,.02]), quaternion)
        self.gripper.open()
        self.move_to_point(position + np.array([0,0,.05]), quaternion)

    def flip(self, block):
        b_pos = self.align_with_top(block)
        quat = [-.245, 0.677,.26,.64]

        self.move_to_point(b_pos + np.array([0,0,.15]), quat)
        t = self.listener.getLatestCommonTime(self.parent_frame, 'right_gripper')
        position, quaternion = self.listener.lookupTransform(self.parent_frame, 'right_gripper', t)
        self.move_to_point(b_pos + np.array([0,0,.05]), quaternion)
        self.gripper.open()


    def align_with_top(self, block):
        if not self.listener.frameExists(self.parent_frame):
            print 'parent frame not found'
            exit(0)
        if not self.listener.frameExists(block):
            print 'block not found'
            exit(0)
        self.listener.waitForTransform(self.parent_frame, block, rospy.Time(), rospy.Duration(4.0))
        t = self.listener.getLatestCommonTime(self.parent_frame, block)

        b_pos, b_quat = self.listener.lookupTransform(self.parent_frame, block, t)
        b_rot = trans.quaternion_matrix(b_quat)
        bx, by, bz = b_rot[:3,0], b_rot[:3,1], b_rot[:3,2]

        new_quat = np.array([1,0,0,0])
        new_pos = b_pos + np.array([0,0,.1])

        self.move_to_point(new_pos, new_quat)
        print("Move Succesful")

        velocities = self.limb.joint_velocities()
        cmd = copy.deepcopy(velocities)
        for joint in cmd.keys():
            cmd[joint] = 0
        cmd['right_w2'] = .5
        while not rospy.is_shutdown():
            self.limb.set_joint_velocities(cmd)
            t = self.listener.getLatestCommonTime(self.parent_frame, block)
            position, quaternion = self.listener.lookupTransform(block, 'right_gripper', t)
            rot = np.array(trans.quaternion_matrix(quaternion)[:3,:3])
            count = 0
            for i in range(3):
                for j in range(3):
                    if abs(abs(rot[i,j])-1) < .03:
                        count += 1
            if count >= 3:
                break
        print('done')

        self.move_to_point(b_pos - np.array([0,0,.02]), quaternion)
        print("Move Succesful")
        self.gripper.close()

        self.move_to_point(b_pos + np.array([0,0,.03]), quaternion)
        return b_pos


if __name__ == '__main__':
    # For recording Trajectory
    # rospy.wait_for_service('endpoint_info')
    # endpointSave = rospy.ServiceProxy('endpoint_save', endpoint_save)
    # endpointInfo = rospy.ServiceProxy('endpoint_info', endpoint_service)
    # endpointLoad = rospy.ServiceProxy('endpoint_load', endpoint_load)

    right_regrasper = Regrasper('right')
    
    right_regrasper.adjust_yaw('block_0')
    right_regrasper.flip('block_3')
    right_regrasper.adjust_yaw('block_0')




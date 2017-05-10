#!/usr/bin/env python
"""
Baxter_the_builder main node
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

from record_trajectory import JointRecorder
from apply_trajectory import map_file

PI = np.pi

FLIP_MOVES = [((1,0),(0,0)),((1,2),(5,0)),((4,2),(5,3)),((4,0),(0,1)),((3,0),(0,2)),((3,2),(5,2)),((2,0),(0,3)),((2,2),(5,1)),((0,0),(3,2)),((0,1),(2,2)),((0,2),(1,2)),((0,3),(4,2)),((5,0),(3,0)),((5,1),(4,0)),((5,2),(1,0)),((5,3),(2,0))]

SPIN_C_MOVES = []
for i in range(6):
    for j in range(4):
        SPIN_C_MOVES.append(((i, (j+1)%4),(i, j)))

SPIN_CC_MOVES = [(move[1], move[0]) for move in SPIN_C_MOVES]

MOVES = [('flip',FLIP_MOVES), ('spin_c',SPIN_C_MOVES), ('spin_cc',SPIN_CC_MOVES)]


class Builder:
    def __init__(self, limb):
        rospy.init_node('regrasping_' + limb)

        # Set up gripper
        self.gripper = baxter_gripper.Gripper(limb)
        self.gripper.calibrate()

        # Set up limb
        self.limb = baxter_interface.Limb(limb)
        self.limb_name = limb

        # Set up tf stuff
        self.parent_frame = 'base'
        self.listener = tf.TransformListener()

        # Set up IK solver
        self.ns = "ExternalTools/" + self.limb_name + "/PositionKinematicsNode/IKService"
        self.iksvc = rospy.ServiceProxy(self.ns, SolvePositionIK)

        self.rate = rospy.Rate(100)
        rospy.sleep(1.0)

    def move_to_point(self, position, orientation, threshold=0.008726646, timeout = 15.0):
        goal = PoseStamped()
        goal.header = Header(stamp=rospy.Time.now(), frame_id='base')

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
            rospy.wait_for_service(self.ns, 5.0)
            resp = self.iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return 1

        if (resp.isValid[0]):
            # Format solution into limb compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            # Moves to solution
            self.limb.move_to_joint_positions(limb_joints, threshold=threshold, timeout=timeout)
        else:
            print("INVALID POSE - No Valid Joint Solution Found.")

    def align_with_top(self, block, realign_with_base = False):
        if not self.listener.frameExists(block):
            print block + ' not found'
            exit(0)

        # Finds block position and orientation
        self.listener.waitForTransform(self.parent_frame, block, rospy.Time(), rospy.Duration(4.0))
        t = self.listener.getLatestCommonTime(self.parent_frame, block)
        b_pos, b_quat = self.listener.lookupTransform(self.parent_frame, block, t)
        if abs(1-abs(trans.quaternion_matrix(b_quat)[0,0])) < .01:
            on_side = False
        else: 
            on_side = True
        # Throws away original orientation
        new_quat = np.array([1,0,0,0])
        new_pos = b_pos + np.array([0,0,.1])

        # Moves to above brick, then rotates until alignment
        self.move_to_point(new_pos, new_quat, threshold = .004)
        velocities = self.limb.joint_velocities()
        cmd = copy.deepcopy(velocities)
        for joint in cmd.keys():
            cmd[joint] = 0
        if self.limb.joint_angle('right_w2') < 0:
            cmd['right_w2'] = .5
        else:
            cmd['right_w2'] = -.5
        while not rospy.is_shutdown():
            self.limb.set_joint_velocities(cmd)
            self.listener.waitForTransform('right_gripper', block, rospy.Time(), rospy.Duration(4.0))
            t = self.listener.getLatestCommonTime('right_gripper', block)
            position, quaternion = self.listener.lookupTransform(block, 'right_gripper', t)
            rot = np.array(trans.quaternion_matrix(quaternion)[:3,:3])
            # This is very hacky, forgive me

            if on_side:
                count = 0
                for i in range(2):
                    for j in range(1,3):
                        if abs(abs(rot[i,j])-1) < .04:
                            count += 1
                if count >= 2:
                    break
            else:
                count = 0
                for i in range(3):
                    for j in range(3):
                        if abs(abs(rot[i,j])-1) < .04:
                            count += 1
                if count >= 3:
                    break

        # Picks up brick
        t = self.listener.getLatestCommonTime('base', 'right_gripper')
        position, quaternion = self.listener.lookupTransform('base', 'right_gripper', t)
        self.move_to_point(b_pos - np.array([0,0,.02]), quaternion, timeout = 2.0)
        self.gripper.close()

        # Keeps or throws away current orientation
        if realign_with_base:
            print('realigning')
            self.move_to_point(b_pos + np.array([0,0,.03]), new_quat)
        else:
            self.move_to_point(b_pos + np.array([0,0,.05]), quaternion)
        # Returns original position for reference
        return b_pos

    def adjust_yaw(self, block, angle=PI/2):
        if not self.listener.frameExists(block):
            print block + ' not found'
            exit(0)

        # Picks up and saves original position
        b_pos = self.align_with_top(block)

        # Rotate by just moving wrist
        positions = self.limb.joint_angles()
        cmd = copy.deepcopy(positions)
        if angle == -PI/2 and positions['right_w2'] < -1.8:
            angle = 3*PI/2
        cmd['right_w2'] += angle
        self.limb.move_to_joint_positions(cmd)

        # Grabs current orientation so we don't lose it
        t = self.listener.getLatestCommonTime(self.parent_frame, 'right_gripper')
        position, quaternion = self.listener.lookupTransform(self.parent_frame, 'right_gripper', t)

        # Moves back to blocks original position at new orientation
        self.move_to_point(b_pos - np.array([0,0,.01]), quaternion)
        self.gripper.open()
        self.move_to_point(position + np.array([0,0,.05]), quaternion)

    def flip(self, block):
        # Grabs from top and realigns with base
        b_pos = self.align_with_top(block, False)

        # Rotates arm upwards
        positions = self.limb.joint_angles()
        cmd = copy.deepcopy(positions)
        if self.limb.joint_angle('right_w1') > 0:
            cmd['right_w1'] -= PI/2
        else:
            cmd['right_w1'] += PI/2
        self.limb.move_to_joint_positions(cmd)


        # Rotates until aligned
        velocities = self.limb.joint_velocities()
        cmd = copy.deepcopy(velocities)
        for joint in cmd.keys():
            cmd[joint] = 0
        cmd['right_w2'] = .5
        while not rospy.is_shutdown():
            self.limb.set_joint_velocities(cmd)
            t = self.listener.getLatestCommonTime(self.parent_frame, 'right_gripper')
            position, quaternion = self.listener.lookupTransform(self.parent_frame, 'right_gripper', t)
            rot = np.array(trans.quaternion_matrix(quaternion)[:3,:3])
            if abs(np.dot(rot[:2,1], rot[:2,2])) < .01:
                break

        # Grabs current orientation
        t = self.listener.getLatestCommonTime(self.parent_frame, 'right_gripper')
        position, quaternion = self.listener.lookupTransform(self.parent_frame, 'right_gripper', t)

        # Places brick in new orientation at original position
        self.move_to_point(b_pos + np.array([0,0,.04]), quaternion)
        self.gripper.open()
        self.move_to_point(b_pos + np.array([0,0,.3]), quaternion)

    def stack_bricks(self, b1, b2):
        if not self.listener.frameExists(b2):
            print 'block not found'
            exit(0)
        self.align_with_top(b1)

        # Moves to above next brick
        self.listener.waitForTransform(self.parent_frame, b2, rospy.Time(), rospy.Duration(4.0))
        t = self.listener.getLatestCommonTime(self.parent_frame, b2)
        b_pos, b_quat = self.listener.lookupTransform(self.parent_frame, b2, t)
        new_quat = np.array([1,0,0,0])
        new_pos = b_pos + np.array([0,0,.2])
        self.move_to_point(new_pos, new_quat)

        # Rotates until aligned
        velocities = self.limb.joint_velocities()
        cmd = copy.deepcopy(velocities)
        for joint in cmd.keys():
            cmd[joint] = 0
        cmd['right_w2'] = .5
        while not rospy.is_shutdown():
            self.limb.set_joint_velocities(cmd)
            t = self.listener.getLatestCommonTime(self.parent_frame, b2)
            position, quaternion = self.listener.lookupTransform(b2, 'right_gripper', t)
            rot = np.array(trans.quaternion_matrix(quaternion)[:3,:3])
            count = 0
            for i in range(3):
                for j in range(3):
                    if abs(abs(rot[i,j])-1) < .03:
                        count += 1
            if count >= 3:
                break

        # Stacks 'em
        self.move_to_point(b_pos + np.array([0,0,.055]), quaternion)
        print("Move Succesful")
        self.gripper.open()

        self.move_to_point(b_pos + np.array([0,0,.09]), quaternion)
        return b_pos

    def find_path(self, block, goal, isBlock=True):
        if not self.listener.frameExists(block):
            print block + ' not found'
            exit(0)
        if isBlock:
            if not self.listener.frameExists(goal):
                print goal + ' not found'
                exit(0)
            end = self.find_block_state(goal)
        else:
            end = self.find_state(goal)

        start = self.find_block_state(block)
        print("start" + str(start))
        print("end" + str(end))

        if start == end:
            return True

        queue = []
        i = 0
        queue.append((start, None))
        while not rospy.is_shutdown():
            nxt = queue[i]
            i = i + 1
            if nxt[0] == end:
                break
            for moves in MOVES:
                for move in moves[1]:
                    if move[0] == nxt[0]:
                        queue.append((move[1],moves[0], nxt))
        
        path = []
        while nxt[1] is not None:
            path.append((nxt[1], nxt[0]))
            nxt = nxt[2]
        path.reverse()
        print path
        return path

    def reorient(self, block, goal, isBlock=True):
        path = self.find_path(block, goal)
        while path != True:
            if path[0][0] == 'spin_c':
                self.adjust_yaw(block)
            elif path[0][0] == 'spin_cc':
                self.adjust_yaw(block, -PI/2)
            else:
                self.flip(block)
            path = self.find_path(block, goal)
        print 'FINISHED REORIENTATION'

    def find_state(self, quaternion):
        """
        State is represented as:
        -(face on table, direction of 'front' AR tag) for 
        right side up or upside down blocks
        -(face on table, direction of top) otherwise
        """
        rot = trans.quaternion_matrix(quaternion)
        bot_face, direction = None, None
        if abs(1-rot[0,0]) < .01:
            bot_face = 5
        elif abs(1-rot[2,0]) < .01:
            bot_face = 4
        elif abs(1+rot[1,0]) < .01:
            bot_face = 1        
        elif abs(1+rot[2,0]) < .01:
            bot_face = 2
        elif abs(1-rot[1,0]) < .01:
            bot_face = 3
        else:
            bot_face = 0

        # Literally magic
        if bot_face in {1,2,3,4}:
            ang = trans.angle_between_vectors([1,0,0], rot[:3,1])
            if bot_face in {1,3}:                
                dot = np.dot(np.array([0,0,1]), rot[:3,1])
            else:
                dot = np.dot(np.array([0,1,0]), rot[:3,1])
            if bot_face in {1,4}:
                dot = -dot
            if ang < .8:
                direction = 1
            elif ang > 2.6:
                direction = 3
            elif dot > 0:
                direction = 0
            else:
                direction = 2
        else:
            ang =  trans.angle_between_vectors([0,0,1], rot[:3,1])
            dot =  np.dot(np.array([0,1,0]), rot[:3,1])
            if bot_face == 0:
                ang = abs(ang-3.14)
            if ang < .8:
                direction = 0
            elif ang > 2.6:
                direction = 2
            elif dot > 0:
                direction = 3
            else:
                direction = 1

        return (bot_face, direction)


    def find_block_state(self, block):
        self.listener.waitForTransform(self.parent_frame, block, rospy.Time(), rospy.Duration(4.0))
        t = self.listener.getLatestCommonTime(self.parent_frame, block)
        _, quat = self.listener.lookupTransform(self.parent_frame, block, t)
        return self.find_state(quat)

    def record_move(self, block):
        # Grabs from top and realigns with base
        b_pos = self.align_with_top(block, False)

        recorder = JointRecorder('test', 10.0)
        print 'record in 3'
        rospy.sleep(1.0)        
        print 'record in 2'
        rospy.sleep(1.0)
        print 'record in 1'
        rospy.sleep(1.0)
        print 'recording'
        recorder.record()
        print 'finished recording'
        rospy.sleep(2.0)
        # Grabs current orientation
        t = self.listener.getLatestCommonTime(self.parent_frame, 'right_gripper')
        position, quaternion = self.listener.lookupTransform(self.parent_frame, 'right_gripper', t)


        # Places brick in new orientation at original position
        self.move_to_point(b_pos + np.array([0,0,.04]), quaternion)
        self.gripper.open()
        self.move_to_point(b_pos + np.array([0,0,.3]), quaternion)

    def replay_move(self, block):
        # Grabs from top and realigns with base
        b_pos = self.align_with_top(block, False)

        map_file('test')

        # Grabs current orientation
        t = self.listener.getLatestCommonTime(self.parent_frame, 'right_gripper')
        position, quaternion = self.listener.lookupTransform(self.parent_frame, 'right_gripper', t)


        # Places brick in new orientation at original position
        self.move_to_point(b_pos + np.array([0,0,.04]), quaternion)
        self.gripper.open()
        self.move_to_point(b_pos + np.array([0,0,.3]), quaternion)

if __name__ == '__main__':
    # For recording Trajectory
    rospy.wait_for_service('endpoint_info')
    endpointSave = rospy.ServiceProxy('endpoint_save', endpoint_save)
    endpointInfo = rospy.ServiceProxy('endpoint_info', endpoint_service)
    endpointLoad = rospy.ServiceProxy('endpoint_load', endpoint_load)

    right_builder = Builder('right')
    # while not rospy.is_shutdown():
    #     print ' '
    #     right_builder.find_path('block_0', 'block_1')
    #     rospy.sleep(0.5)
    # right_builder.reorient('block_0', 'block_1')
    #right_builder.find_path('block_0', 'block_1')
    #right_builder.adjust_yaw('block_0')
    #right_builder.flip('block_0')
    # right_builder.record_move('block_0')
    # rospy.sleep(3.0)
    right_builder.replay_move('block_0')

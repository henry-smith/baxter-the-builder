#!/usr/bin/env python
"""
Baxter_the_builder regrasping node
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
    def __init__(self, arm, gripper, parent_frame):
        self.arm = arm
        self.gripper = gripper
        self.parent_frame = parent_frame
        self.listener = tf.TransformListener()
        self.rate = rospy.Rate(100)
        time.sleep(1)

    def move_to_point(self, position, orientation):
        limb = 'right'
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        ikreq = SolvePositionIKRequest()
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')

        goal = PoseStamped()
        # goal.header.frame_id = self.parent_frame
        goal.header = hdr

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

        ikreq.pose_stamp.append(goal)
        try:
            rospy.wait_for_service(ns, 5.0)
            resp = iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return 1

        if (resp.isValid[0]):
            print("SUCCESS - Valid Joint Solution Found:")
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            print limb_joints
        else:
            print("INVALID POSE - No Valid Joint Solution Found.")
        arm = baxter_interface.Limb(limb)
        i = 0
        while not rospy.is_shutdown() and i < 100:
            arm.set_joint_positions(limb_joints)
            i += 1
            rospy.sleep(0.01)
        return 0

        # #Set the goal state to the pose you just defined
        # self.arm.set_pose_target(goal)

        # #Set the start state for the left arm
        # self.arm.set_start_state_to_current_state()

        # #Plan a path
        # plan = self.arm.plan()

        # #Execute the plan
        # #raw_input('Press <Enter> to move the right gripper to goal')
        # self.arm.go()

    def move_to_point_cartesian(self, position, orientation):
        goal = Pose()
        print "\nMoving by cartesian path to:"
        print(position)
        print(orientation)
        #x, y, and z position
        goal.position.x =  position[0]
        goal.position.y =  position[1]
        goal.position.z =  position[2]
        
        #Orientation as a quaternion
        goal.orientation.x = orientation[0] 
        goal.orientation.y = orientation[1]
        goal.orientation.z = orientation[2]
        goal.orientation.w = orientation[3]

        #Set the start state for the left arm
        self.arm.set_start_state_to_current_state()

        (plan, fraction) = self.arm.compute_cartesian_path(
                           [goal],   # waypoints to follow with end 
                           0.001,        # eef_step
                           0.0) 

        #Execute the plan
        #raw_input('Press <Enter> to move the right gripper to goal')
        self.arm.execute(plan)

    def reorient_block(self, arm, block, goal):
        # Decide to come from top or side
        pass

    def adjust_yaw(self, block, angle=90):
        self.align_with_top(block)

    def align_with_top(self, block):
        if not self.listener.frameExists(self.parent_frame):
            print 'parent frame not found'
            exit(0)
        if not self.listener.frameExists(block):
            print 'block not found'
            exit(0)
        self.listener.waitForTransform(self.parent_frame, block, rospy.Time(), rospy.Duration(4.0))
        t = self.listener.getLatestCommonTime(self.parent_frame, block)

        position, quaternion = self.listener.lookupTransform(self.parent_frame, block, t)
        # rotation = trans.quaternion_matrix(quaternion)

        # y_axis = rotation[:3,1]
        # z_axis = np.array([0, 0, -1])
        # x_axis = -np.cross(y_axis, z_axis)
        # x_axis = x_axis / np.linalg.norm(x_axis)

        # rotation[:3,:3] = np.vstack((x_axis, y_axis, z_axis)).T

        # new_quat = trans.quaternion_from_matrix(rotation)
        new_quat = np.array([1,0,0,0])
        new_pos = position + np.array([0,0,.1])

        self.move_to_point(new_pos, new_quat)
        #moveit_commander.roscpp_shutdown()
        self.rate.sleep()
        self.arm.set_start_state_to_current_state()
        self.rate.sleep()

        #moveit_commander.roscpp_shutdown()
        right_arm = baxter_interface.limb.Limb('right')
        velocities = right_arm.joint_velocities()
        cmd = copy.deepcopy(velocities)
        for joint in cmd.keys():
            cmd[joint] = 0
        cmd['right_w2'] = .2
        print cmd
        while not rospy.is_shutdown():
            right_arm.set_joint_velocities(cmd)
            t = self.listener.getLatestCommonTime(self.parent_frame, block)
            position, quaternion = self.listener.lookupTransform('right_gripper', block, t)
            print(quaternion)
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('regrasping')

    #Set up MoveIt
    # robot = moveit_commander.RobotCommander()
    # scene = moveit_commander.PlanningSceneInterface()
    right_arm = moveit_commander.MoveGroupCommander('right_arm')
    right_arm.set_pose_reference_frame('base')
    right_arm.set_planner_id('RRTConnectkConfigDefault')
    right_arm.set_planning_time(5)
    # Set up gripper
    right_gripper = baxter_gripper.Gripper('right')
    right_gripper.calibrate()

    # For recording Trajectory
    # rospy.wait_for_service('endpoint_info')
    # endpointSave = rospy.ServiceProxy('endpoint_save', endpoint_save)
    # endpointInfo = rospy.ServiceProxy('endpoint_info', endpoint_service)
    # endpointLoad = rospy.ServiceProxy('endpoint_load', endpoint_load)

    right_regrasper = Regrasper(right_arm, right_gripper, 'base')
    #right_regrasper = Regrasper(1, 1, 'base')


    right_regrasper.align_with_top('block_3')



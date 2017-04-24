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

def move_to_point(position, orientation, arm, parent_frame):
    goal = PoseStamped()
    goal.header.frame_id = parent_frame
    print "Moving to:"
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

    #Set the goal state to the pose you just defined
    arm.set_pose_target(goal)

    #Set the start state for the left arm
    arm.set_start_state_to_current_state()

    #Plan a path
    plan = arm.plan()

    #Execute the plan
    raw_input('Press <Enter> to move the right gripper to goal')
    arm.execute(plan)

def move_to_point_cartesian(position, orientation, arm, parent_frame):
    goal = Pose()
    print "Moving by cartesian path to:"
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
    arm.set_start_state_to_current_state()

    (plan, fraction) = arm.compute_cartesian_path(
                       [goal],   # waypoints to follow with end 
                       0.02,        # eef_step
                       0.0) 

    #Execute the plan
    raw_input('Press <Enter> to move the right gripper to goal')
    arm.execute(plan)


if __name__ == '__main__':
    rospy.init_node('baxter_builds')

    # # Set up MoveIt
    # robot = moveit_commander.RobotCommander()
    # scene = moveit_commander.PlanningSceneInterface()
    # right_arm = moveit_commander.MoveGroupCommander('right_arm')
    # right_arm.set_pose_reference_frame('base')
    # right_arm.set_planner_id('RRTConnectkConfigDefault')
    # right_arm.set_planning_time(5)
    # # Set up gripper
    # right_gripper = baxter_gripper.Gripper('right')
    # right_gripper.calibrate()

    # For recording Trajectory
    # rospy.wait_for_service('endpoint_info')
    # endpointSave = rospy.ServiceProxy('endpoint_save', endpoint_save)
    # endpointInfo = rospy.ServiceProxy('endpoint_info', endpoint_service)
    # endpointLoad = rospy.ServiceProxy('endpoint_load', endpoint_load)

    listener = tf.TransformListener()
    from_frame = 'base'
    to_frame = 'ar_marker_0'
    time.sleep(1)
    if not listener.frameExists(from_frame):
        print 'from_frame not found'
        exit(0)
    if not listener.frameExists(to_frame):
        print 'to_frame not found'
        exit(0)

    t = listener.getLatestCommonTime(from_frame, to_frame)
    position, quaternion = listener.lookupTransform(from_frame, to_frame, t)
    move_to_point(position, quaternion, right_arm, 'base')



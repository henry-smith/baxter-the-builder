#!/usr/bin/env python
"""
EE106b Lab 1 
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


class EndpointSubscriber:
	def __init__(self):
		self.last_pose = None
		self.last_twist = None
		self.poses = []
		self.twists = []
		self.pause = 10
		self.index = 0
		self.saving = False


	def begin(self):
		rospy.init_node('listener', anonymous=True)
		rospy.Subscriber("/robot/limb/left/endpoint_state", EndpointState, self.saveVals)
		rospy.Service('endpoint_info', endpoint_service, self.returnValues)
		rospy.Service('endpoint_load', endpoint_load, self.beginSave)
		rospy.Service('endpoint_save', endpoint_save, self.saveArray)
		print 'Success'
		rospy.spin()

	def beginSave(self, req):
		self.saving = True
		self.index = 0
		return True

	def saveVals(self, state):
		self.last_pose = state.pose
		self.last_twist = state.twist
		if self.saving:
			if self.index == self.pause -1:
				self.poses.append(self.last_pose)
				self.twists.append(self.last_twist)
			self.index = (self.index + 1) % self.pause

	def returnValues(self, req):
		return (self.last_pose, self.last_twist)

	def saveArray(self, req):
		with open(req.filename, 'wb') as f:
			print "hehre"
			pickle.dump([self.poses, self.twists] , f)
		self.saving = False
		self.poses = []
		self.twists = []
		return True


if __name__ == '__main__':
	ES = EndpointSubscriber()
	ES.begin()





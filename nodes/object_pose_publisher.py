#!/usr/bin/env python  
import roslib
import rospy
import math
import numpy as np
import tf
import geometry_msgs.msg
import sys

from core import RigidTransform

class ObjectTemplate(object):
    """ Struct for specifying object templates """
    def __init__(self, name, ar_marker, R_ar_obj=np.eye(3), t_ar_obj=np.zeros(3)):
        self.name = name
        self.ar_marker = ar_marker
        self.T_ar_obj = RigidTransform(rotation=R_ar_obj, translation=t_ar_obj,
                                       from_frame=name, to_frame=ar_marker)

    @property
    def q_ar_obj(self):
        return tf.transformations.quaternion_from_matrix(self.T_ar_obj.matrix)

    @property
    def t_ar_obj(self):
        return self.T_ar_obj.translation

OBJECT_TEMPLATES = {
    ObjectTemplate(name='block_0', 
                   ar_marker='ar_marker_0', 
                   t_ar_obj=[0.0, 0.0, -0.02], 
                   R_ar_obj=np.array([[1, 0, 0],
                                      [0, 0, -1],
                                      [0, 1, 0]])),
}

if __name__ == '__main__':
    rospy.init_node('object_pose_publisher')

    broadcaster = tf.TransformBroadcaster()
    listener = tf.TransformListener()
 
    print 'Publishing object pose'
    
    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        try:
            for object_template in OBJECT_TEMPLATES:
                broadcaster.sendTransform(object_template.t_ar_obj,object_template.q_ar_obj, listener.getLatestCommonTime('base', 'left_hand_camera'), object_template.name, object_template.ar_marker)
        except:
            continue
        rate.sleep()

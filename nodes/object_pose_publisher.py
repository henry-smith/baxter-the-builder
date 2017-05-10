#!/usr/bin/env python  
import roslib
import rospy
import math
import numpy as np
import tf
import geometry_msgs.msg
import sys

from core import RigidTransform

class TransformTemplate(object):
    """ Struct for specifying object templates """
    def __init__(self, name, ar_marker, R_ar_obj=np.eye(3), t_ar_obj=np.zeros(3)):
        self.ar_marker = ar_marker
        self.T_ar_obj = RigidTransform(rotation=R_ar_obj, translation=t_ar_obj,
                                       from_frame=name, to_frame=ar_marker)

    @property
    def q_ar_obj(self):
        return tf.transformations.quaternion_from_matrix(self.T_ar_obj.matrix)

    @property
    def t_ar_obj(self):
        return self.T_ar_obj.translation

BLOCKS = {
    ('block_0',(
    TransformTemplate(name='block_0',
                      ar_marker='ar_marker_0', 
                      t_ar_obj=[-0.002, 0.0, -0.03], 
                      R_ar_obj=np.array([[1, 0, 0],
                                         [0, 0, -1],
                                         [0, 1, 0]])),
    TransformTemplate(name='block_0',
                      ar_marker='ar_marker_3', 
                      t_ar_obj=[0.0, 0.0, -0.03], 
                      R_ar_obj=np.array([[0, -1, 0],
                                         [0, 0, -1],
                                         [1, 0, 0]])),
    TransformTemplate(name='block_0',
                      ar_marker='ar_marker_4', 
                      t_ar_obj=[0.0, 0.0, -0.03], 
                      R_ar_obj=np.array([[-1, 0, 0],
                                         [0, 0, -1],
                                         [0, -1, 0]])),
    TransformTemplate(name='block_0',
                      ar_marker='ar_marker_5', 
                      t_ar_obj=[0.0, 0.0, -0.03], 
                      R_ar_obj=np.array([[0, 1, 0],
                                         [0, 0, -1],
                                         [-1, 0, 0]]))
    )),
    ('block_1', (
    TransformTemplate(name='block_1',
                ar_marker='ar_marker_2', 
                t_ar_obj=[0.0, 0.0, -0.03], 
                R_ar_obj=np.array([[1, 0, 0],
                                   [0, 0, -1],
                                   [0, 1, 0]])),
    ))
}

if __name__ == '__main__':
    rospy.init_node('object_pose_publisher')

    broadcaster = tf.TransformBroadcaster()
    listener = tf.TransformListener()
    rospy.sleep(1.0)
 
    print 'Publishing object pose'
    
    rate = rospy.Rate(100.0)
    while not rospy.is_shutdown():
        try:
          for block in BLOCKS:
            # This next loop finds the most recent ar_tag timestamps 
            # and uses that for creating the block frame
            transform_to_use = None
            most_recent_time = rospy.Time.now()
            for transform_template in block[1]:
              if listener.frameExists(transform_template.ar_marker):
                t = listener.getLatestCommonTime('base', transform_template.ar_marker)
                if t > most_recent_time:
                  transform_to_use = transform_template
                  most_recent_time = t
            if transform_to_use is not None:
              broadcaster.sendTransform(transform_to_use.t_ar_obj,transform_to_use.q_ar_obj, listener.getLatestCommonTime('base', 'left_hand_camera'), block[0], transform_to_use.ar_marker)
            rate.sleep()
        except Exception,e: 
          print str(e)
          continue
        rate.sleep()

#!/usr/bin/env python  

import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('blockframe_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform((0.0, 2.0, 0.0),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "'ar_marker_0'",
                         "brick_0")
        rate.sleep()
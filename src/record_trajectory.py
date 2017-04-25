#!/usr/bin/env python
"""
Baxter_the_builder trajectory feature recorder
Author: Henry Smith & Andrew Chan
"""
import rospy
import baxter_interface
import tf
import argparse
import numpy as np
from geometry_msgs.msg import PoseStamped, Pose


class FeatureRecorder(object):
    def __init__(self, filename, rate):
        """
        Records joint data to a file at a specified rate.
        """
        self.filename = filename
        self.raw_rate = rate
        self.rate = rospy.Rate(rate)
        self.start_time = rospy.get_time()
        self._done = False

        self.left_limb = baxter_interface.Limb("left")
        self.right_limb= baxter_interface.Limb("right")
        self.left_gripper = baxter_interface.Gripper("left")
        self.right_gripper = baxter_interface.Gripper("right")
        self.io_left_lower = baxter_interface.DigitalIO('left_lower_button')
        self.io_left_upper = baxter_interface.DigitalIO('left_upper_button')
        self.io_right_lower = baxter_interface.DigitalIO('right_lower_button')
        self.io_right_upper = baxter_interface.DigitalIO('right_upper_button')

        # Verify Grippers Have No Errors and are Calibrated
        if self.left_gripper.error():
            self.left_gripper.reset()
        if self.right_gripper.error():
            self.right_gripper.reset()
        if (not self.left_gripper.calibrated() and
            self.left_gripper.type() != 'custom'):
            self.left_gripper.calibrate()
        if (not self.right_gripper.calibrated() and
            self.right_gripper.type() != 'custom'):
            self.right_gripper.calibrate()

    def _time_stamp(self):
        return rospy.get_time() - self.start_time

    def stop(self):
        """
        Stop recording.
        """
        self._done = True

    def done(self):
        """
        Return whether or not recording is done.
        """
        if rospy.is_shutdown():
            self.stop()
        return self._done

    def record(self, goal_block):
        """
        Records the current feature states to a csv file 
        """
        if self.filename:
            joints_left = self.left_limb.joint_names()
            joints_right = self.right_limb.joint_names()

            listener = tf.TransformListener()
            from_frame = 'base'
            to_frame = goal_block
            rospy.sleep(1)
            if not listener.frameExists(from_frame):
                print 'from_frame not found'
                exit(0)
            if not listener.frameExists(to_frame):
                print 'to_frame not found'
                exit(0)

            traj = []
            while not self.done():
                # Look for gripper button presses
                if self.io_left_lower.state:
                    self.left_gripper.open()
                elif self.io_left_upper.state:
                    self.left_gripper.close()
                if self.io_right_lower.state:
                    self.right_gripper.open()
                elif self.io_right_upper.state:
                    self.right_gripper.close()
                angles_left = [self.left_limb.joint_angle(j)
                               for j in joints_left]
                angles_right = [self.right_limb.joint_angle(j)
                                for j in joints_right]

                output = [self._time_stamp(), self.right_gripper.position()]
                output.extend(angles_right)

                t = listener.getLatestCommonTime(from_frame, to_frame)
                pos, quat = listener.lookupTransform(from_frame, to_frame, t)
                output.extend([str(pos[0]), str(pos[1]), str(pos[2])])
                output.extend([str(quat[0]), str(quat[1]), str(quat[2]), str(quat[3])])
                traj.append(np.array(output))
                self.rate.sleep()
            trajectory = np.vstack(tuple(traj))
            np.save(self.filename, trajectory)
            print('Saved')

def main():
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-f', '--file', dest='filename', required=True,
        help='the file name to record to'
    )
    parser.add_argument(
        '-r', '--record-rate', type=int, default=100, metavar='RECORDRATE',
        help='rate at which to record (default: 100)'
    )
    args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("feature_recorder")
    # print("Getting robot state... ")
    # rs = baxter_interface.RobotEnable(CHECK_VERSION)
    # print("Enabling robot... ")
    # rs.enable()

    recorder = FeatureRecorder(args.filename, args.record_rate)
    rospy.on_shutdown(recorder.stop)
    rospy.sleep(5)
    print("Recording. Press Ctrl-C to stop.")
    recorder.record('block_0')

    print("\nDone.")

if __name__ == '__main__':
    main()

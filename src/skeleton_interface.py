#!/usr/bin/env python
################
# ROS IMPORTS: #
################
import roslib; roslib.load_manifest('kinect_pendulum_demo')
import rospy
import tf
from skeletonmsgs_nu.msg import Skeletons
from skeletonmsgs_nu.msg import Skeleton
from skeletonmsgs_nu.msg import SkeletonJoint


####################
# NON ROS IMPORTS: #
####################
import sys
import os
import os.path
import math
import copy
from math import fmod, pi, copysign


####################
# GLOBAL VARIABLES #
####################




class PendulumController:
    def __init__(self):
        rospy.loginfo("Starting pendulum controller...")

        # define a subscriber for the skeleton information:
        self.skel_sub = rospy.Subscriber("skeletons", Skeletons,
                                         self.skelcb)

        self.fname = "fifopipe"
        try:
            os.mkfifo(self.fname)
        except OSError, e:
            print "Failed to create FIFO: %s" % e


    def skelcb(self, data):
        # get RH position, and set its value
        if len(data.skeletons) == 0:
            return
        skel = data.skeletons[0]
        pos = skel.right_hand.transform.translation.x

        # If the left hand is above the hip, then we are active:
        hand_height = skel.left_hand.transform.translation.y
        hip_height = skel.left_hip.transform.translation.y
        if hand_height < hip_height:
            out = '{0:6.3f}\n'.format(pos)
            fifo = open(self.fname, 'w')
            fifo.write(out)
            fifo.close()

        return

    def shutdown(self):
        print "Destroying pipe!"
        os.remove(self.fname)


def main():
    # ros initialization:
    rospy.init_node('pendulum_simulator', log_level=rospy.INFO)
    try:
        pend = PendulumController()
    except rospy.ROSInterruptException as e:
        rospy.logerror("Failed to instantiate PendulumController()")
        rospy.logerror("message = %s"%e.message)

    rospy.on_shutdown(pend.shutdown)
    rospy.spin()

if __name__=='__main__':
    main()

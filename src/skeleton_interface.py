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
import skeleton_filter as sf


####################
# NON ROS IMPORTS: #
####################
import sys, signal
import os
import math
import copy
from math import fmod, pi, copysign
import pendulum_simulator as ps
import numpy as np

####################
# GLOBAL VARIABLES #
####################
SCALING = 5.0 ## real-world motion gets multiplied by this before being sent as
              ## a reference


class QtROS(ps.QThread):
    def __init__(self, node_name):
        super(QtROS, self).__init__()
        rospy.init_node(node_name, log_level=rospy.INFO)
        rospy.loginfo("QtROS connected to roscore")
        self.quitfromgui = False

    def quitnow(self):
        self.quitfromgui = True

    def run(self):
        while not rospy.is_shutdown() and not self.quitfromgui:
            rospy.sleep(1/30.0)
        if self.quitfromgui:
            rospy.loginfo("emitting rosQuits() signal")
            self.emit(ps.SIGNAL("rosQuits()"))



class PendulumController:
    def __init__(self, DW):
        rospy.loginfo("Starting pendulum controller...")

        # define a subscriber for the skeleton information:
        self.skel_sub = rospy.Subscriber("skeletons", Skeletons,
                                         self.skelcb)
        self.skel_pub = rospy.Publisher("skeleton_filt", Skeleton)
        self.filt = sf.SkeletonFilter(sf.joints)

        self.DW = DW
        self.first_flag = True
        self.already_reset_flag = True
        self.base_pos = 0.0
        # filter parameters:
        self.falpha = 0.25
        self.gamma = 0.05
        self.a_low = 0.01
        self.a_high = 0.50
        self.v_high = 0.008
        self.v_low = 0.001
        self.bn = 0.0
        self.prebn = 0.0
        self.prex = 0.0


    def skelcb(self, data):
        # get RH position, and set its value
        if len(data.skeletons) == 0:
            return
        skel = data.skeletons[0]
        skelf = self.filt.filter_skeletons(skel)
        self.skel_pub.publish(skelf)

        pos = skel.right_hand.transform.translation.x

        # run filter:
        if not self.first_flag:
            vn = np.abs(pos - self.prex)
            if vn < self.v_low:
                self.falpha = self.a_low
            elif self.v_low <= vn <= self.v_high:
                self.falpha = self.a_high + ((vn-self.v_high)/(self.v_low-self.v_high))*\
                  (self.a_low-self.a_high)
            elif vn > self.v_high:
                self.falpha = self.a_high
            else:
                self.falpha = (self.a_high+self.a_low)/2.0
            pos = self.falpha*pos + (1-self.falpha)*(self.prex+self.bn)
            self.bn = self.gamma*(pos-self.prex) + (1-self.gamma)*self.prebn
            self.prebn = self.bn
            self.prex = pos

        # If the left hand is above the hip, then we are active:
        hand_height = skel.left_hand.transform.translation.y
        hip_height = skel.left_hip.transform.translation.y
        if hand_height < hip_height:
            self.already_reset_flag = False
            if self.first_flag: # find offset and reset filter:
                self.base_pos = pos
                self.first_flag = False
                self.bn = 0.0
                self.prex = pos
                self.prebn = 0.0
            pos -= self.base_pos
            self.DW.mouse_pos = np.array([(SCALING)*pos])
            # /float(self.DW.num_links)
        else:
            self.first_flag = True

        hand_height_r = skel.right_hand.transform.translation.y
        hip_height_r = skel.right_hip.transform.translation.y
        if hand_height > hip_height and hand_height_r > hip_height_r:
            if not self.already_reset_flag:
                rospy.loginfo("Reset!")
                self.DW.reset_flag = True
                self.already_reset_flag = True

        return


def main():
    app = ps.QApplication(sys.argv)
    demo = ps.DemoWindow()
    demo.resize(*ps.DEFAULT_WINDOW_SIZE)
    demo.setWindowTitle("Pendulum Animation")
    demo.show()

    # start QtROS:
    qtros = QtROS("skeleton_interface")
    # connect quit signals:
    ps.QObject.connect(qtros, ps.SIGNAL("rosQuits()"), app.quit)
    ps.QObject.connect(app, ps.SIGNAL("aboutToQuit()"), qtros.quitnow)
    ps.QObject.connect(qtros, ps.SIGNAL("rosQuits()"), demo.close)
    # create pendulum controller:
    pend = PendulumController(demo)

    qtros.start()
    signal.signal(signal.SIGINT, signal.SIG_DFL) # allows demo to be killed with Ctrl+C
    sys.exit(app.exec_())


if __name__=='__main__':
    main()

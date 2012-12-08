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
        self.DW = DW
        self.run_flag = False
        self.first_flag = True
        self.base_pos = 0.0
        self.pos_hist = [0.0]*5
        RC = 1/10.0
        DT = 1/30.0
        self.alpha = DT/(RC+DT)


    def skelcb(self, data):
        # get RH position, and set its value
        if len(data.skeletons) == 0:
            return
        skel = data.skeletons[0]

        pos = skel.right_hand.transform.translation.x
        self.pos_hist.append(pos)
        self.pos_hist.pop(0)
        y = [self.pos_hist[0]]*len(self.pos_hist)
        for i in range(1,len(self.pos_hist)):
            y[i] = self.alpha*self.pos_hist[i] + (1-self.alpha)*y[i-1]
        pos = y[-1]

        # If the left hand is above the hip, then we are active:
        hand_height = skel.left_hand.transform.translation.y
        hip_height = skel.left_hip.transform.translation.y
        if hand_height < hip_height:
            self.run_flag = True
            if self.first_flag:
                self.base_pos = pos
                self.first_flag = False

            pos -= self.base_pos
            self.DW.mouse_pos = np.array([(SCALING)*pos])
            # /float(self.DW.num_links)

        else:
            self.first_flag = True
            return
        hand_height = skel.right_hand.transform.translation.y
        hip_height = skel.right_hip.transform.translation.y
        if hand_height < hip_height:
            if self.run_flag:
                self.DW.reset_flag = True
                self.run_flag = False

        return


def main():
    app = ps.QApplication(sys.argv)
    demo = ps.DemoWindow()
    demo.resize(*ps.DEFAULT_WINDOW_SIZE)
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

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
import testing_pendulum_simulator as ps
import numpy as np

####################
# GLOBAL VARIABLES #
####################
SCALING = 7.5 ## real-world motion gets multiplied by this before being sent as
              ## a reference
# frequency divider for checking "key" skeleton:
FREQ_DIV = 30
ANG_MULT = 20
DIST_MULT = 1



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
        # key skel params:
        self.count = 0
        self.key_index = 0
        self.key_id = 1

        
    def get_key_user(self, skels):
        data = []
        for i,s in enumerate(skels):
            v2 = np.array([s.head.transform.translation.x,
                           s.head.transform.translation.z])
            ang = np.arccos(v2[1]/np.linalg.norm(v2))
            dist = v2[1]
            cost = ANG_MULT*ang + DIST_MULT*dist
            data.append([i, s.userid, cost])
        val, idx = min((val[2], idx) for (idx, val) in enumerate(data))
        self.key_index = data[idx][0]
        self.key_id = data[idx][1]
        return

    
    def skelcb(self, data):
        if len(data.skeletons) == 0:
            return
        if self.count%FREQ_DIV == 0:
            self.get_key_user(data.skeletons)
        self.count += 1
        if self.key_index < len(data.skeletons) and \
                data.skeletons[self.key_index].userid == self.key_id:
            skel = data.skeletons[self.key_index]
        else:
            for i,skel in enumerate(data.skeletons):
                if skel.userid == self.key_id:
                    found = True
                    break
                found = False
            if not found:
                rospy.logwarn("Could not find a skeleton userid that matches the key user")
                return

        if self.DW.lefty:
            pos = skel.left_hand.transform.translation.x
        else:
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

        if self.DW.simulation_running:
            self.already_reset_flag = False
            if self.first_flag: # find offset and reset filter:
                self.base_pos = pos + self.DW.offset_mag/SCALING
                self.first_flag = False
                self.bn = 0.0
                self.prex = pos
                self.prebn = 0.0
            pos -= self.base_pos
            self.DW.mouse_pos = np.array([(SCALING)*pos])
            # /float(self.DW.num_links)
        else:
            self.first_flag = True

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

#!/usr/bin/python

# import matplotlib as mpl
# import sys
# # lookup and use the backend_pgf module in the parent folder
# mpl.use("module://backend_pgf")
# # pdf backend with text.usetex
# font_spec = {
#     "text.usetex": True,
#     "text.latex.unicode": True,
#     "text.latex.preamble": [r"\usepackage{lmodern}",
#                             r"\usepackage[T1]{fontenc}",
#                             r"\usepackage{amsmath}"],
#     "font.family": "serif",
#     "font.serif": ["CMU Serif"],
#     "font.sans-serif": ["CMU Sans Serif"],
#     "font.monospace": [], # fallback to the default LaTeX monospace font
#     }
# # fnames_sets.append(["figure-pdf-usetex.pdf"])
# # # use latex default fonts only
# # font_spec = {"pgf.rcfonts": False,
# #              "font.family": "serif",
# #              # fonts specified in matplotlib are ignored
# #              "font.serif": ["dont care"],
# #              "font.sans-serif": ["me neither"],
# #            }
# mpl.rcParams.update(font_spec)
# rc_text = {"font.family": "serif", "axes.labelsize": 11, "text.fontsize": 11,
#            "legend.fontsize": 11, "xtick.labelsize": 11, "ytick.labelsize": 11}
# mpl.rcParams.update(rc_text)
# ##############################################################################

import os
import sys
from math import sin, cos
from math import pi
import numpy as np
from matplotlib.pyplot import hold, plot
import matplotlib.pyplot as mp
import copy
import scipy.io as sio
import glob
import re
import angle_utils as au
from matplotlib import rc
import itertools
from scipy.special import erf

# global constants:
base_dir = '/home/jarvis/ros/packages/kinect_pendulum_demo/data/ordered/'
NUM_TRIALS_PER = 15
NUM_TRIALS = 4*NUM_TRIALS_PER
MAX_ANGLE = pi/6.0
BASE_COST = 602
TASK_DIST_BASE_COST = 120.4
ANGLE_DIST_BASE_COST = 0
POSTURE_BASE_COST = 120.4
TF = 6.0
DT = 0.02
SCALING = 5.0 ## real-world motion gets multiplied by this before being sent as
              ## a reference
# paramters related to window scaling
WIDTH = 1918.
HEIGHT = 602.
MAX_Y = 2.75
MIN_Y = -0.5
YSPAN = MAX_Y - MIN_Y
XSPAN = (WIDTH/HEIGHT)*YSPAN # width of testing screen in "trep meters"


# Predefined trust points:
T1 = 0.0
T2 = 40.0
T3 = 100.0

def norm_func(dat):
    """
    This function is for actually calculating the norm of the data.  It is
    supposed to take in a dict from one of the mat files.
    """
    # configuration reference
    # qd = np.array([dat['goal_offset'][0,0]])/SCALING
    qd = np.array([0])

    # now calculate the total error between q and qd throughout time horizon
    err_a = 0
    # for q in dat['q']:
    #     if np.abs(q[0]) > MAX_ANGLE:
    #         q[0] = MAX_ANGLE
    #     err_a += np.linalg.norm(q[0] - qd)
    # err_a /= len(dat['q'])

    qd = np.array([dat['goal_offset'][0,0]])/SCALING
    err_p = 0
    for q in dat['q']:
        err_p += np.linalg.norm(q[1]/SCALING - qd)
    err_p /= len(dat['q'])
    return err_a+err_p



def posture_norm_func(dat):
    # configuration reference
    qd = np.array([dat['goal_offset'][0,0]])/SCALING
    # now calculate the total error between q and qd throughout time horizon
    err = 0
    for q in dat['u']:
        err += np.linalg.norm(q/SCALING - qd)
    return err/len(dat['u'])



class User:
    def __init__(self, index, group):
        index = str(index)
        self.index = index
        group = str(group)
        assert ( int(group) in range(1,7) )
        self.group = group
        self.data = None


    def fill_trials(self):
        """
        This function scans through the directory and adds a list of
        dictionaries to the class
        """
        self.data = []
        subdir = 'subject' + self.index + '_' + self.group
        dir = os.path.join(base_dir, subdir)

        for i in range(1, NUM_TRIALS+1):
            fname = 'subject{0:s}_order_{1:s}_trial_{2:d}.mat'.format(self.index, self.group, i)
            fname = os.path.join(dir,fname)
            try:
                dat = sio.loadmat(fname)
                self.data.append(dat)
            except:
                print 'Could not locate file: \r\n',fname,'\r\n'



    def calc_all_errors(self):
        self.errs = []
        self.valid_errs = []
        self.errs_trials = []
        self.perrs = []
        for i in range(NUM_TRIALS):
            err = norm_func(self.data[i])
            perr = posture_norm_func(self.data[i])
            self.errs.append(err)
            self.perrs.append(perr)
            if err < BASE_COST:
                self.valid_errs.append(err)
                self.errs_trials.append(i)

            q = self.data[i]['q']
            if len(q) > 301:
                print "Data too long:",len(q)
                print "name = ",self.name
                print "trial = ",i,"\r\n"
                print "Final time = ",self.data[i]['t'][-1]


    def sort_errs_by_group(self):
        self.trainers = []
        self.t1errs = []
        self.t2errs = []
        self.t3errs = []
        for i in range(NUM_TRIALS):
            if self.data[i]['trust'][0,0] == T1:
                self.t1errs.append(self.errs[i])
            elif self.data[i]['trust'][0,0] == T2:
                self.t2errs.append(self.errs[i])
            elif self.data[i]['trust'][0,0] == T3:
                self.t3errs.append(self.errs[i])
            elif self.data[i]['trust'][0,0] == -1:
                self.trainers.append(self.errs[i])
            else:
                print "INVALID TRUST VALUE!"


    def sort_posture_errs_by_group(self):
        self.posturetrainers = []
        self.t1perrs = []
        self.t2perrs = []
        self.t3perrs = []
        for i in range(NUM_TRIALS):
            if self.data[i]['trust'][0,0] == T1:
                self.t1perrs.append(self.perrs[i])
            elif self.data[i]['trust'][0,0] == T2:
                self.t2perrs.append(self.perrs[i])
            elif self.data[i]['trust'][0,0] == T3:
                self.t3perrs.append(self.perrs[i])
            elif self.data[i]['trust'][0,0] == -1:
                self.posturetrainers.append(self.perrs[i])
            else:
                print "INVALID TRUST VALUE!"


# def main():
# experimental definitions
users = [
    User(1, 1),
    User(2, 2),
    User(3, 3),
    User(4, 4),
    User(5, 5),
    User(6, 6),
    User(7, 1),
    User(8, 2),
    User(9, 3),
    User(10, 4),
    User(11, 5),
    User(12, 6)
    ]
    

for u in users:
    u.fill_trials()
    # u.cleanup_long_datasets()
    # u.calc_last_few_errors()
    u.calc_all_errors()
    u.sort_errs_by_group()
    u.sort_posture_errs_by_group() 

### for each trust value, calculate each user's time evolution

arr = [T1,T2,T3]
dat = {}
for i,t in enumerate(itertools.permutations(arr)):
    dat[i] = list(t)


######################
# TASK SUCCESS STUFF #
######################
mp.rc('text', usetex=True)
mp.rc('font', family='serif', size='14')
fig = mp.gcf()
# T1 stuff:
mp.subplot(3,1,1)
vals = np.zeros((len(users), NUM_TRIALS_PER))
hold(True)
for i,u in enumerate(users):
    vals[i,:] = u.t1errs
# for i,u in enumerate(users):
    # mp.plot(range(1,NUM_TRIALS_PER+1), u.t1errs, '-o', alpha=.4)
    # vals[i,:] = u.t1errs
# mp.errorbar(range(1,NUM_TRIALS_PER+1), np.mean(vals, 0), np.std(vals, 0),
#             marker='x', color='k', ecolor='k', markersize=12, mfc='None', mew = 1.25,
#             linewidth=2.0, linestyle='--', alpha=1, zorder=1)
means = []
stds = []
masks = {}
for i in range(12): masks[i] = []
for i in range(NUM_TRIALS_PER):
    samps = vals[:,i]
    mean = np.mean(samps)
    std = np.std(samps)
    tsamps = []
    ttrials = []
    for j,s in enumerate(samps):
        err = np.abs(s-mean)/std
        prob = 1-erf(err/np.sqrt(2))
        if prob*len(samps) > 0.5:
            tsamps.append(s)
            ttrials.append(i+1)
            masks[j].append(i)
    means.append(np.mean(tsamps))
    stds.append(np.std(tsamps))
    # mp.plot(ttrials, tsamps,  '-o', alpha=.4)
    print "Trial {0:s} has {1:d} subjects remaining".format(str(i+1), len(tsamps))
for i,u in enumerate(users):
    mp.plot(np.array(masks[i])+1, vals[i,masks[i]], '--o', alpha=.4)
mp.errorbar(range(1,NUM_TRIALS_PER+1), means, stds,
            marker='x', color='b', ecolor='b', markersize=12, mfc='None', mew = 1.25,
            linewidth=2.0, linestyle='-', alpha=.75, zorder=1)
# mp.plot([0,NUM_TRIALS_PER+1], [ANGLE_DIST_BASE_COST+TASK_DIST_BASE_COST]*2 ,'--',linewidth=2, color='gray')
hold(False)
mp.xlim([0,NUM_TRIALS_PER+1])
mp.ylim([0,0.71])
# mp.xlabel('Trial Number')
# mp.ylabel('Continuous Task Success Metric')
mp.title(r'Automation-Only Control Allocation $(f_{auto})$', fontsize=12)
# mp.show()
# mp.close()
# mp.cla()
# mp.clf()


# T2 stuff:
mp.subplot(3,1,2)
vals = np.zeros((len(users), NUM_TRIALS_PER))
hold(True)
for i,u in enumerate(users):
    vals[i,:] = u.t2errs
# for i,u in enumerate(users):
    # mp.plot(range(1,NUM_TRIALS_PER+1), u.t2errs, '-o', alpha=.4)
    # vals[i,:] = u.t2errs
# mp.errorbar(range(1,NUM_TRIALS_PER+1), np.mean(vals, 0), np.std(vals, 0),
#             marker='x', color='k', ecolor='k', markersize=12, mfc='None', mew = 1.25,
#             linewidth=2.0, linestyle='--', alpha=1, zorder=1)
means = []
stds = []
masks = {}
for i in range(12): masks[i] = []
for i in range(NUM_TRIALS_PER):
    samps = vals[:,i]
    mean = np.mean(samps)
    std = np.std(samps)
    tsamps = []
    ttrials = []
    for j,s in enumerate(samps):
        err = np.abs(s-mean)/std
        prob = 1-erf(err/np.sqrt(2))
        if prob*len(samps) > 0.5:
            tsamps.append(s)
            ttrials.append(i+1)
            masks[j].append(i)
    means.append(np.mean(tsamps))
    stds.append(np.std(tsamps))
    # mp.plot(ttrials, tsamps,  '-o', alpha=.4)
    print "Trial {0:s} has {1:d} subjects remaining".format(str(i+1), len(tsamps))
for i,u in enumerate(users):
    mp.plot(np.array(masks[i])+1, vals[i,masks[i]], '--o', alpha=.4)
mp.errorbar(range(1,NUM_TRIALS_PER+1), means, stds,
            marker='x', color='b', ecolor='b', markersize=12, mfc='None', mew = 1.25,
            linewidth=2.0, linestyle='-', alpha=.75, zorder=1)
# mp.plot([0,NUM_TRIALS_PER+1], [ANGLE_DIST_BASE_COST+TASK_DIST_BASE_COST]*2 ,'--',linewidth=2, color='gray')
hold(False)
mp.xlim([0,NUM_TRIALS_PER+1])
mp.ylim([0,0.71])
# mp.xlabel('Trial Number')
# mp.ylabel('Continuous Task Success Metric')
mp.title(r'Shared Control Allocation $(f_{split})$', fontsize=12)
# mp.show()
# mp.close()
# mp.cla()
# mp.clf()


# T3 stuff:
mp.subplot(3,1,3)
vals = np.zeros((len(users), NUM_TRIALS_PER))
hold(True)
for i,u in enumerate(users):
    vals[i,:] = u.t3errs
# for i,u in enumerate(users):
    # mp.plot(range(1,NUM_TRIALS_PER+1), u.t3errs, '-o', alpha=.4)
    # vals[i,:] = u.t3errs
# mp.errorbar(range(1,NUM_TRIALS_PER+1), np.mean(vals, 0), np.std(vals, 0),
#             marker='x', color='k', ecolor='k', markersize=12, mfc='None', mew = 1.25,
#             linewidth=2.0, linestyle='--', alpha=1, zorder=1)
means = []
stds = []
masks = {}
for i in range(12): masks[i] = []
for i in range(NUM_TRIALS_PER):
    samps = vals[:,i]
    mean = np.mean(samps)
    std = np.std(samps)
    tsamps = []
    ttrials = []
    for j,s in enumerate(samps):
        err = np.abs(s-mean)/std
        prob = 1-erf(err/np.sqrt(2))
        if prob*len(samps) > 0.5:
            tsamps.append(s)
            ttrials.append(i+1)
            masks[j].append(i)
    means.append(np.mean(tsamps))
    stds.append(np.std(tsamps))
    # mp.plot(ttrials, tsamps,  '-o', alpha=.4)
    print "Trial {0:s} has {1:d} subjects remaining".format(str(i+1), len(tsamps))
for i,u in enumerate(users):
    mp.plot(np.array(masks[i])+1, vals[i,masks[i]], '--o', alpha=.4)
mp.errorbar(range(1,NUM_TRIALS_PER+1), means, stds,
            marker='x', color='b', ecolor='b', markersize=12, mfc='None', mew = 1.25,
            linewidth=2.0, linestyle='-', alpha=.75, zorder=1)
# mp.plot([0,NUM_TRIALS_PER+1], [ANGLE_DIST_BASE_COST+TASK_DIST_BASE_COST]*2 ,'--',linewidth=2, color='gray')
hold(False)
mp.xlim([0,NUM_TRIALS_PER+1])
mp.ylim([0,0.71])
# mp.xlabel('Trial Number')
# mp.ylabel('Continuous Task Success Metric')
mp.title(r'Human-Only Control Allocation $(f_{user})$', fontsize=12)
# mp.show()
# mp.close()
# mp.cla()
# mp.clf()

mp.subplots_adjust(hspace=.25)
fig.text(0.5, 0.04, 'Trial Number', ha='center', va='center', fontsize=14)
fig.text(0.06, 0.5, 'Mean Position Task Error [m]', ha='center', va='center', rotation='vertical', fontsize=14)
# fig.text(0.06, 0.5, 'Mean Angle Task Error [rad]', ha='center', va='center', rotation='vertical', fontsize=14)
# fig.text(0.06, 0.5, 'Mean Total Task Error', ha='center', va='center', rotation='vertical', fontsize=14)
mp.show()
mp.close()
mp.cla()
mp.clf()



# #########################
# # POSTURE SUCCESS STUFF #
# #########################
# fig = mp.gcf()
# # T1 stuff:
# mp.subplot(3,1,1)
# vals = np.zeros((len(users), NUM_TRIALS_PER))
# hold(True)
# for i,u in enumerate(users):
#     vals[i,:] = u.t1perrs
# means = []
# stds = []
# masks = {}
# for i in range(12): masks[i] = []
# for i in range(NUM_TRIALS_PER):
#     samps = vals[:,i]
#     mean = np.mean(samps)
#     std = np.std(samps)
#     tsamps = []
#     ttrials = []
#     for j,s in enumerate(samps):
#         err = np.abs(s-mean)/std
#         prob = 1-erf(err/np.sqrt(2))
#         if prob*len(samps) > 0.5:
#             tsamps.append(s)
#             ttrials.append(i+1)
#             masks[j].append(i)
#     means.append(np.mean(tsamps))
#     stds.append(np.std(tsamps))
#     # mp.plot(ttrials, tsamps,  '-o', alpha=.4)
#     print "Trial {0:s} has {1:d} subjects remaining".format(str(i+1), len(tsamps))
# for i,u in enumerate(users):
#     mp.plot(np.array(masks[i])+1, vals[i,masks[i]], '--o', alpha=.4)
# mp.errorbar(range(1,NUM_TRIALS_PER+1), means, stds,
#             marker='x', color='b', ecolor='b', markersize=12, mfc='None', mew = 1.25,
#             linewidth=2.0, linestyle='-', alpha=.75, zorder=1)
# # mp.plot([0,NUM_TRIALS_PER+1], [POSTURE_BASE_COST]*2 ,'--',linewidth=2, color='gray')
# hold(False)
# mp.xlim([0,NUM_TRIALS_PER+1])
# mp.ylim([0,0.71])
# mp.title(r'Automation-Only Control Allocation $(f_{auto})$', fontsize=12)


# # T2 stuff:
# mp.subplot(3,1,2)
# vals = np.zeros((len(users), NUM_TRIALS_PER))
# hold(True)
# for i,u in enumerate(users):
#     vals[i,:] = u.t2perrs
# means = []
# stds = []
# masks = {}
# for i in range(12): masks[i] = []
# for i in range(NUM_TRIALS_PER):
#     samps = vals[:,i]
#     mean = np.mean(samps)
#     std = np.std(samps)
#     tsamps = []
#     ttrials = []
#     for j,s in enumerate(samps):
#         err = np.abs(s-mean)/std
#         prob = 1-erf(err/np.sqrt(2))
#         if prob*len(samps) > 0.5:
#             tsamps.append(s)
#             ttrials.append(i+1)
#             masks[j].append(i)
#     means.append(np.mean(tsamps))
#     stds.append(np.std(tsamps))
#     # mp.plot(ttrials, tsamps,  '-o', alpha=.4)
#     print "Trial {0:s} has {1:d} subjects remaining".format(str(i+1), len(tsamps))
# for i,u in enumerate(users):
#     mp.plot(np.array(masks[i])+1, vals[i,masks[i]], '--o', alpha=.4)
# mp.errorbar(range(1,NUM_TRIALS_PER+1), means, stds,
#             marker='x', color='b', ecolor='b', markersize=12, mfc='None', mew = 1.25,
#             linewidth=2.0, linestyle='-', alpha=.75, zorder=1)
# # mp.plot([0,NUM_TRIALS_PER+1], [POSTURE_BASE_COST]*2 ,'--',linewidth=2, color='gray')
# hold(False)
# mp.xlim([0,NUM_TRIALS_PER+1])
# mp.ylim([0,0.71])
# mp.title(r'Shared Control Allocation $(f_{split})$', fontsize=12)


# # T3 stuff:
# mp.subplot(3,1,3)
# vals = np.zeros((len(users), NUM_TRIALS_PER))
# hold(True)
# for i,u in enumerate(users):
#     vals[i,:] = u.t3perrs
# means = []
# stds = []
# masks = {}
# for i in range(12): masks[i] = []
# for i in range(NUM_TRIALS_PER):
#     samps = vals[:,i]
#     mean = np.mean(samps)
#     std = np.std(samps)
#     tsamps = []
#     ttrials = []
#     for j,s in enumerate(samps):
#         err = np.abs(s-mean)/std
#         prob = 1-erf(err/np.sqrt(2))
#         if prob*len(samps) > 0.5:
#             tsamps.append(s)
#             ttrials.append(i+1)
#             masks[j].append(i)
#     means.append(np.mean(tsamps))
#     stds.append(np.std(tsamps))
#     # mp.plot(ttrials, tsamps,  '-o', alpha=.4)
#     print "Trial {0:s} has {1:d} subjects remaining".format(str(i+1), len(tsamps))
# for i,u in enumerate(users):
#     mp.plot(np.array(masks[i])+1, vals[i,masks[i]], '--o', alpha=.4)
# mp.errorbar(range(1,NUM_TRIALS_PER+1), means, stds,
#             marker='x', color='b', ecolor='b', markersize=12, mfc='None', mew = 1.25,
#             linewidth=2.0, linestyle='-', alpha=.75, zorder=1)
# # mp.plot([0,NUM_TRIALS_PER+1], [POSTURE_BASE_COST]*2 ,'--',linewidth=2, color='gray')
# hold(False)
# mp.xlim([0,NUM_TRIALS_PER+1])
# mp.ylim([0,0.71])
# mp.title(r'Human-Only Control Allocation $(f_{user})$', fontsize=12)

# mp.subplots_adjust(hspace=.25)
# fig.text(0.5, 0.04, 'Trial Number', ha='center', va='center', fontsize=14)
# fig.text(0.06, 0.5, 'Mean Posture Error [m]', ha='center', va='center', rotation='vertical', fontsize=14)
# mp.show()
# mp.close()
# mp.cla()
# mp.clf()



# f = open('./metric_summary.csv','w')
# for i,u in enumerate(users):
#     # task metrics
#     for v in u.t1errs:
#         out = "{0:f},".format(v)
#         f.write(out)
#     f.write('\r\n')
#     for v in u.t2errs:
#         out = "{0:f},".format(v)
#         f.write(out)
#     f.write('\r\n')
#     for v in u.t3errs:
#         out = "{0:f},".format(v)
#         f.write(out)
#     f.write('\r\n')
#     # posture metrics
#     for v in u.t1perrs:
#         out = "{0:f},".format(v)
#         f.write(out)
#     f.write('\r\n')
#     for v in u.t2perrs:
#         out = "{0:f},".format(v)
#         f.write(out)
#     f.write('\r\n')
#     for v in u.t3perrs:
#         out = "{0:f},".format(v)
#         f.write(out)
#     f.write('\r\n')
#     f.write('\r\n')
# f.close()

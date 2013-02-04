#!/usr/bin/python

# -*- coding: utf-8 -*-

import matplotlib as mpl
import sys

# lookup and use the backend_pgf module in the parent folder
mpl.use("module://backend_pgf")

# pdf backend with text.usetex
font_spec = {
    "text.usetex": True,
    "text.latex.unicode": True,
    "text.latex.preamble": [r"\usepackage{lmodern}",
                            r"\usepackage[T1]{fontenc}",
                            r"\usepackage{amsmath}"],
    "font.family": "serif",
    "font.serif": ["CMU Serif"],
    "font.sans-serif": ["CMU Sans Serif"],
    "font.monospace": [], # fallback to the default LaTeX monospace font
    }
# fnames_sets.append(["figure-pdf-usetex.pdf"])
# # use latex default fonts only
# font_spec = {"pgf.rcfonts": False,
#              "font.family": "serif",
#              # fonts specified in matplotlib are ignored
#              "font.serif": ["dont care"],
#              "font.sans-serif": ["me neither"],
#            }
mpl.rcParams.update(font_spec)
rc_text = {"font.family": "serif", "axes.labelsize": 11, "text.fontsize": 11,
           "legend.fontsize": 11, "xtick.labelsize": 11, "ytick.labelsize": 11}
mpl.rcParams.update(rc_text)
##############################################################################

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


# global constants:
base_dir = '/home/jarvis/ros/packages/kinect_pendulum_demo/data/'
NUM_TRIALS = 30
MAX_ANGLE = pi/6.0
BASE_COST = 602.0
TF = 6.0
DT = 0.02

def norm_func(dat):
    """
    This function is for actually calculating the norm of the data.  It is
    supposed to take in a dict from one of the mat files.
    """
    # configuration reference
    qd = np.array([0, dat['goal_offset'][0,0]])

    # now calculate the total error between q and qd throughout time horizon
    err = 0
    # for q in dat['q']:
    #     if np.abs(q[0]) > MAX_ANGLE:
    #         return -1.0
    for q in dat['q']:
        err += np.linalg.norm(q-qd)
    if err > BASE_COST:
        return -1.0
    return err



class User:
    def __init__(self, index, name, group):
        self.index = index
        self.name = name
        assert ( group == 'fixed' or group == 'training' )
        self.group = group
        self.data = None


    def fill_trials(self):
        """
        This function scans through the directory and adds a list of
        dictionaries to the class
        """
        self.data = []
        dir = os.path.join(base_dir, self.group, self.name)

        for i in range(NUM_TRIALS):
            fname = '{0:s}_{1:s}_trial_{2:d}.mat'.format(self.name,self.group,i)
            fname = os.path.join(dir,fname)
            try:
                dat = sio.loadmat(fname)
            except:
                print 'Could not locate file: \r\n',fname,'\r\n'
            self.data.append(dat)

    def cleanup_long_datasets(self):
        if self.data == None:
            return
        for i,dat in enumerate(self.data):
            if len(dat['q']) > 301:
                self.data[i]['q'] = dat['q'][-301:]
                self.data[i]['t'] = dat['t'][-301:]
                self.data[i]['u'] = dat['u'][-301:]


    def calc_last_few_errors(self):
        self.last_errs = []
        for i in range(NUM_TRIALS-5, NUM_TRIALS):
            dat = self.data[i]
            err = norm_func(dat)
            if err > 0:
                self.last_errs.append(err)

    def calc_all_errors(self):
        self.errs = []
        self.errs_trials = []
        for i in range(NUM_TRIALS):
            err = norm_func(self.data[i])
            if err > 0:
                self.errs.append(err)
                self.errs_trials.append(i)

            q = self.data[i]['q']
            if len(q) > 301:
                print "Data too long:",len(q)
                print "name = ",self.name
                print "trial = ",i,"\r\n"
                print "Final time = ",self.data[i]['t'][-1]




# def main():
# experimental definitions
users = [
    User(0, 'rab', 'training'),
    User(1, 'david', 'fixed'),
    User(2, 'lucie', 'training'),
    User(3, 'tommaso', 'fixed'),
    User(4, 'tim', 'training'),
    User(5, 'matt', 'fixed'),
    User(6, 'yoni', 'training'),
    User(7, 'anastasia', 'fixed'),
    User(8, 'lauren', 'training'),
    User(9, 'steven', 'fixed')
    ]
for u in users:
    u.fill_trials()
    u.cleanup_long_datasets()
    u.calc_last_few_errors()
    u.calc_all_errors()



###############################
# ANALYZING THE LAST 5 TRIALS #
###############################
fixvals = []
trainvals = []
fdat = []
tdat = []

fig = mp.figure()
mp.subplot(1,1,1)
mp.subplots_adjust(top=.97,bottom=.18,left=.18,right=.97)
hold(True)
for u in users:
    if len(u.last_errs) == 0:
        break
    if u.group == 'fixed':
        xvals = np.array([1]*len(u.last_errs))
        for e in u.last_errs: fixvals.append(e)
        # xvals = np.array([1]*1)
        # fixvals.append(min(u.last_errs))
        fdat.append([u.name, min(u.last_errs), np.mean(u.last_errs), np.std(u.last_errs)])
    elif u.group == 'training':
        xvals = np.array([2]*len(u.last_errs))
        for e in u.last_errs: trainvals.append(e)
        # xvals = np.array([2]*1)
        # trainvals.append(min(u.last_errs))
        tdat.append([u.name, min(u.last_errs), np.mean(u.last_errs), np.std(u.last_errs)])
    mp.plot(xvals, u.last_errs, 'o', markersize=10, alpha=.7)
    # mp.plot(xvals, min(u.last_errs), 'o', markersize=10, alpha=.7)

mp.errorbar(1, np.mean(fixvals), yerr=np.std(np.ravel(fixvals)),
            marker='x', color='k', ecolor='k', markersize=16, mfc='None', mew = 2.0,
            linewidth=2, linestyle='--', alpha=1)
mp.errorbar(2, np.mean(trainvals), yerr=np.std(np.ravel(trainvals)),
            marker='x', color='k', ecolor='k', markersize=16, mfc='None', mew = 2.0,
            linewidth=2, linestyle='--', alpha=1)
mp.grid(True)
mp.xlim([0,3])
mp.xticks([1,2], ['Fixed\nGroup', 'Trained\nGroup'])
ax = mp.gca()
for tick in ax.xaxis.get_major_ticks(): tick.label.set_fontsize(10)
# mp.title('All trials that never pass $\pm\pi/6$')
# mp.title('All trials that improved on base cost')
mp.ylabel('${L_2}$ Error', fontsize=10)
ax.tick_params(labelsize=10)
fig.set_size_inches(7.14*.45, 3/4.*(7.14*.45))
hold(False)


mp.savefig('pend_data.pdf')
mp.savefig('pend_data.pgf')

fdat.sort(key=lambda tup: tup[1])
tdat.sort(key=lambda tup: tup[1])

print "trained data summary:"
print "   Name   |   Best   |   Mean   |   StDev"
print "-----------------------------------------"
for d in tdat:
    print '{0:<10s}|{1:^10.3f}|{2:^10.3f}|{3:^10.3f}'.format(d[0],d[1],d[2],d[3])

print "\r\n\r\n"
print "fixed data summary:"
print "   Name   |   Best   |   Mean   |   StDev"
print "-----------------------------------------"
for d in fdat:
    print '{0:<10s}|{1:^10.3f}|{2:^10.3f}|{3:^10.3f}'.format(d[0],d[1],d[2],d[3])

# mp.show()
# mp.close()
# mp.cla()
# mp.clf()


# ############################
# # ANALYZING TIME HISTORIES #
# ############################
# hold(True)
# mp.subplot(2,1,1)
# for u in users:
#     if u.group == 'training':
#         mp.plot(u.errs_trials, u.errs, '-o')
# mp.title('Training data time history, all data')
# # mp.xlim([25, 30])
# mp.grid(True)

# mp.subplot(2,1,2)
# for u in users:
#     if u.group == 'fixed':
#         mp.plot(u.errs_trials, u.errs, '-o')
# mp.title('Fixed data time history, all data')
# # mp.xlim([25, 30])

# mp.grid(True)
# hold(False)

# mp.show()
# mp.close()
# mp.cla()
# mp.clf()

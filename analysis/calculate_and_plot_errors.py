#!/usr/bin/env python
import os
import sys
from math import sin, cos
from math import pi as mpi
import numpy as np
from matplotlib.pyplot import hold, plot
import matplotlib.pyplot as mp
import copy
import scipy.io as sio
import glob
import re
import angle_utils as au


# global constants:
base_dir = '/home/jarvis/ros/packages/kinect_pendulum_demo/data/'
NUM_TRIALS = 30



def norm_func(dat):
    """
    This function is for actually calculating the norm of the data.  It is
    supposed to take in a dict from one of the mat files.
    """
    # configuration reference
    qd = np.array([0, dat['goal_offset'][0,0]])

    # now calculate the total error between q and qd throughout time horizon
    err = 0
    for q in dat['q']:
        err += np.linalg.norm(q-qd)
    return err



class User:
    def __init__(self, index, name, group):
        self.index = index
        self.name = name
        assert ( group == 'fixed' or group == 'training' )
        self.group = group


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

    def calc_last_few_errors(self):
        self.last_errs = []
        for i in range(NUM_TRIALS-5, NUM_TRIALS):
            dat = self.data[i]
            err = norm_func(dat)
            self.last_errs.append(err)



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
    u.calc_last_few_errors()


fixvals = []
trainvals = []
fdat = []
tdat = []
hold(True)
for u in users:
    if u.group == 'fixed':
        xvals = np.array([1]*1)
        fixvals.append(min(u.last_errs))
        fdat.append([u.name, min(u.last_errs), np.mean(u.last_errs), np.std(u.last_errs)])
    elif u.group == 'training':
        xvals = np.array([2]*1)
        trainvals.append(min(u.last_errs))
        tdat.append([u.name, min(u.last_errs), np.mean(u.last_errs), np.std(u.last_errs)])
    mp.plot(xvals, min(u.last_errs), 'o')

mp.errorbar(1, np.mean(fixvals), yerr=np.std(fixvals),
            marker='o', color='k', ecolor='r', markersize=12,
            linewidth=2, linestyle='--')
mp.errorbar(2, np.mean(trainvals), yerr=np.std(trainvals),
            marker='o', color='k', ecolor='r', markersize=12,
            linewidth=2, linestyle='--')
hold(False)
mp.xlim([0,3])
# mp.ylim([0,3000])

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


mp.show()

mp.close()
mp.cla()
mp.clf()

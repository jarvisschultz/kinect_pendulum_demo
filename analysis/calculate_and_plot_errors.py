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


class User:
    def __init__(self, index, name, group):
        self.index = index
        self.name = name
        assert ( group == 'fixed' or group == 'training' )
        self.group = group

users = [
    User(0, 'Rab', 'training'),
    User(1, 'David', 'fixed'),
    User(2, 'Lucie', 'training'),
    User(3, 'Tommaso', 'fixed'),
    User(4, 'Tim', 'training'),
    User(5, 'Matt', 'fixed'),
    User(6, 'Yoni', 'training'),
    User(7, 'Anastasia', 'fixed'),
    User(8, 'Lauren', 'training'),
    User(9, 'Steven', 'fixed')
    ]


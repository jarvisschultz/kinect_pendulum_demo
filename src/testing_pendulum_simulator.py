#!/usr/bin/env python
####################
# IMPORTS:         #
####################
import sys
import os
import os.path
import math
import numpy as np
import scipy as sp
import trep
import trep.discopt as discopt
from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4.QtOpenGL import *
from OpenGL.GL import *
import copy
from math import fmod, pi, copysign
import threading
import argparse
import scipy.io as sio
import time
import itertools

####################
# GLOBAL VARIABLES #
####################
MAX_LINKS = 1
TIMESTEP = 20 # in ms
TF = 10.0 # total time of a trial in seconds
NUM_TRIALS_PER = 10 # how many trials are we going to do for each precalculated
                    #trust val
NUM_TRIALS = 4*NUM_TRIALS_PER
REF_POS = 1.0 # how far to the side we want to get the pendulum

# Weight limits:
MAX_WEIGHT = 1000000000
MIN_WEIGHT = 0.0001

# Trust limits:
MAX_ALPHA = 1.0
MIN_ALPHA = 0.0

# Predefined trust points:
T1 = 0.0
T2 = 80.0
T3 = 100.0

# Set to None or a specific (WIDTH, HEIGHT) size.  If None, we use a
# reasonable fraction of the desktop size.
DEFAULT_WINDOW_SIZE = (1440, 600)
# Center the window if not using fullscreen
os.environ['SDL_VIDEO_CENTERED'] = '1'

# Constants for states:
STATE_CONTROLLED_INTERACTIVE = 'state_controlled_interactive'


###########################
# MISCELLANEOUS FUNCTIONS #
###########################
# map to 0 to 2pi
def normalize_angle_positive(angle):
    return fmod(fmod(angle, 2.0*pi) + 2.0*pi, 2.0*pi)

# map to -pi to +pi
def normalize_angle(angle):
    a = normalize_angle_positive(angle)
    if a > pi: a -= 2.0*pi
    return a

# utility function for getting minimum angular distance between two angles in
# radians.  Answer will be -pi <= result <= pi, adding result to 'before' will
# always be an angle equivalent to 'after'
def shortest_angular_distance(before, after):
    result = normalize_angle_positive(normalize_angle_positive(after) -
                                      normalize_angle_positive(before))
    if result > pi:
        result = -(2.0*pi-result)
    return normalize_angle(result)


# trep system generator:
class Cart(trep.System):
    def __init__(self, num_links, link_length, link_mass, damping):
        super(Cart, self).__init__()

        self.num_links = num_links
        self.link_length = link_length
        self.link_mass = link_mass

        cart = trep.Frame(self.world_frame, trep.TX, 'cart-x', kinematic=True, name='cart')
        self._add_link(cart, 0)
        trep.potentials.Gravity(self, (0, -9.8, 0))
        trep.forces.Damping(self, damping)


    def _add_link(self, parent, link):
        if link == self.num_links:
            return
        base = trep.Frame(parent, trep.RZ, 'link-%d' % link, 'link-%d-base' % link)
        end  = trep.Frame(base, trep.TY, self.link_length,
                          mass=self.link_mass, name='link-%d-mass' % link)
        self._add_link(end, link+1)



def create_systems(max_links, link_length, link_mass, frequency, amplitude, damping):
    """
    Creates all of the carts and loads or generates their trajectories.
    """
    mvis = {}

    for num_links in range(1, max_links+1):
        print "Creating %d link cart" % num_links
        cart = Cart(num_links, float(link_length), float(link_mass), float(damping))
        mvis[num_links] = trep.MidpointVI(cart)

    return mvis


class RenderParameters():
    background_color = QColor(160, 160, 200)

    track_thickness = 0.1
    track_color = (0.4,0.4,0.4)

    cart_height = 0.15
    cart_width = 0.2
    cart_color = (0.2, 0.2, 0.2)
    ref_cart_color = (0.7, 0.1, 0.1)
    goal_trans = 0.55
    goal_cart_color = (0.2, 0.2, 0.2, goal_trans)
    goal_link_color = (0.8, 0.8, 0.8, goal_trans)

    link_width = 0.1
    link_color = (0.8, 0.8, 0.8)


class GLWidget(QGLWidget):
    def __init__(self):
        super(GLWidget, self).__init__()

        self.min_x = None
        self.max_x = None

        self.max_cart_height = 1.0

        self.cart_pos = None
        self.cart_ref_pos = None
        self.goal_ref_pos = None
        self.link_length = None
        self.link_frames = None
        self.desired_link_frames = None

        self.tracking_cart = False
        self.tracking_delta = 0.0
        self.tracking_last_pos = None


    def initializeGL(self):
        self.qglClearColor(RenderParameters.background_color)

    def resizeGL(self, width, height):
        glViewport(0,0, width, height)
        self.update_view_rect()

    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        self.draw_desired_links()
        self.draw_track()
        self.draw_links()
        self.draw_cart()
        self.draw_ref_cart()
        self.draw_goal_system()

    def set_max_cart_height(self, height):
        self.max_cart_height = height
        self.update_view_rect()

    def update_view_rect(self):
        self.min_y = -self.max_cart_height*0.2
        self.max_y = self.max_cart_height*1.1

        y_span = self.max_y - self.min_y
        x_span = float(self.width()) / self.height() * y_span
        self.min_x = -x_span/2.0
        self.max_x =  x_span/2.0

        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        glOrtho(self.min_x, self.max_x, self.min_y, self.max_y, -1.0, 1.0)
        glMatrixMode(GL_MODELVIEW)

    def draw_rect(self, rect, color):
        p0 = rect.topLeft()
        p1 = rect.topRight()
        p2 = rect.bottomRight()
        p3 = rect.bottomLeft()

        if len(color) > 3:
            glEnable (GL_BLEND);
            glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glColor4fv(color)
        else:
            glColor3fv(color)
        glBegin(GL_QUADS)
        glVertex2f(p0.x(), p0.y())
        glVertex2f(p1.x(), p1.y())
        glVertex2f(p2.x(), p2.y())
        glVertex2f(p3.x(), p3.y())
        glEnd()

    def draw_track(self):
        rect = QRectF()
        rect.setWidth(self.max_x - self.min_x)
        rect.setHeight(RenderParameters.track_thickness)
        rect.moveCenter( QPointF((self.max_x + self.min_x)/2, 0.0))
        self.draw_rect(rect, RenderParameters.track_color)

    def draw_cart(self):
        if self.cart_pos is None:
            return

        rect = QRectF()
        rect.setWidth(RenderParameters.cart_width)
        rect.setHeight(RenderParameters.cart_height)
        rect.moveCenter(self.cart_pos)
        self.draw_rect(rect, RenderParameters.cart_color)

    def draw_ref_cart(self):
        if self.cart_ref_pos is None:
            return

        rect = QRectF()
        rect.setWidth(0.75*RenderParameters.cart_width)
        rect.setHeight(0.75*RenderParameters.cart_height)
        rect.moveCenter(self.cart_ref_pos)
        self.draw_rect(rect, RenderParameters.ref_cart_color)

    def draw_goal_system(self):
        if self.goal_ref_pos is None:
            return
        if self.link_length is None:
            return
        # link
        rect = QRectF()
        rect.setHeight(self.link_length)
        rect.setWidth(RenderParameters.link_width)
        rect.moveCenter(QPointF(0.0, 0.0))
        rect.moveTop(0.0)
        # define transform:
        frame = np.diag((1,1,1,1))
        frame[0,3] = self.goal_ref_pos
        glPushAttrib(GL_POLYGON_BIT)
        glPushMatrix()
        glMultMatrixf(frame.transpose().flatten())
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
        self.draw_rect(rect, RenderParameters.goal_link_color)
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE)
        self.draw_rect(rect, (0.0, 0.0, 0.0, RenderParameters.goal_trans))
        glPopMatrix()
        glPopAttrib(GL_POLYGON_BIT)
        # cart
        rect = QRectF()
        rect.setWidth(RenderParameters.cart_width)
        rect.setHeight(RenderParameters.cart_height)
        rect.moveCenter(QPointF(self.goal_ref_pos, 0.0))
        self.draw_rect(rect, RenderParameters.goal_cart_color)




    def draw_links(self):
        if self.link_length is None:
            return
        if self.link_frames is None:
            return

        rect = QRectF()
        rect.setHeight(self.link_length)
        rect.setWidth(RenderParameters.link_width)
        rect.moveCenter(QPointF(0.0, 0.0))
        rect.moveTop(0.0)

        glPushAttrib(GL_POLYGON_BIT)
        for frame in self.link_frames:
            glPushMatrix()
            glMultMatrixf(frame.transpose().flatten())

            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
            self.draw_rect(rect, RenderParameters.link_color)
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE)
            self.draw_rect(rect, (0.0, 0.0, 0.0))
            glPopMatrix()
        glPopAttrib(GL_POLYGON_BIT)


    def draw_desired_links(self):
        if self.link_length is None:
            return
        if self.desired_link_frames is None:
            return

        rect = QRectF()
        rect.setHeight(self.link_length)
        rect.setWidth(RenderParameters.link_width)
        rect.moveCenter(QPointF(0.0, 0.0))
        rect.moveTop(0.0)

        glPushMatrix()
        try:
            scale = 0.3
            width = self.max_x - self.min_x
            height = self.max_y - self.min_y

            new_width = height*0.2 * scale
            new_height = height * scale


            dy = (height - new_height)
            dx = (width - new_width)/2

            glTranslatef(dx*0.8, dy*0.8, 0.0)
            glScalef(scale, scale, 1.0)
        except BaseException, e:
            print e
            glPopMatrix()
            return

        glPushAttrib(GL_POLYGON_BIT)
        for frame in self.desired_link_frames:
            glPushMatrix()
            glMultMatrixf(frame.transpose().flatten())

            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
            self.draw_rect(rect, RenderParameters.link_color)
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE)
            self.draw_rect(rect, (0.0, 0.0, 0.0))
            glPopMatrix()
        glPopAttrib(GL_POLYGON_BIT)
        glPopMatrix()

    def scale_mouse_delta(self, x):
        m = (self.max_x - self.min_x) / self.width()
        return x * m

    def mouseMoveEvent(self, event):
        if self.tracking_cart:
            delta = event.posF() - self.tracking_last_pos
            self.tracking_last_pos = event.posF()
            self.tracking_delta += self.scale_mouse_delta(delta.x())

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.tracking_cart = True
            self.tracking_last_pos = event.posF()

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.tracking_cart = False




class DemoWindow(QMainWindow):
    def __init__(self):
        super(DemoWindow, self).__init__()

        # define all arguments that can be passed in:
        parser = argparse.ArgumentParser()
        parser.add_argument("-n", "--name", required=True,
                            help="the test subject's name or identification")
        # parser.add_argument("-t", "--training",
        #                     help="Set to true means that we will modify the "\
        #                     "trust as the test progresses")
        parser.add_argument("-o", "--order", required=True, choices=range(1,7),
                            help="must pass this parameter to specify which "\
                            "order we proceed through the test with", type=int)
        parser.add_argument("-l", "--lefty",
                            help="set true if subject is left-handed")
        parser.add_argument("-i", "--index", type=int, default=0,
                            help="set the index to start the test on")
        parser.add_argument("-w", "--write", action='store_false', default=True,
                            help="pass arg to disable writing of data")
        args, unknown = parser.parse_known_args()

        # now we can process all of the args:
        # left-handed?
        if args.lefty in ['True', 'true', '1', 't', 'T']:
            print "Mirrored problem!"
            self.lefty = True
            self.offset_mag = -REF_POS
        else:
            self.lefty = False
            self.offset_mag = REF_POS
        # which index do we start on
        if args.index:
            self.index = int(args.index)
        else:
            self.index = 0
        # which order?
        self.order_index = args.order

        # name?
        self.username = args.name

        # are we writing out data?
        if args.write:
            subdir = "order_"+str(self.order_index)
            self.write_bool = True
            # let's create path for settings:
            base_dir = '/home/jarvis/ros/packages/kinect_pendulum_demo/data/'
            base_dir = os.path.join(base_dir, subdir)
            self.dir = os.path.join(base_dir, self.username)
            # check if directory exists, if not, create it:
            if os.path.exists(self.dir):
                print "Base directory already exists!"
            else:
                print "Directory does not exist, creating..."
                os.makedirs(self.dir)
        else:
            print "Not writing out data"
            self.write_bool = False
            
        # set vars related to interface:
        self.mouse_pos = -self.offset_mag
        self.alpha = 1.0
        self.trust = 0.0
        self.reset_flag = False
        self.qvec = []
        self.uvec = []
        self.tvec = []
        self.k = 0

        # vars related to systems:
        self.all_mvis = create_systems(MAX_LINKS,
                                       link_length='2.0',
                                       link_mass='60',
                                       frequency='0.5',
                                       amplitude='0.5',
                                       damping='0.01')
        self.num_links = -1
        self.iteration = -1
        self.link_choices = sorted(self.all_mvis.keys())
        # self.kin_weight = MIN_WEIGHT+ self.weight_slider.value()*(MAX_WEIGHT-MIN_WEIGHT)
        self.kin_weight = MAX_WEIGHT

        # vars related to display
        self.gl = GLWidget()
        self.setCentralWidget(self.gl)
        self.progress_bar = QProgressBar()
        self.statusBar().addPermanentWidget(self.progress_bar, 0.9)
        self.progress_bar.hide()
        self.setup_toolbars()
        self.create_actions()
        self.state = None
        self.select_cart(self.link_choices[0])

        # vars for timing:
        self.longstart = time.time()

        # start in the controlled-interactive mode:
        self.select_iteration_slot(0)
        self.state = STATE_CONTROLLED_INTERACTIVE

        # generate array for all f the permutations of weight order:
        arr = [T1,T2,T3]
        dat = {}
        for i,t in enumerate(itertools.permutations(arr)):
            dat[i] = list(t)
        self.torder = dat[self.order_index-1]
        arr = [-1]*NUM_TRIALS_PER
        for i in self.torder: arr.extend([i]*NUM_TRIALS_PER)
        self.trust_array = arr
        self.training_flag = True
        self.trust = self.trust_array[self.index]

        # initialize flags and trust
        if self.trust != -1:
            self.training_flag = False
            self.trust_slider.setValue(int(self.trust*10.0))
            self.trust_slider_released()

        self.update_status_message()
        self.startTimer(TIMESTEP)




    def setup_toolbars(self):

        self.toolbar = self.addToolBar("")
        self.toolbar.setFloatable(False)
        self.toolbar.setMovable(False)

        self.cart_combo = QComboBox()
        for i in self.link_choices:
            self.cart_combo.addItem("%d links" % i)
        self.toolbar.addWidget(self.cart_combo)
        self.cart_combo.currentIndexChanged.connect(self.select_cart_slot)

        self.iteration_combo = QComboBox()
        self.toolbar.addWidget(self.iteration_combo)
        self.iteration_combo.currentIndexChanged.connect(self.select_iteration_slot)

        self.reset_button = QPushButton("Reset")
        self.toolbar.addWidget(self.reset_button)
        self.reset_button.clicked.connect(self.reset_clicked)

        self.start_button = QPushButton("Start")
        self.toolbar.addWidget(self.start_button)
        self.start_button.clicked.connect(self.start_clicked)

        self.trust_slider = QSlider(Qt.Horizontal, self)
        self.toolbar.addWidget(self.trust_slider)
        self.trust_slider.setTickPosition(QSlider.TicksBothSides)
        self.trust_slider.setMaximumWidth(100)
        self.trust_slider.setMinimum(0)
        self.trust_slider.setMaximum(1000)
        self.trust_slider.setTickInterval(100)
        self.trust_slider.setFocusPolicy(Qt.NoFocus)
        self.trust_slider.connect(self.trust_slider, SIGNAL("sliderReleased()"), self.trust_slider_released)
        self.trust_slider.connect(self.trust_slider, SIGNAL("valueChanged(int)"), self.trust_slider_changed)
        self.trust_slider_label_gen = lambda k: "Trust Slider\r\n{0:3.1f}%".format(k)
        self.trust_label = QLabel(self.trust_slider_label_gen(self.trust_slider.value()/10.0), self)
        self.toolbar.addWidget(self.trust_label)

    def trust_slider_changed(self, value):
        self.trust_label.setText(self.trust_slider_label_gen(value/10.0))

    def trust_slider_released(self):
        self.trust = self.trust_slider.value()/10.0
        self.alpha = MAX_ALPHA - self.trust/100.0*(MAX_ALPHA-MIN_ALPHA)
        self.update_status_message()

    def create_actions(self):
        # shortcut for quiting:
        self.sh1 = QShortcut(self)
        self.sh1.setKey(QKeySequence(Qt.CTRL+Qt.Key_Q))
        self.connect(self.sh1, SIGNAL("activated()"), self.close)
        # shortcut for resetting integration
        self.sh3 = QShortcut(self)
        self.sh3.setKey(QKeySequence(Qt.CTRL+Qt.Key_R))
        self.connect(self.sh3, SIGNAL("activated()"), self.reset_clicked)
        # shortcut for starting integration
        self.sh4 = QShortcut(self)
        self.sh4.setKey(QKeySequence(Qt.CTRL+Qt.Key_S))
        self.connect(self.sh4, SIGNAL("activated()"), self.start_clicked)
        # shortcuts for controlling the iteration number:
        self.sh5 = QShortcut(self)
        self.sh5.setKey(QKeySequence(Qt.CTRL+Qt.Key_Up))
        self.connect(self.sh5, SIGNAL("activated()"), self.increment_trial)
        self.sh6 = QShortcut(self)
        self.sh6.setKey(QKeySequence(Qt.CTRL+Qt.Key_Down))
        self.connect(self.sh6, SIGNAL("activated()"), self.decrement_trial)


    def wheelEvent(self, event):
        if event.delta() > 0 :
            self.select_cart(self.num_links-1)
        else:
            self.select_cart(self.num_links+1)

    def increment_trial(self):
        if self.index < NUM_TRIALS-1:
            self.index += 1
        self.reset_clicked()

    def decrement_trial(self):
        if self.index > 0:
            self.index -= 1
        self.reset_clicked()

    def select_cart_slot(self, index):
        self.select_cart(self.link_choices[index])
        self.mouse_pos = -self.offset_mag


    def select_cart(self, num_links):
        if num_links not in self.link_choices:
            return
        if num_links == self.num_links:
            return

        self.num_links = num_links
        self.mvi = self.all_mvis[num_links]
        self.cart = self.mvi.system

        self.cart_combo.setCurrentIndex(self.link_choices.index(num_links))
        self.update_iteration_combo()

        self.gl.set_max_cart_height(self.max_cart_height())


    def update_iteration_combo(self):
        self.iteration_combo.clear()
        self.iteration_combo.addItem("Interactive w/ stabilization")
        self.select_iteration_slot(0)


    def reset_clicked(self):
        self.iteration = -1
        self.mouse_pos = -self.offset_mag
        self.select_iteration_slot(self.iteration_combo.currentIndex())
        self.reset_flag = False
        self.simulation_running = False
        self.simulation_completed = False
        # calculate current trust value:
        trust = self.trust_array[self.index]
        if trust == -1:
            self.trust = -1.0
            self.training_flag = True
            self.update_status_message()
        else:
            self.training_flag = False
            self.trust_slider.setValue(int(trust*10.0))
            self.trust_slider_released()


    def start_clicked(self):
        self.longstart = time.time()
        self.iteration = -1
        self.mouse_pos = -self.offset_mag
        self.select_iteration_slot(self.iteration_combo.currentIndex())
        self.reset_flag = False
        self.simulation_running = True
        self.simulation_completed = False


    def select_iteration_slot(self, index):
        self.start_controlled_interactive()


    def update_status_message(self):
        message = ''
        message += "Integration Time = %.1fs   |   " % (self.k*TIMESTEP/1000.0)
        message += "Iteration Number = %2d of %2d   |   " % (self.index + 1, NUM_TRIALS)
        message += "Current trust value = %3.1f   |   " % (self.trust)
        message += "Test subject: %s   |    " % self.username
        message += "User Control Group: "
        if self.trust == T1: message += "Low"
        elif self.trust == T2: message += "Medium"
        elif self.trust == T3: message += "High"
        else: message += "Training"
        self.statusBar().showMessage(message)


    def start_controlled_interactive(self):
        self.state = STATE_CONTROLLED_INTERACTIVE
        self.simulation_failed = False
        self.simulation_running = False
        self.simulation_completed = False
        self.iteration_combo.setCurrentIndex(1)
        self.progress_bar.hide()
        self.update_status_message()

        # calculate linearization, and feedback controller:
        tvec = np.arange(0, 500.0*TIMESTEP/1000.0, TIMESTEP/1000.0)
        qd = np.zeros((len(tvec), self.mvi.system.nQ))
        qd[:] = [0]*self.mvi.system.nQ
        rhod = np.zeros((len(tvec)-1, self.mvi.system.nQk))
        rhod[:] = [0]
        dsys = trep.discopt.DSystem(self.mvi, tvec)
        (Xd, Ud) = dsys.build_trajectory(Q=qd, rho=rhod)
        dyn_weight = 1
        kin_weight = self.kin_weight
        mom_weight = 0.01
        vel_weight = kin_weight*TIMESTEP/1000.0
        Q = np.diag(np.hstack(([dyn_weight]*self.cart.nQd, [kin_weight]*self.cart.nQk,
                               [mom_weight]*self.cart.nQd, [vel_weight]*self.cart.nQk)))
        R = np.diag([mom_weight])
        Qk = lambda k: Q
        Rk = lambda k: R
        (Kstab, A, B) = dsys.calc_feedback_controller(Xd, Ud, Qk, Rk, return_linearization=True)
        self.Kstab = Kstab[0]

        self.lerp_steps = 0
        self.lerp_max_steps = 3
        self.lerp_delta = 0.0
        self.gl.tracking_delta = 0.0
        self.k = 0

        Q = np.zeros(self.cart.nQ)
        Q[self.cart.get_config('cart-x').index] = -self.offset_mag
        p = np.zeros(self.cart.nQd)
        self.mvi.initialize_from_state(0.0, Q, p)

        self.disp_q = self.mvi.q2
        self.disp_qd = np.hstack((self.mvi.q2[0:self.cart.nQd],[0]*self.cart.nQk))


    def record_data(self):
        dat = {}
        print "Writing out data for trial", self.index
        dat['q'] = self.qvec
        dat['u'] = self.uvec
        dat['t'] = self.tvec
        dat['subject_name'] = self.username
        dat['lefty_bool'] = self.lefty
        dat['goal_offset'] = self.offset_mag
        dat['link_length'] = self.cart.link_length
        dat['link_mass'] = self.cart.link_mass
        dat['damping'] = self.cart.forces[0].get_damping_coefficient(self.cart.get_config(0))
        dat['trial_index'] = self.index
        dat['trust'] = self.trust
        fname = self.username + "_order_" + str(self.order_index)
        fname += "_trial_"+str(self.index)+".mat"
        fname = os.path.join(self.dir, fname)
        sio.savemat(fname, dat, appendmat=False)
        # reset all data:
        self.qvec = []
        self.uvec = []
        self.tvec = []
        if self.index < NUM_TRIALS -1:
            self.index += 1
        self.update_status_message()


    def update_interactive(self):
        if self.simulation_failed or not self.simulation_running:
            return

        self.lerp_steps += 1
        if self.lerp_steps == self.lerp_max_steps:
            self.lerp_steps = 0
            self.lerp_delta = self.gl.tracking_delta / 3.0
            self.gl.tracking_delta = 0.0

        rho = self.mvi.q2[self.cart.nQd:]
        rho[0] += self.lerp_delta

        ## if we are in interactive+control mode, let's run that controller:
        if self.state == STATE_CONTROLLED_INTERACTIVE:
            self.mouse_pos += np.array([self.lerp_delta])
            rho = 0
            # get state stuff
            qtmp = self.mvi.q2
            ptmp = self.mvi.p2
            if self.mvi.t2 != self.mvi.t1:
                vtmp = (self.mvi.q2-self.mvi.q1)/(self.mvi.t2-self.mvi.t1)
            else:
                vtmp = [0.0]*self.mvi.system.nQ
            vtmp = vtmp[self.mvi.system.get_config('cart-x').index]
            # wrap angle of pendulum:
            for q in self.cart.dyn_configs:
                q0 = qtmp[self.cart.get_config(q).index]
                qtmp[self.cart.get_config(q).index] = normalize_angle(q0)
            X = np.hstack((qtmp, ptmp, vtmp))
            X[self.mvi.system.get_config('cart-x').index] -= self.mouse_pos[0]

            # calculate feedback law:
            Xd = np.zeros(2*self.mvi.system.nQ)
            Xd[self.mvi.system.get_config('cart-x').index] = self.mouse_pos[0]
            rho = self.mouse_pos - self.alpha*np.dot(self.Kstab, X)
            rho_old = self.mvi.q2[self.cart.get_config('cart-x').index]

        try:
            self.k += 1
            t2 = self.k*TIMESTEP/1000.0
            self.mvi.step(t2, k2=rho)
            # self.statusBar().showMessage("Interactive... %.1fs" % t2)
        except trep.ConvergenceError:
            self.statusBar().showMessage("Simulation failed at %.1fs :(" % t2)
            self.simulation_failed = True
            return

        self.update_status_message()
        if self.state == STATE_CONTROLLED_INTERACTIVE:
            self.qvec.append(self.mvi.q2)
            self.uvec.append(rho)
            self.tvec.append(self.mvi.t2)
        if self.mvi.t2 > TF:
            self.simulation_running = False
            self.simulation_completed = True
            now = time.time()
            elapsed = now - self.longstart
            strout = "Total time elapsed = {0:6.4f}".format(elapsed)
            print strout
            if self.write_bool:
                self.record_data()


        self.disp_q = self.mvi.q2
        self.disp_qd = np.hstack((self.mvi.q2[0:self.cart.nQd],[0]*self.cart.nQk))



    def max_cart_height(self):
        return self.num_links * self.cart.link_length


    def timerEvent(self, event):
        self.update_interactive()
        self.update_display_info()
        self.gl.update()
        # do we need to reset?
        if self.reset_flag:
            self.reset_clicked()
            self.reset_flag = False


    def update_display_info(self):
        if self.disp_q is None:
            return

        self.cart.q = self.disp_q
        self.gl.cart_pos = QPointF(self.cart.get_config('cart-x').q, 0.0)
        self.gl.link_length = self.cart.link_length
        links = []
        for link in range(self.num_links):
            links.append(self.cart.get_frame('link-%d-base' % link).g())
        self.gl.link_frames = links

        self.cart.q = self.disp_qd
        links = []
        for link in range(self.num_links):
            links.append(self.cart.get_frame('link-%d-base' % link).g())
        self.gl.desired_link_frames = links

        self.gl.goal_ref_pos = self.offset_mag

        # set reference config:
        if self.state == STATE_CONTROLLED_INTERACTIVE:
            self.gl.cart_ref_pos = QPointF(self.mouse_pos, 0.0)
        else:
            self.gl.cart_ref_pos = None



def main():
    # application initialization:
    app = QApplication(sys.argv)
    demo = DemoWindow()
    demo.resize(*DEFAULT_WINDOW_SIZE)

    demo.show()
    sys.exit(app.exec_())


if __name__=='__main__':
    main()

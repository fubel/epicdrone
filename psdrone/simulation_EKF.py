from __future__ import division
from __future__ import print_function

import numpy as np
import matplotlib
matplotlib.rcParams['backend'] = "TkAgg"
import matplotlib.pyplot as plt
import time
import logging

from mpl_toolkits.mplot3d import proj3d
from matplotlib.patches import FancyArrowPatch
from scipy.stats import norm

import numpy as np
from abc import ABCMeta, abstractmethod

class ExtendedKalmanFilter(object):

    __metaclass__ = ABCMeta

    def __init__(self, n, m, pval=0.1, qval=1e-4, rval=0.1):
        '''
        Creates a KF object with n states, m observables, and specified values for
        prediction noise covariance pval, process noise covariance qval, and
        measurement noise covariance rval.
        '''

        # No previous prediction noise covariance
        self.P_pre = None

        # Current state is zero, with diagonal noise covariance matrix
        self.x = np.zeros((n, 1))
        self.P_post = np.eye(n) * pval

        # Get state transition and measurement Jacobians from implementing class
        self.F = self.getF(self.x)
        self.H = self.getH(self.x)

        # Set up covariance matrices for process noise and measurement noise
        self.Q = np.eye(n) * qval
        self.R = np.eye(m) * rval

        # Identity matrix will be usefel later
        self.I = np.eye(n)

    def step(self, z):
        '''
        Runs one step of the EKF on observations z, where z is a tuple of length M.
        Returns a NumPy array representing the updated state.
        '''

        # Predict ----------------------------------------------------

        # $\hat{x}_k = f(\hat{x}_{k-1})$
        self.x = self.f(self.x)

        # $P_k = F_{k-1} P_{k-1} F^T_{k-1} + Q_{k-1}$
        self.P_pre = self.F * self.P_post * self.F.T + self.Q

        # Update -----------------------------------------------------

        # $G_k = P_k H^T_k (H_k P_k H^T_k + R)^{-1}$
        G = np.dot(self.P_pre * self.H.T, np.linalg.inv(self.H * self.P_pre * self.H.T + self.R))

        # $\hat{x}_k = \hat{x_k} + G_k(z_k - h(\hat{x}_k))$
        self.x += np.dot(G, (np.array(z) - self.h(self.x).T).T)

        # $P_k = (I - G_k H_k) P_k$
        self.P_post = np.dot(self.I - np.dot(G, self.H), self.P_pre)

        # return self.x.asarray()
        return self.x

    @abstractmethod
    def f(self, x):
        '''
        Your implementing class should define this method for the state transition function f(x),
        returning a NumPy array of n elements.  Typically this is just the identity function np.copy(x).
        '''
        raise NotImplementedError()

    @abstractmethod
    def getF(self, x):
        '''
        Your implementing class should define this method for returning the n x n Jacobian matrix F of the
        state transition function as a NumPy array.  Typically this is just the identity matrix np.eye(n).
        '''
        raise NotImplementedError()

    @abstractmethod
    def h(self, x):
        '''
        Your implementing class should define this method for the observation function h(x), returning
        a NumPy array of m elements. For example, your function might include a component that
        turns barometric pressure into altitude in meters.
        '''
        raise NotImplementedError()

    @abstractmethod
    def getH(self, x):
        '''
        Your implementing class should define this method for returning the m x n Jacobian matirx H of the
        observation function as a NumPy array.
        '''


raise NotImplementedError()
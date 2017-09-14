from __future__ import division
from __future__ import print_function

import numpy as np
import matplotlib
matplotlib.rcParams['backend'] = "TkAgg"
from drone import Drone
from filterpy.kalman import KalmanFilter
import logging


def main():

    test_linearKF = KalmanFilter(dim_x=6, dim_z=3)

    # 1/60 second
    delta_t = 1.                        # 1 second

    test_linearKF.x = np.array([[1.],   # pos_x
                                [1.],   # pos_z
                                [1.],   # pos_y
                                [0.],   # vel_x
                                [0.],   # vel_z
                                [0.]])  # vel_y

    # state transition matrix
    test_linearKF.F = np.array([[1.,0.,0.,delta_t,0.,0.],
                                [0.,1.,0.,0.,delta_t,0.],
                                [0.,0.,1.,0.,0.,delta_t],
                                [0.,0.,0.,1.,0.,0.],
                                [0.,0.,0.,0.,1.,0.],
                                [0.,0.,0.,0.,0.,1.]])

    # measreument matrix
    test_linearKF.H = np.array([[1.,0.,0.,0.,0.,0.],
                                [0.,1.,0.,0.,0.,0.],
                                [0.,0.,1.,0.,0.,0.]])

    epic_drone = Drone(simulation=True)

    while True:
        velocity_drone = epic_drone.get_velocity()
        # todo calculate velocity relative to coordinate system

        measurement = np.array([[3.5],
                                [1.],
                                [2.]])

        test_linearKF.predict()
        test_linearKF.update(measurement)

        logging.info("%s"%test_linearKF.x)

    #test_linearKF.P *= 100.     # covariance matrix
    #test_linearKF.R = 5.        # state uncertainty
    #test_linearKF.Q = Q_discrete_white_noise(6, delta_t, .1)    # process uncertainty


    # todo add used sensors to drone
    #D = DummyDrone(np.array([3.5, 1., 2.]), 0)
    #EKF = Drone1DEKF()
    #observation = 0, 1.5, 10 # initial pos, value from accelerometer in m/s^2, inital speed in m/s
    #pos_new = EKF.step(observation)
    #logging.info("%s"%pos_new)

    # todo simulate movement
    # take estimation and new values from accelerometer and speedometer as input for next step

if __name__ == '__main__':
    main()
from __future__ import division
from __future__ import print_function
import logging
import numpy as np
import matplotlib
from resources.drone import Drone
from filterpy.kalman import KalmanFilter
import methods.velocity
matplotlib.rcParams['backend'] = "TkAgg"


class Velocity_Markerposition_KF(KalmanFilter):

    def __init__(self):
        super(Velocity_Markerposition_KF, self).__init__(dim_x=6, dim_z=6)

        delta_t = 1/60.                     # 1/60 seconds

        self.x = np.array([[1.],   # pos_x
                                    [1.],   # pos_z
                                    [1.],   # pos_y
                                    [0.],   # vel_x
                                    [0.],   # vel_z
                                    [0.]])  # vel_y

        # state transition matrix
        self.F = np.array([[1.,0.,0.,delta_t,0.,0.],
                                    [0.,1.,0.,0.,delta_t,0.],
                                    [0.,0.,1.,0.,0.,delta_t],
                                    [0.,0.,0.,1.,0.,0.],
                                    [0.,0.,0.,0.,1.,0.],
                                    [0.,0.,0.,0.,0.,1.]])

        # measreument matrix
        self.H = np.array([[1.,0.,0.,0.,0.,0.],
                                    [0.,1.,0.,0.,0.,0.],
                                    [0.,0.,1.,0.,0.,0.],
                                    [0.,0.,0.,1.,0.,0.],
                                    [0.,0.,0.,0.,1.,0.],
                                    [0.,0.,0.,0.,0.,1.]])

if __name__ == '__main__':
    test_linearKF = Velocity_Markerposition_KF()
    epic_drone = Drone(simulation=True)

    while True:
        velocity_norm = methods.velocity.position_by_velocity(epic_drone)

        measurement = np.array([[3.5],
                                [1.],
                                [2.],
                                [velocity_norm[0]],
                                [velocity_norm[1]],
                                [velocity_norm[2]]])

        test_linearKF.predict()
        test_linearKF.update(measurement)

        logging.info("%s" % test_linearKF.x)
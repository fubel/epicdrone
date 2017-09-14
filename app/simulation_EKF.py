from __future__ import division
from __future__ import print_function
import logging
import numpy as np
import matplotlib
from resources.drone import Drone
from filterpy.kalman import KalmanFilter
matplotlib.rcParams['backend'] = "TkAgg"


def main():

    test_linearKF = KalmanFilter(dim_x=6, dim_z=6)

    # 1/60 second
    delta_t = 1/60.                     # 1 second

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
                                [0.,0.,1.,0.,0.,0.],
                                [0.,0.,0.,1.,0.,0.],
                                [0.,0.,0.,0.,1.,0.],
                                [0.,0.,0.,0.,0.,1.]])

    epic_drone = Drone(simulation=True)

    while True:
        velocity_drone = epic_drone.get_velocity()
        rotation_matrix = epic_drone.get_rotation_matrix()

        velocity_norm = velocity_drone.dot(rotation_matrix)

        measurement = np.array([[3.5],
                                [1.],
                                [2.],
                                [velocity_norm[0]],
                                [velocity_norm[1]],
                                [velocity_norm[2]]])

        test_linearKF.predict()
        test_linearKF.update(measurement)

        logging.info("%s"%test_linearKF.x)

if __name__ == '__main__':
    main()
from __future__ import division
from __future__ import print_function
import logging
import numpy as np
import matplotlib
from resources.drone import Drone
from filterpy.kalman import KalmanFilter
import methods.velocity
matplotlib.rcParams['backend'] = "TkAgg"


class Location_KF(KalmanFilter):

    def __init__(self, timestep = 1/60.):
        super(Location_KF, self).__init__(dim_x=6, dim_z=6)

        delta_t = timestep

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
    delta_t = 1/60.0 #s
    drone_KF = Location_KF(delta_t)
    epic_drone = Drone(simulation=True)

    velocity_norm = methods.velocity.position_by_velocity(epic_drone, delta_t)

    # todo: get from aruco detection
    aruco_norm = np.array([3.5, 1., 2.])

    measurement = np.array([[aruco_norm[0]],
                            [aruco_norm[1]],
                            [aruco_norm[2]],
                            [velocity_norm[0]],
                            [velocity_norm[1]],
                            [velocity_norm[2]]])

    # uses previous state + time
    drone_KF.predict()

    logging.info("%s" % drone_KF.x)

    # uses aruco-detection + velocity values from sensor
    drone_KF.update(measurement)

    logging.info("%s" % drone_KF.x)
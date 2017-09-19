from __future__ import division
from __future__ import print_function
import logging
import numpy as np
import matplotlib
from resources.drone import Drone
from resources.common import Marker
from filterpy.kalman import KalmanFilter
from methods.velocity import measure_velocity, position_by_velocity
from methods.marker_detect import measure
matplotlib.rcParams['backend'] = "TkAgg"
from resources.world import World

class Location_KF(KalmanFilter):

    def __init__(self, timestep = 1/6., initial_position = None):
        super(Location_KF, self).__init__(dim_x=6, dim_z=10)

        delta_t = timestep

        self.x = np.r_[initial_position, np.array([0., 0., 0.])]

        # state transition matrix
        self.F = np.array([[1.,0.,0.,delta_t,0.,0.],
                            [0.,1.,0.,0.,delta_t,0.],
                            [0.,0.,1.,0.,0.,delta_t],
                            [0.,0.,0.,1.,0.,0.],
                            [0.,0.,0.,0.,1.,0.],
                            [0.,0.,0.,0.,0.,1.]])

        my = 0.5
        mx = mz = 3.75

        #self.B = np.array([[mx*delta_t, 0, 0],
        #                   [0, my*delta_t, 0],
        #                   [0, 0, mz*delta_t],
        #                   [0, 0, 0],
        #                   [0, 0, 0],
        #                   [0, 0, 0]])


        # measurement matrix
        self.H = np.array([[1., 0., 0., 0., 0., 0.],
                           [0., 1., 0., 0., 0., 0.],
                           [0., 0., 1., 0., 0., 0.],
                           [1., 0., 0., 0., 0., 0.],
                           [0., 1., 0., 0., 0., 0.],
                           [0., 0., 1., 0., 0., 0.],
                           [0., 0., 0., 1., 0., 0.],
                           [0., 0., 0., 0., 1., 0.],
                           [0., 0., 0., 0., 0., 1.],
                           [0., 1., 0., 0., 0., 0.]])

        # process noise
        self.Q = np.diag([0.5, 1.0, 0.5, 1.0, 1.0, 1.0])

        # measurement noise
        self.R = 8 * np.diag([0.2, 0.2, 0.2, 1.0, 1.0, 1.0, 1.3, 1.3, 1.3, 0.5])

        # covariance matrix
        self.P = np.diag([0.5, 0.5, 0.5, 1.0, 1.0, 1.0])


if __name__ == '__main__':
    world = World(simulation=False)
    world_dimensions = world.get_dimensions()
    markers = Marker.read('/home/cp/PycharmProjects/epicdrone/app/resources/room/markers.csv')
    world.set_markers(markers)
    print(markers)
    delta_t = 1 / 10.
    epic_drone = world.get_drone()
    drone_KF = Location_KF(delta_t, epic_drone.get_position())
    world.base_position_velocity = epic_drone.get_position()

    def sim_loop(world, task):
        if task.frame % 6 == 0:
            # Input orientation
            #u_raw = epic_drone.get_input_velocity()[0:3]
            #[h, p, r] = epic_drone.get_orientation()
            #u = np.array([u_raw[1]*np.sin(np.deg2rad(h)), u_raw[2], u_raw[0]*np.cos(np.deg2rad(h))])

            # prediction
            drone_KF.predict()

            # reset velocity measurement every few frames to avoid dead reckoning
            if task.frame % 300 == 0:
                world.base_position_velocity = drone_KF.x

            # collect measurements
            velocities = measure_velocity(epic_drone, delta_t)
            position_landmarks = measure(epic_drone, markers=markers)
            altitude = epic_drone.get_altitude()
            world.base_position_velocity = position_velo = position_by_velocity(epic_drone, 1./10, world.base_position_velocity)

            # process measurements
            if position_landmarks is None:
                position_landmarks = drone_KF.x[0:3]
            measurement = np.r_[position_landmarks, position_velo, velocities, altitude]

            # update with measurements
            drone_KF.update(measurement)
            logging.info("STATE: %s" % drone_KF.x)

            # set position for 3D visuals:
            epic_drone.set_position(drone_KF.x[0:3])


    world.hook_loop(sim_loop)
    world.run()
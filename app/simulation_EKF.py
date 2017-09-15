from __future__ import division
from __future__ import print_function
import logging
import numpy as np
import matplotlib
from resources.drone import Drone
from filterpy.kalman import KalmanFilter
from methods.velocity import measure_velocity, position_by_velocity
from methods.marker_detect import measure
matplotlib.rcParams['backend'] = "TkAgg"
from resources.world import World

class Location_KF(KalmanFilter):

    def __init__(self, timestep = 1/60., initial_position = None):
        super(Location_KF, self).__init__(dim_x=6, dim_z=9)

        delta_t = timestep

        self.x = np.r_[initial_position, np.array([0., 0., 0.])]

        # state transition matrix
        self.F = np.array([[1.,0.,0.,delta_t,0.,0.],
                            [0.,1.,0.,0.,delta_t,0.],
                            [0.,0.,1.,0.,0.,delta_t],
                            [0.,0.,0.,1.,0.,0.],
                            [0.,0.,0.,0.,1.,0.],
                            [0.,0.,0.,0.,0.,1.]])

        # measurement matrix
        self.H = np.array([[1.0, 0., 0., 0., 0., 0.],
                           [0., 1.0, 0., 0., 0., 0.],
                           [0., 0., 1.0, 0., 0., 0.],
                           [1.0, 0., 0., 0., 0., 0.],
                           [0., 1.0, 0., 0., 0., 0.],
                           [0., 0., 1.0, 0., 0., 0.],
                           [0., 0., 0., 1., 0., 0.],
                           [0., 0., 0., 0., 1., 0.],
                           [0., 0., 0., 0., 0., 1.]])


if __name__ == '__main__':
    world = World(simulation=False)
    world_dimensions = world.get_dimensions()
    markers = {
        1: [[1., 1.5, world_dimensions[1] - 0.01], [0., 0., 0.]],
        2: [[2., 1.5, world_dimensions[1] - 0.01], [0., 0., 0.]],
        3: [[3., 1.5, world_dimensions[1] - 0.01], [0., 0., 0.]],
        5: [[4., 1.5, world_dimensions[1] - 0.01], [0., 0., 0.]],
        6: [[5., 1.5, world_dimensions[1] - 0.01], [0., 0., 0.]],
    }
    delta_t = 1/60.0 #s
    epic_drone = world.get_drone()
    drone_KF = Location_KF(delta_t, epic_drone.get_position())


    def sim_loop(world, task):
        # uses previous state + time
        drone_KF.predict()

        velocities = measure_velocity(epic_drone, delta_t)
        position = measure(epic_drone, markers=markers)
        position_velo = position_by_velocity(epic_drone, 1./60)
        if position is None:
            position = drone_KF.x[0:3]
        measurement = np.r_[position, position_velo, velocities]


        logging.info("State: %s" % drone_KF.x)

        # uses aruco-detection + velocity values from sensor
        drone_KF.update(measurement)

        logging.info("%s" % drone_KF.x)

        epic_drone.set_position(drone_KF.x[0:3])


    world.hook_loop(sim_loop)

    world.run()
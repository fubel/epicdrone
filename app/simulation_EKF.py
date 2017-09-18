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

        my = 0.5
        mx = mz = 3.75

        self.B = np.array([[mx*delta_t, 0, 0],
                           [0, my*delta_t, 0],
                           [0, 0, mz*delta_t],
                           [0, 0, 0],
                           [0, 0, 0],
                           [0, 0, 0]])


        # measurement matrix
        self.H = np.array([[1., 0., 0., 0., 0., 0.],
                           [0., 1., 0., 0., 0., 0.],
                           [0., 0., 1., 0., 0., 0.],
                           [1., 0., 0., 0., 0., 0.],
                           [0., 1., 0., 0., 0., 0.],
                           [0., 0., 1., 0., 0., 0.],
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
        7: [[world_dimensions[0] - 0.01, 1.58, 1.04], [0., 0., 0.]],
        8: [[world_dimensions[0]-0.01, 1.58, 2.02], [0., 0., 0.]],
        9: [[world_dimensions[0]-0.01, 1.58, 2.64], [0., 0., 0.]],
        10: [[world_dimensions[0]-0.01, 1.58, 1.64], [0., 0., 0.]],
        11: [[world_dimensions[0]-0.01, 1.58, 0.67], [0., 0., 0.]],
        12: [[1.4, 0.86, 0.01], [0., 0., 0.]],
        13: [[2.4, 0.86, 0.01], [0., 0., 0.]],
        14: [[3.10, 0.86, 0.01], [0., 0., 0.]],
        15: [[world_dimensions[0] - 1.15, 1.50, 0.01], [0., 0., 0.]],
        16: [[0.01, 1.45, world_dimensions[1] - 2.27], [0., 0., 0.]],
        17: [[0.01, 1.46, world_dimensions[1] - 2.], [0., 0., 0.]],
        18: [[0.01, 1.47, world_dimensions[1] - 0.55], [0., 0., 0.]],
        19: [[0.40, 1.50, 1.30], [0., 0., 0.]],
    }
    delta_t = 1/60.0 #s
    epic_drone = world.get_drone()
    drone_KF = Location_KF(delta_t, epic_drone.get_position())
    world.base_position_velocity = epic_drone.get_position()

    def sim_loop(world, task):
        if task.frame % 6 == 0:
            # uses previous state + time
            u_raw = epic_drone.get_input_velocity()[0:3]
            [h, p, r] = epic_drone.get_orientation()
            u = np.array([u_raw[1]*np.cos(np.deg2rad(h)), u_raw[2], u_raw[0]*np.sin(np.deg2rad(h))])
            drone_KF.predict(u=u)

            if task.frame % 300 == 0:
                world.base_position_velocity = drone_KF.x
            velocities = measure_velocity(epic_drone, delta_t)
            position_landmarks = measure(epic_drone, markers=markers)
            world.base_position_velocity = position_velo = position_by_velocity(epic_drone, 1./6, world.base_position_velocity)

            print(position_velo, position_landmarks, velocities)
            if position_landmarks is None:
                position_landmarks = drone_KF.x[0:3]
            measurement = np.r_[position_landmarks, position_velo, velocities]

            logging.info("State: %s" % drone_KF.x)

            # uses aruco-detection + velocity values from sensor
            drone_KF.update(measurement)

            logging.info("%s" % drone_KF.x)

            epic_drone.set_position(drone_KF.x[0:3])


    world.hook_loop(sim_loop)

    world.run()
from __future__ import division

import numpy as np


def get_rotation_matrix(drone):
    heading, pitch, roll = map(np.deg2rad, drone.get_orientation())
    a, b, c = heading, pitch, roll

    rot_x = [np.cos(a) * np.cos(b), np.cos(a) * np.sin(b) * np.sin(c) - np.sin(a) * np.cos(c),
             np.cos(a) * np.sin(b) * np.cos(c) + np.sin(a) * np.sin(c)]
    rot_y = [np.sin(a) * np.cos(b), np.sin(a) * np.sin(b) * np.sin(c) + np.cos(a) * np.cos(c),
             np.sin(a) * np.sin(b) * np.cos(c) - np.cos(a) * np.sin(c)]
    rot_z = [-np.sin(a), np.cos(b) * np.sin(c), np.cos(b) * np.cos(c)]

    return np.array([rot_x, rot_y, rot_z])


def position_by_velocity(drone, delta_t, position):
    velocity_drone = drone.get_velocity()
    rotation_matrix = get_rotation_matrix(drone)

    velocity_norm = velocity_drone.dot(rotation_matrix)
    print velocity_norm
    return [
        position[0] + velocity_norm[0] / 1000 * delta_t,
        position[1] + velocity_norm[2] / 1000 * delta_t,
        position[2] - velocity_norm[1] / 1000 * delta_t]


def measure_velocity(drone, delta_t):
    velocity_drone = drone.get_velocity()
    rotation_matrix = get_rotation_matrix(drone)

    velocity_norm = velocity_drone.dot(rotation_matrix)
    print velocity_norm
    return [
        velocity_norm[0] / 100,
        velocity_norm[2] / 100,
        -velocity_norm[1] / 100]
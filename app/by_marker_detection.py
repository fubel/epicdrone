from __future__ import division

from resources.world import World

from methods.marker_detect import measure

import numpy as np

if __name__ == '__main__':

    world = World(simulation=False)
    world_dimensions = world.get_dimensions()

    # one marker in the middle of the four walls
    # world.set_default_markers()

    markers = {
        1: [[1., 1.5, world_dimensions[1] - 0.01], [0., 0., 0.]],
        2: [[2., 1.5, world_dimensions[1] - 0.01], [0., 0., 0.]],
        3: [[3., 1.5, world_dimensions[1] - 0.01], [0., 0., 0.]],
        5: [[4., 1.5, world_dimensions[1] - 0.01], [0., 0., 0.]],
        6: [[5., 1.5, world_dimensions[1] - 0.01], [0., 0., 0.]],
        7: [[world_dimensions[0] - 0.01, 0.8, world_dimensions[1] - 1.04], [0., 0., 0.]],
        8: [[world_dimensions[0] - 0.01, 0.8, world_dimensions[1] - 2.02], [0., 0., 0.]],
        9: [[world_dimensions[0] - 0.01, 0.8, 2.64], [0., 0., 0.]],
        10: [[world_dimensions[0] - 0.01, 0.8, 1.64], [0., 0., 0.]],
        11: [[world_dimensions[0] - 0.01, 0.8, 0.67], [0., 0., 0.]],
        12: [[1.4, 0.86, 0.01], [0., 0., 0.]],
        13: [[2.4, 0.86, 0.01], [0., 0., 0.]],
        14: [[3.10, 0.86, 0.01], [0., 0., 0.]],
        15: [[world_dimensions[0] - 1.15, 1.50, 0.01], [0., 0., 0.]],
        16: [[0.01, 1.45, world_dimensions[1] - 2.27], [0., 0., 0.]],
        17: [[0.01, 1.46, world_dimensions[1] - 2.], [0., 0., 0.]],
        18: [[0.01, 1.47, world_dimensions[1] - 0.55], [0., 0., 0.]],
        19: [[0.40, 1.50, 1.30], [0., 0., 0.]],
    }

    world.set_markers(markers)
    t = np.array([0.,0.,0.])
    def sim_loop(world, task):
        global t
        drone = world.get_drone()

        # drone.set_orientation( orientation_by_imu(drone) )
        #
        try:
            m = measure(drone, markers=markers)
            t = m
        except Exception as e:
            print(e)
            m = t
        print m
        if m is not None:
            drone.set_position(m)


    world.hook_loop(sim_loop)

    world.run()
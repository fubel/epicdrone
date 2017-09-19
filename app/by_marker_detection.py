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
        1: [[1., 1.5, world_dimensions[1] - 0.01], [0., 0., 0.], 'W'],
        2: [[2., 1.5, world_dimensions[1] - 0.01], [0., 0., 0.], 'W'],
        3: [[3., 1.5, world_dimensions[1] - 0.01], [0., 0., 0.], 'W'],
        5: [[4., 1.5, world_dimensions[1] - 0.01], [0., 0., 0.], 'W'],
        6: [[5., 1.5, world_dimensions[1] - 0.01], [0., 0., 0.], 'W'],
        7: [[world_dimensions[0] - 0.01, 1.2, 4.82], [0., 0., 0.], 'N'],
        8: [[world_dimensions[0] - 0.01, 1.2, 3.81], [0., 0., 0.], 'N'],
        9: [[world_dimensions[0] - 0.01, 1.2, 2.74], [0., 0., 0.], 'N'],
        10: [[world_dimensions[0] - 0.01, 1.2, 1.74], [0., 0., 0.], 'N'],
        11: [[world_dimensions[0] - 0.01, 1.2, 0.78], [0., 0., 0.], 'N'],
        12: [[1.4, 0.86, 0.01], [0., 0., 0.], 'E'],
        13: [[2.4, 0.86, 0.01], [0., 0., 0.], 'E'],
        14: [[3.10, 0.86, 0.01], [0., 0., 0.], 'E'],
        15: [[world_dimensions[0] - 1.15, 1.50, 0.01], [0., 0., 0.], 'E'],
        16: [[0.01, 1.45, 2.57], [0., 0., 0.], 'S'],
        17: [[0.01, 1.46, 3.85], [0., 0., 0.], 'S'],
        18: [[0.01, 1.47, 5.31], [0., 0., 0.], 'S'],
        19: [[0.40, 1.50, 1.32], [0., 0., 0.], 'S'],
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
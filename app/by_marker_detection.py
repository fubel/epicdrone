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
        1: [[1., 1.5, world_dimensions[1]-0.01], [0.,0.,0.]],
        2: [[2., 1.5, world_dimensions[1]-0.01], [0.,0.,0.]],
        3: [[3., 1.5, world_dimensions[1]-0.01], [0.,0.,0.]],
        5: [[4., 1.5, world_dimensions[1]-0.01], [0.,0.,0.]],
        6: [[5., 1.5, world_dimensions[1]-0.01], [0.,0.,0.]],
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
        drone.set_position(m)


    world.hook_loop(sim_loop)

    world.run()
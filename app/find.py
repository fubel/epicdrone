from __future__ import division

from resources.world import World
from methods.orientation import orientation_by_imu
from methods.velocity import position_by_velocity
from methods.marker_detect import measure, find_to_follow

import numpy as np

if __name__ == '__main__':

    world = World(simulation=False)
    marker = {
        29: [[0, 0, 0], [0, 0, 0], 'N']
    }
    world.set_markers(marker)
    drone = world.get_drone()
    #drone.set_position_by_input(True)

    world.delta_x = 15
    world.delta_y = 15
    world.next_move = [0, 0, 0, 0]

    def sim_loop(world, task):
        mv = world.next_move
        drone.move(mv[0], mv[1], mv[2], mv[3])

        # after startup, rotate drone
        m, c = find_to_follow(drone, marker)
        print(m, c)
        if m is not None:
            move_z = move_x = move_y = 0
            if m[0] > (320 + world.delta_x):
                move_x = 0.01
            elif m[0] < (320 - world.delta_x):
                move_x = -0.01
            else:
                move_x = 0
                if c < 50:
                    move_z = 0.02
                elif c > 60:
                    move_z = -0.02
                else:
                    move_z = 0
            if m[1] > (180 + world.delta_y):
                move_y = -0.1
            elif m[1] < (180 + world.delta_y):
                move_y = +0.1
            else:
                move_y = 0
                if c < 50:
                    move_z = 0.02
                elif c > 60:
                    move_z = -0.02
                else:
                    move_z = 0
            world.next_move = [move_x, move_z, move_y, 0]
            print("New move: %s" % world.next_move)
        else:
            world.next_move = [0, 0, 0, 0]

    world.hook_loop(sim_loop)

    world.run()
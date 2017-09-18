from __future__ import division

from resources.world import World
from methods.orientation import orientation_by_imu
from methods.input import position_by_velocity
from methods.velocity import get_rotation_matrix

from libs import xbox
import time
import numpy as np

if __name__ == '__main__':

    world = World(simulation=False, video=False)

    # one marker in the middle of the four walls
    world.set_default_markers()

    mz = 0.5
    mx = my = 3.75
    mh = 57.7

    active_input = -1
    next_input = 10 * 60
    user_inputs = [
        #[0, 0, 0, -0.2, 7.8],
        #[0, 0, 0.2, 0, 1.5],
        #[0, 0, 0, 0, 5],
        [0.8, 0, 0, 0, 0.5],
        [0, 0, 0, 0, 10],
        #[-0.1, 0, 0, 0, 1.0],
        #[0, 0, 0, 0, 1.0]
    ]

    def binit(world):
        drone = world.get_drone()
        drone.takeoff()
        drone.set_position([drone.get_position()[0], 0.85, drone.get_position()[2]])

    def run_input(index, drone):
        global active_input, next_input
        active_input = index
        next_input += int(user_inputs[index][4] * 60)
        drone.move(user_inputs[index][0], user_inputs[index][1], user_inputs[index][2], user_inputs[index][3])
        R = get_rotation_matrix(drone)
        drone_orientation = drone.get_orientation()
        drone.set_orientation([drone_orientation[0] - mh * user_inputs[index][3] * user_inputs[index][4],
                               drone_orientation[1],
                               drone_orientation[2]])
        print user_inputs[index]
        user_input_rot = np.array(user_inputs[index][0:3]).dot(R)
        print user_input_rot
        print drone.get_velocity()
        drone_pos = drone.get_position()
        drone.set_position([drone_pos[0] - my * user_input_rot[1] * user_inputs[index][4],
                           drone_pos[1] + mz * user_input_rot[2] * user_inputs[index][4],
                           drone_pos[2] + mx * user_input_rot[0] * user_inputs[index][4]])


    def sim_loop(world, task):
        drone = world.get_drone()
        if next_input == task.frame:
            if active_input + 1 == len(user_inputs):
                drone.move(0, 0, 0, 0)
            else:
                run_input(active_input + 1, drone)

    world.hook_init(binit)
    world.hook_loop(sim_loop)

    world.run()


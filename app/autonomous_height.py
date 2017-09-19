from __future__ import division

from resources.world import World
from methods.orientation import orientation_by_imu
from methods.velocity import position_by_velocity
from resources.drone import Drone


if __name__ == '__main__':

    world = World(simulation=False)

    # one marker in the middle of the four walls
    world.set_default_markers()

    drone = world.get_drone()

    # enables virtual control
    drone.set_position_by_input(True)

    def sim_loop(world, task):
        if drone.get_altitude() < 1.5:
            drone.psdrone.move(0., 0., .4, 0.)

        drone.psdrone.land()

        #print(drone.psdrone.NavData["demo"][2])
        pass

    world.hook_loop(sim_loop)

    world.run()
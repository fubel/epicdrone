from __future__ import division

from resources.world import World
from methods.orientation import orientation_by_imu
from methods.velocity import position_by_velocity


if __name__ == '__main__':

    world = World(simulation=False)

    # one marker in the middle of the four walls
    world.set_default_markers()

    def sim_loop(world, task):
        drone = world.get_drone()
        orienation = orientation_by_imu(drone)
        print orienation
        drone.set_orientation( orienation )
        drone.set_position( position_by_velocity(drone, 1./60.) )


    world.hook_loop(sim_loop)

    world.run()
from __future__ import division

from resources.world import World
from methods.orientation import orientation_by_imu
from methods.velocity import position_by_velocity


if __name__ == '__main__':

    world = World(simulation=False, video=False)

    # one marker in the middle of the four walls
    world.set_default_markers()

    def sim_loop(world, task):
        drone = world.get_drone()

        orienation = orientation_by_imu(drone)

        drone.set_orientation( orienation )
        drone.set_position( position_by_velocity(drone, 1./40., drone.get_position()) )

        print drone.get_position()
        print orienation

    world.hook_loop(sim_loop)

    world.run()
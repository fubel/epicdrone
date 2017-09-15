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
        drone.set_orientation( orientation_by_imu(drone) )
        drone.set_position( position_by_velocity(drone) )


    world.hook_loop(sim_loop)

    world.run()
from __future__ import division

from resources.world import World


if __name__ == '__main__':

    world = World(simulation=False)

    # one marker in the middle of the four walls
    world.set_default_markers()

    def sim_loop(world, task):
        drone = world.get_drone()
        v = drone.get_velocity()

        drone.position[1][0]+= v[0]/10E3*1/60
        drone.position[1][1]+= v[2]/10E3*1/60
        drone.position[1][2]+= v[1]/10E3*1/60


    world.hook_loop(sim_loop)

    world.run()
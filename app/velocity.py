from __future__ import division

from resources.world import World


if __name__ == '__main__':

    world = World(simulation=False)

    # one marker in the middle of the four walls
    world.set_default_markers()

    def sim_loop(world, task):
        drone = world.get_drone()
        v = drone.get_velocity()
        print v
        drone.position[0] += v[0]/1000/60
        drone.position[1] += v[2]/1000/60
        drone.position[2] += v[1]/1000/60


    world.hook_loop(sim_loop)

    world.run()
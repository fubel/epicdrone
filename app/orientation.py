from __future__ import division

from world import World


if __name__ == '__main__':

    world = World(simulation=True)

    # one marker in the middle of the four walls
    world.set_default_markers()

    def sim_loop(world, task):
        pass

    world.hook_loop(sim_loop)

    world.run()
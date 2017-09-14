import libs.xbox as xbox
from resources.world import World


if __name__ == '__main__':

    joy = xbox.Joystick()
    print "Controller initialized"

    unlocked = False

    def block(xbox_key):
        """ blocks the xbox key until it's released """
        while xbox_key():
            pass

    print "You have to startup the drone by pressing START"

    world = World(simulation=True)

    # one marker in the middle of the four walls
    world.set_default_markers()

    drone = world.get_drone()

    def sim_loop(world, task):
        global unlocked, drone

        if joy.Start():
            drone.startup()
            unlocked = True
            block(joy.Start)

        if unlocked:
            # takeoff:
            if joy.A():
                drone.takeoff()
                block(joy.A)

            # emergency:
            if joy.X():
                drone.emergency()
                block(joy.X)

            # reset:
            if joy.B():
                drone.reset()
                block(joy.B)

        (roll, throttle) = joy.leftStick()
        (yaw, pitch) = joy.rightStick()
        drone.move(roll, pitch, throttle, yaw)

        if joy.Back():
            world.stop()

    world.hook_loop(sim_loop)

    world.run()
    joy.close()








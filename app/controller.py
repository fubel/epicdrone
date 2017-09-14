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

    world = World(simulation=False)

    # one marker in the middle of the four walls
    world.set_default_markers()

    drone = world.get_drone()
    drone.set_position_by_input()

    def sim_loop(world, task):
        global unlocked, drone

        if joy.Back():
            world.stop()

        # takeoff:
        if joy.A():
            print "takeoff"
            drone.takeoff()
            block(joy.A)

        # emergency:
        if joy.X():
            print "emergency"
            drone.emergency()
            block(joy.X)

        # emergency:
        if joy.B():
            print "land"
            drone.land()
            block(joy.B)

        (roll, throttle) = joy.leftStick()
        (yaw, pitch) = joy.rightStick()
        print roll, pitch, throttle, yaw
        drone.move(roll, pitch, throttle, yaw)

    world.hook_loop(sim_loop)

    world.run()
    joy.close()








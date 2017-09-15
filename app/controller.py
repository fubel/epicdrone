import libs.xbox as xbox
from resources.world import World


if __name__ == '__main__':

    #joy = xbox.Joystick()
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

        # if joy.Back():
        #     world.stop()
        #
        # # takeoff:
        # if joy.A():
        #     print "takeoff"
        #     drone.takeoff()
        #     block(joy.A)
        #
        # # emergency:
        # if joy.X():
        #     print "emergency"
        #     drone.emergency()
        #     block(joy.X)
        #
        # # emergency:
        # if joy.B():
        #     print "land"
        #     drone.land()
        #     block(joy.B)
        #
        # (roll, throttle) = joy.leftStick()
        # (yaw, pitch) = joy.rightStick()
        # drone.move(roll, pitch, throttle, yaw)

        velocity_drone = drone.get_velocity()
        rotation_matrix = drone.get_rotation_matrix()

        velocity_norm = velocity_drone.dot(rotation_matrix)
        print velocity_norm
        drone.position[0] += velocity_norm[0]/1000/60
        drone.position[1] += velocity_norm[2]/1000/60
        drone.position[2] += velocity_norm[1]/1000/60

        key = drone.get_key()
        started = False
        if key == " ":
            if not started:
                drone.takeoff()
            else:
                drone.land()
        if key == "x":
            drone.emergency()
        elif key == "0":
            drone.hover()
        elif key == "c":
            drone.move(0., 0., 0., 0.)
        elif key == "w":
            drone.move(0., .2, 0., 0.)
        elif key == "s":
            drone.move(0., -.2, 0., 0.)
        elif key == "a":
            drone.move(-.2, 0., 0., 0.)
        elif key == "d":
            drone.move(.2, 0., 0., 0.)
        elif key == "q":
            drone.move(0., 0., 0., 0.2)
        elif key == "e":
            drone.move(0., 0., 0., -0.2)

    world.hook_loop(sim_loop)

    world.run()
    #joy.close()








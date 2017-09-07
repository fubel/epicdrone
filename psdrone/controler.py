import xbox
import ps_drone

#drone = ps_drone.Drone()
joy = xbox.Joystick()

class Drone():
    # dummy
    def __init__(self):
        pass

    def startup(self):
        print "started up"

    def takeoff(self):
        print "takeoff"

    def emergency(self):
        print "emergency"

    def reset(self):
        print "reset"


drone = Drone()
lock = False


def block(xbox_key):
    """ blocks the xbox key until it's released """
    while xbox_key():
        pass


while not joy.Back():
    # startup:
    if joy.Start():
        drone.startup()
        block(joy.Start)

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

    if joy.leftX():
        print joy.leftX()

joy.close()
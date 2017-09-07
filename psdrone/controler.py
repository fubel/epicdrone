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

drone = Drone()
lock = False


def on_click(xbox_key, drone_action):
    """
    Wrapper that takes an xbox key(e.g. joy.Back()), performs the drone action and then blocks the key
    Args:
        xbox_key: a function for an xbox.Joystick()
        drone_action: action to perform by the drone
    """
    if xbox_key():
        drone_action()
        while xbox_key():
            pass


while not joy.Back():
    on_click(joy.Start, drone.startup)
    on_click(joy.A, drone.takeoff)
    on_click(joy.X, drone.emergency)
joy.close()
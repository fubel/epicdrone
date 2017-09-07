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
safety = True

while not joy.Back():
    if joy.connected():
        print "Connected"
    else:
        print "Disconnected"
    if joy.X():
        if safety:
            drone.startup()
            safety = False
        else:
            drone.takeoff()
    if joy.B():
        drone.emergency()
        safety = True
joy.close()
from __future__ import division
from __future__ import print_function

import numpy as np
import logging
import ps_drone
import time

# logging settings:
logging.basicConfig(level=logging.INFO)

class Drone(object):
    def __init__(self, simulation=False):
        if (simulation==False):
            self.flag = False
            self.psdrone = ps_drone.Drone()
            self.psdrone.debug = True
            self.psdrone.startup()

            while (self.psdrone.getBattery()[0] == -1): time.sleep(0.1)  # Reset completed ?
            print ("Battery :" + str(self.psdrone.getBattery()[0]) + " % " + str(self.psdrone.getBattery()[1]))

            self.psdrone.useDemoMode(False)
            self.psdrone.addNDpackage(["demo"])
            self.psdrone.addNDpackage(["all"])

            timeCurrent = time.time()
            while True:
                if "magneto" in self.psdrone.NavData:
                    break
                if(time.time() - timeCurrent >= 5):
                    raise Exception("No NavData")


        def getPosition(self):
            self.psdrone

if __name__ == '__main__':
    my_drone = Drone()
    if my_drone.flag== False:
        while True:
            print(str(my_drone.psdrone.NavData["demo"][0][2]) + " " + " " + str(my_drone.psdrone.NavData["magneto"][10]) + " " + str(
                my_drone.psdrone.NavData["magneto"][11]))
from __future__ import division
from __future__ import print_function

import numpy as np
import logging
import ps_drone
import time

# logging settings:
logging.basicConfig(level=logging.INFO)

class Drone(object):
    def __init__(self, simulation=True):
        if simulation is False:
            self.psdrone = ps_drone.Drone()
            self.psdrone.debug = True
            self.psdrone.startup()

            while (self.drone.getBattery()[0] == -1): time.sleep(0.1)  # Reset completed ?
            print ('Battery :' + str(self.psdrone.getBattery()[0]) + " % " + str(self.psdrone.getBattery()[1]))

            self.psdrone.useDemoMode(False)
            self.psdrone.addNDpackage(["demo"])
            self.psdrone.addNDpackage(["all"])

    def getPosition(self):
        self.psdrone
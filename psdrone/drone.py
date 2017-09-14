from __future__ import division

import numpy as np
import logging
import ps_drone
import time
import math
# logging settings:
logging.basicConfig(level=logging.INFO)


class Drone(object):

    simulation = None
    position = [2, 1.5, 2] # x, z, y deault position somewhat center in the room
    orientation = [0, 0, 0] # heading, pitch, roll (angle in degrees)
    velocity = [0, 0, 0] # x,y,z in mm/s
    _ax = _ay = _az = 0.0
    _gx = _gy = _gz = 0.0

    def __init__(self, simulation=True):

        self.simulation = simulation
        if (self.simulation==False):
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

    def get_position(self):
        return self.position

    def get_orientation(self):
        if (self.simulation == False):
            gyro = self.psdrone.NavData["raw_measures"][1]  # x, y, z
            accelerometer = self.psdrone.NavData["raw_measures"][0]  # x, y, z

            print accelerometer, gyro

            self._ax = self._ay = self._az = 0.0

            # angles based on accelerometer
            self._ay = np.arctan2(accelerometer[1], np.sqrt(pow(accelerometer[0], 2) + pow(accelerometer[2], 2))) * 180. / np.pi
            self._ax = np.arctan2(accelerometer[0], np.sqrt(pow(accelerometer[1], 2) + pow(accelerometer[2], 2))) * 180. / np.pi
            # angles based on gyro(deg / s)
            self._gx += gyro[0] / 8.
            self._gy -= gyro[1] / 8.
            self._gz += gyro[2] / 8.

            # weighting both measurments
            self._gx = self._gx * 0.96 + self._ax * 0.04
            self._gy = self._gy * 0.96 + self._ay * 0.04

            self.orientation = [-self._ax, self._ay, self._az]

        return self.orientation

    def get_data_dump(self):
        return str(my_drone.psdrone.NavData["demo"][0][2]) + " " + " " + str(
            my_drone.psdrone.NavData["magneto"][10]) + " " + str(
            my_drone.psdrone.NavData["magneto"][11])

    def get_velocity(self):
        """returns velocity [v_x,v_y,v_z] in mm/s"""
        if self.simulation:
            return self.velocity
        else:
            return self.psdrone.NavData["demo"][4]


    def get_orientation_normvector(self):
        heading, pitch, roll = self.get_orientation()

        x = math.cos(heading) * math.cos(pitch)
        y = math.sin(heading) * math.cos(pitch)
        z = math.sin(pitch)

        return [x,y,z]


if __name__ == '__main__':
    my_drone = Drone(simulation=False)
    while True:
        #print(my_drone.get_data_dump())
        print my_drone.get_orientation_normvector()
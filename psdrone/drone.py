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
            accelerometer   = self.psdrone.NavData["raw_measures"][0]  # x, y, z
            gyro            = self.psdrone.NavData["raw_measures"][1]  # x, y, z

            # shitty hack to get it working
            gyro[0] -= 56
            gyro[1] -= 31
            gyro[2] += 27
            accelerometer = map(lambda x: (x-2040)/50.968399, accelerometer)

            self._ax = self._ay = self._az = 0.0

            # angles based on accelerometer
            self._ay = np.arctan2(accelerometer[1], np.sqrt(pow(accelerometer[0], 2) + pow(accelerometer[2], 2))) * 180. / np.pi
            self._ax = np.arctan2(accelerometer[0], np.sqrt(pow(accelerometer[1], 2) + pow(accelerometer[2], 2))) * 180. / np.pi

            for key in range(len(gyro)):
                if gyro[key] < 10 and gyro[key] > -10:
                    gyro[key] = 0
            print gyro

            # angles based on gyro(deg / s)
            self._gx += gyro[0] / 1030.
            self._gy -= gyro[1] / 1030.
            self._gz += gyro[2] / 1030.

            # weighting both measurments
            self._gx = self._gx * 0.96 + self._ax * 0.04
            self._gy = self._gy * 0.96 + self._ay * 0.04

            self.orientation = [self._gz, self._ax, self._ay]
            print self.orientation

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


    def get_rotation_matrix(self):
        heading, pitch, roll = map(np.deg2rad, self.get_orientation())
        a, b, c = heading, pitch, roll

        rot_x = [np.cos(a)*np.cos(b), np.cos(a)*np.sin(b)*np.sin(c)-np.sin(a)*np.cos(c), np.cos(a)*np.sin(b)*np.cos(c)+np.sin(a)*np.sin(c)]
        rot_y = [np.sin(a)*np.cos(b), np.sin(a)*np.sin(b)*np.sin(c)+np.cos(a)*np.cos(c), np.sin(a)*np.sin(b)*np.cos(c)-np.cos(a)*np.sin(c)]
        rot_z = [-np.sin(a), np.cos(b)*np.sin(c), np.cos(b)*np.cos(c)]

        return [rot_x, rot_y, rot_z]


if __name__ == '__main__':
    my_drone = Drone(simulation=True)
    print
    while True:
        print (np.array([1,1,1]) * my_drone.get_rotation_matrix())
        time.sleep(1)
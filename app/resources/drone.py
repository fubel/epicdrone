from __future__ import division

import numpy as np
import logging
import libs.ps_drone
import time
# logging settings:
logging.basicConfig(level=logging.INFO)


class Drone(object):

    simulation = None
    position = [2.5, 0., 2.7] # x, z, y deault position somewhat center in the room
    orientation = [0, 0, 0] # heading, pitch, roll (angle in degrees)
    velocity = [0, 0, 0] # x,y,z in mm/s
    input_velocity = [0, 0, 0, 0] # x,y,z,heading in mm/s
    calib_ok = False
    magneto = [0,0,0] # x, y, z
    _ax = _ay = _az = 0.0
    _gx = _gy = _gz = 0.0
    postion_by_input = False

    def __init__(self, simulation=True, video=True):
        # simulation flag
        self.simulation = simulation

        if not self.simulation:
            # general startup
            self.psdrone = libs.ps_drone.Drone()
            self.psdrone.debug = True
            self.psdrone.startup()
            self.psdrone.reset()
            self.psdrone.useDemoMode(False)
            self.psdrone.addNDpackage(["demo"])
            self.psdrone.addNDpackage(["all"])

            # connection
            timeCurrent = time.time()
            while (self.psdrone.getBattery()[0] == -1):
                if(time.time() - timeCurrent >= 5):
                    raise Exception("Drone not found")
                time.sleep(0.1)  # Reset completed ?
            print ("Battery :" + str(self.psdrone.getBattery()[0]) + " % " + str(self.psdrone.getBattery()[1]))

            # get nav data
            timeCurrent = time.time()
            while True:
                if "magneto" in self.psdrone.NavData:
                    break
                if (time.time() - timeCurrent >= 5):
                    raise Exception("No NavData")

            # start video stream (caution: we try HD here, that might not work well)
            if video:
                self.psdrone.setConfigAllID()
                self.psdrone.hdVideo()
                self.psdrone.frontCam()
                self.CDC = self.psdrone.ConfigDataCount
                while self.CDC == self.psdrone.ConfigDataCount:
                    time.sleep(0.0001)
                self.psdrone.startVideo()
                self.IMC = self.psdrone.VideoImageCount


    def startup(self):
        if not self.simulation:
            self.psdrone.startup()

    def takeoff(self):
        if not self.simulation:
            self.psdrone.takeoff()

    def emergency(self):
        if not self.simulation:
            self.psdrone.emergency()

    def mtrim(self):
        if not self.simulation:
            self.psdrone.mtrim()

    def land(self):
        if not self.simulation:
            self.psdrone.land()

    def move(self, leftright, backwardforward, downup, turnleftright):
        if not self.simulation:
            self.input_velocity = [leftright, backwardforward, downup, turnleftright]
            self.psdrone.move(leftright, backwardforward, downup, turnleftright)
        else:
            self.orientation[0] += -turnleftright
        if self.postion_by_input:
            self.position[0] += backwardforward * 0.05
            self.position[2] += leftright * 0.05
            self.position[1] += downup * 0.05

    def get_key(self,):
        return self.psdrone.getKey()

    def set_position_by_input(self, value=True):
        self.postion_by_input = value

    def get_raw(self):
        return self.psdrone.NavData

    def get_position(self):
        return self.position

    def get_orientation(self):
        return self.orientation

    def set_orientation(self, orientation):
        self.orientation = orientation
        return self

    def set_position(self, position):
        self.position = position
        return self

    def get_velocity(self):
        """returns velocity [v_x, v_y, v_z] in mm/s"""
        if self.simulation:
            return np.array(self.velocity)
        else:
            return np.array(self.psdrone.NavData["demo"][4])

    def get_input_velocity(self):
        return self.input_velocity

    def get_magneto(self):
        '''returns values from magentometer (x,y,z) and bool if calibration is ok'''
        if self.simulation:
            return np.append(np.array(self.magneto), np.array(self.calib_ok))
        else:
            raw_calib_ok = self.psdrone.NavData["magneto"][7]
            calib_ok = False
            if raw_calib_ok == 0:
                calib_ok== True
            return np.append(np.array(self.psdrone.NavData["magneto"][0]), np.array(calib_ok))

if __name__ == '__main__':
    my_drone = Drone(simulation=True)

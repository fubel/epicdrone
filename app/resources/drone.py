from __future__ import division

import numpy as np
import logging
import app.libs.ps_drone
import time
import cv2
import os
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
    postion_by_input = False

    def __init__(self, simulation=True):
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

            # start video stream (caution: we try HD here, that might not work well)
            self.psdrone.setConfigAllID()
            self.psdrone.hdVideo()
            self.psdrone.frontCam()
            self.CDC = self.psdrone.ConfigDataCount
            while self.CDC == self.psdrone.ConfigDataCount:
                time.sleep(0.0001)
            self.psdrone.startVideo()
            self.IMC = self.psdrone.VideoImageCount

            # get nav data
            timeCurrent = time.time()
            while True:
                if "magneto" in self.psdrone.NavData:
                    break
                if(time.time() - timeCurrent >= 5):
                    raise Exception("No NavData")

    def startup(self):
        if not self.simulation:
            self.psdrone.startup()

    def takeoff(self):
        if not self.simulation:
            self.psdrone.takeoff()

    def emergency(self):
        if not self.simulation:
            self.psdrone.emergency()

    def land(self):
        if not self.simulation:
            self.psdrone.land()

    def move(self, leftright, backwardforward, downup, turnleftright):
        if not self.simulation:
            self.psdrone.move(leftright, backwardforward, downup, turnleftright)
        else:
            self.orientation[0] += -turnleftright
        if self.postion_by_input:
            self.position[0] += backwardforward * 0.05
            self.position[2] += leftright * 0.05
            self.position[1] += downup * 0.05

    def set_position_by_input(self, value=True):
        self.postion_by_input = value

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

            # angles based on gyro(deg / s)
            self._gx += gyro[0] / 1030.
            self._gy -= gyro[1] / 1030.
            self._gz += gyro[2] / 1030.

            # weighting both measurments
            self._gx = self._gx * 0.96 + self._ax * 0.04
            self._gy = self._gy * 0.96 + self._ay * 0.04

            self.orientation = [self._gz, self._ax, self._ay]

        return self.orientation

    def get_data_dump(self):
        return str(my_drone.psdrone.NavData["demo"][0][2]) + " " + " " + str(
            my_drone.psdrone.NavData["magneto"][10]) + " " + str(
            my_drone.psdrone.NavData["magneto"][11])

    def get_velocity(self):
        """returns velocity [v_x,v_y,v_z] in mm/s"""
        if self.simulation:
            return np.array(self.velocity)
        else:
            return np.array(self.psdrone.NavData["demo"][4])

    def get_rotation_matrix(self):
        heading, pitch, roll = map(np.deg2rad, self.get_orientation())
        a, b, c = heading, pitch, roll

        rot_x = [np.cos(a) * np.cos(b), np.cos(a) * np.sin(b) * np.sin(c) - np.sin(a) * np.cos(c),
                 np.cos(a) * np.sin(b) * np.cos(c) + np.sin(a) * np.sin(c)]
        rot_y = [np.sin(a) * np.cos(b), np.sin(a) * np.sin(b) * np.sin(c) + np.cos(a) * np.cos(c),
                 np.sin(a) * np.sin(b) * np.cos(c) - np.cos(a) * np.sin(c)]
        rot_z = [-np.sin(a), np.cos(b) * np.sin(c), np.cos(b) * np.cos(c)]

        return np.array([rot_x, rot_y, rot_z])

    def capture_screen(self):
        """
        Captures the current image from drone video
        """
        if not self.simulation:
            # wait until next available video frame
            while self.psdrone.VideoImageCount == IMC:
                time.sleep(0.01)
            self.IMC = self.psdrone.VideoImageCount
            frame = self.psdrone.VideoImage
            return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        else:
            return None

    def measure(self):
        """
        Computes a measurement vector
        """
        # Todo: We have to discuss where to put this... Does this belong to the drone?
        if not self.simulation:
            measurements = {'tvecs': []}
            # get current image
            frame = self.capture_screen()

            # prepare aruco
            aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)
            parameters = cv2.aruco.DetectorParameters_create()

            # load calibrations
            mtx = np.load(os.path.join('resources', 'calibration', 'cam_broke_mtx.npy'))
            dist = np.load(os.path.join('resources', 'calibration','cam_broke_dist.npy'))
            rvecs = np.load(os.path.join('resources', 'calibration','cam_broke_rvecs.npy'))
            tvecs = np.load(os.path.join('resources', 'calibration','cam_broke_tvecs.npy'))

            # detection
            corners, ids, rejecected = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=paramters)
            # Todo: check if 25 cm is the correct thing here:
            rvecs, tvecs = cv2.aruco.estimatePoseSingleMarkers(corners, 25, mtx, dist, rvecs, tvecs)

            if ids:
                for i, id in enumerate(ids):
                    gray = cv2.aruco.drawAxis(frame, mtx, dist, rvecs[i], tvecs[i], 25)
                    measurements['tvecs'].append(tvecs[i])
                    logging.info("Landmark measurement found: %s" % tvecs[i])
            else:
                logging.info("No landmark measurement found")

            # fuse landmark measurements
            p = 1/len(measurements['tvecs']) * sum(measurements['tvecs'])

            # velocity measurement
            v = self.get_velocity()

            # complete measurement
            m = np.r_[p, v]

            return m

if __name__ == '__main__':
    my_drone = Drone(simulation=True)

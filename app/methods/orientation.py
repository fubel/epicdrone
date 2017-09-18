from __future__ import division

import numpy as np
# from libs.micropython_fusion.fusion import Fusion
# import time

_ax = _ay = _az = 0
_gx = _gy = _gz = 0

last_10 = [[],[],[]]


"""def calibrate(self, getxyz, stopfunc, wait=0):
    magmax = list(getxyz())  # Initialise max and min lists with current values
    magmin = magmax[:]
    while not stopfunc():
        if wait != 0:
            if callable(wait):
                wait()
            else:
                time.sleep(wait/1000)
        magxyz = tuple(getxyz())
        for x in range(3):
            magmax[x] = max(magmax[x], magxyz[x])
            magmin[x] = min(magmin[x], magxyz[x])
    self.magbias = tuple(map(lambda a, b: (a + b) / 2, magmin, magmax))"""


def orientation_by_imu(drone):

    return [drone.psdrone.NavData["demo"][2][2], drone.psdrone.NavData["demo"][2][0], drone.psdrone.NavData["demo"][2][1]]

    global _ax, _ay, _az
    global _gx, _gy, _gz
    global last_10

    accelerometer   = drone.get_raw()["raw_measures"][0]  # x, y, z
    gyro            = drone.get_raw()["raw_measures"][1]  # x, y, z
    magnetometer    = drone.get_raw()["magneto"][0]

    # shitty hack to get it working
    gyro[0] -= 56
    gyro[1] -= 31
    gyro[2] += 27
    accelerometer = map(lambda x: (x-2040)/50.968399, accelerometer)



    _ax = _ay = _az = 0

    # angles based on accelerometer
    _ay = np.arctan2(accelerometer[1], np.sqrt(pow(accelerometer[0], 2) + pow(accelerometer[2], 2))) * 180. / np.pi
    _ax = np.arctan2(accelerometer[0], np.sqrt(pow(accelerometer[1], 2) + pow(accelerometer[2], 2))) * 180. / np.pi


    for key in range(len(gyro)):
        if gyro[key] < 50 and gyro[key] > -50:
            gyro[key] = 0

    # angles based on gyro(deg / s)
    _gx -= gyro[1] / 1000.
    _gy += gyro[0] / 1000.
    _gz += gyro[2] / 1000.

    # weighting both measurments
    _gx = _gx * 0.96 + _ax * 0.04
    _gy = _gy * 0.96 + _ay * 0.04


    if(len(last_10[0])>10):
        last_10[0].pop(0)
        last_10[1].pop(0)
        last_10[2].pop(0)
    last_10[0].append(_gz)
    last_10[1].append(-_gy)
    last_10[2].append(_ax)
    return [
        sum(last_10[0])/len(last_10[0]),
        sum(last_10[1])/len(last_10[1]),
        sum(last_10[2])/len(last_10[2])]


from __future__ import division

import numpy as np


_ax = _ay = _az = 0
_gx = _gy = _gz = 0

def orientation_by_imu(drone):
    global _ax, _ay, _az
    global _gx, _gy, _gz

    accelerometer   = drone.NavData["raw_measures"][0]  # x, y, z
    gyro            = drone.NavData["raw_measures"][1]  # x, y, z

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
        if gyro[key] < 10 and gyro[key] > -10:
            gyro[key] = 0

    # angles based on gyro(deg / s)
    _gx += gyro[0] / 1030.
    _gy -= gyro[1] / 1030.
    _gz += gyro[2] / 1030.

    # weighting both measurments
    _gx = _gx * 0.96 + _ax * 0.04
    _gy = _gy * 0.96 + _ay * 0.04

    orientation = [_gz, _ax, _ay]

def orientation_by_imu(drone):
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
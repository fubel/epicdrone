from __future__ import division

import cv2
import time
import numpy as np
import os
import logging

logging.basicConfig(level=logging.INFO)

def capture_screen(drone):
    """
    Captures the current image from drone video
    """
    if not drone.simulation:
        # wait until next available video frame
        while drone.psdrone.VideoImageCount == drone.IMC:
            time.sleep(0.01)
        drone.IMC = drone.psdrone.VideoImageCount
        frame = drone.psdrone.VideoImage
        return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    else:
        return None


def measure(drone, markers=None):
    """
    Computes a measurement vector
    """
    measurements = []

    # get current image
    frame = capture_screen(drone)

    # prepare aruco
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)
    parameters = cv2.aruco.DetectorParameters_create()

    # load calibrations
    mtx = np.load(os.path.join('resources', 'calibration', 'cam_broke_mtx.npy'))
    dist = np.load(os.path.join('resources', 'calibration', 'cam_broke_dist.npy'))
    rvecs = np.load(os.path.join('resources', 'calibration', 'cam_broke_rvecs.npy'))
    tvecs = np.load(os.path.join('resources', 'calibration', 'cam_broke_tvecs.npy'))

    # detection
    corners, ids, _ = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

    rvecs, tvecs = cv2.aruco.estimatePoseSingleMarkers(corners, 0.176, mtx, dist, rvecs, tvecs)
    if type(ids) == np.ndarray:
        for i, id in enumerate(ids):
            id = id[0]
            gray = cv2.aruco.drawAxis(frame, mtx, dist, rvecs[i][0], tvecs[i][0], 0.176)
            tvecs[i][0][1] *= -1
            measurements.append(markers[id][0] - tvecs[i][0])
            logging.info("Measurement to %s found: %s" % (id, (markers[id][0]-tvecs[i][0])))
    else:
        logging.info("No landmark measurement found")
        return None

    #cv2.imshow('frame', frame)
    #cv2.waitKey(1)

    # fuse landmark measurements
    if len(measurements) == 0:
        p = [0, 0, 0]
    else:
        p = 1 / len(measurements) * sum(measurements)

    # complete measurement
    #m = np.r_[p, np.array([0.,0.,0.])]
    m = p
    logging.info("Measurement vector: %s" % m)
    return m
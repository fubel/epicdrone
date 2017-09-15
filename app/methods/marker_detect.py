import cv2
import time
import numpy as np
import os
import logging


def capture_screen(drone):
    """
    Captures the current image from drone video
    """
    if not drone.simulation:
        # wait until next available video frame
        while drone.psdrone.VideoImageCount == IMC:
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
    # todo: remove that
    if not markers:
        markers = {
            1: np.array([2.95, 1.65, 2.85]),
            3: np.array([2.05, 1.65, 2.85])
        }

    measurements = []

    # get current image
    frame = drone.capture_screen()

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

    rvecs, tvecs = cv2.aruco.estimatePoseSingleMarkers(corners, 0.2, mtx, dist, rvecs, tvecs)
    if ids:
        for i, id in enumerate(ids):
            id = id[0]
            gray = cv2.aruco.drawAxis(frame, mtx, dist, rvecs[i][0], tvecs[i][0], 0.2)
            measurements.append(markers[id] - tvecs[i][0])
            logging.info("Landmark measurement found: %s" % tvecs[i][0])
    else:
        logging.info("No landmark measurement found")

    # fuse landmark measurements
    p = 1 / len(measurements) * sum(measurements)

    # complete measurement
    m = np.r_[p, np.array([0.,0.,0.])]
    logging.info("Measurement vector: %s" % m)
    return m
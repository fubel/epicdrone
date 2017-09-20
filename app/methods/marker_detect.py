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

    rvecs, tvecs = cv2.aruco.estimatePoseSingleMarkers(corners, 0.10, mtx, dist, rvecs, tvecs)
    if type(ids) == np.ndarray:
        for i, id in enumerate(ids):
            id = id[0]
            frame = cv2.aruco.drawAxis(frame, mtx, dist, rvecs[i][0], tvecs[i][0], 0.10)
            tvecs[i][0][1] *= -1
            try:
                print('TVECS: %s' % tvecs[i][0])
                print('MARKERS: %s' % markers[id][0])
                if markers[id][2] == 'S':
                    j = [2, 1, 0]
                    M = markers[id][0] + tvecs[i][0][j]
                elif markers[id][2] == 'E':
                    M = markers[id][0] + tvecs[i][0]
                elif markers[id][2] == 'N':
                    j = [2, 1, 0]
                    M = markers[id][0] - tvecs[i][0][j]
                elif markers[id][2] == 'W':
                    M = markers[id][0] - tvecs[i][0]
                measurements.append(M)
                logging.info("Measurement to %s found: %s" % (id, (M)))
            except KeyError:
                pass
    else:
        logging.info("No landmark measurement found")
        return None

    cv2.imshow('frame', frame)
    cv2.waitKey(1)

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


def find_to_follow(drone, markers=None):
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
    marker_center = None
    rvecs, tvecs = cv2.aruco.estimatePoseSingleMarkers(corners, 0.10, mtx, dist, rvecs, tvecs)
    if type(ids) == np.ndarray:
        for i, id in enumerate(ids):
            try:
                id = id[0]
                if id == 29:
                    frame = cv2.aruco.drawAxis(frame, mtx, dist, rvecs[i][0], tvecs[i][0], 0.10)
                    c = corners[0][0]
                    x = ((c[0][0] + c[1][0])/2 + (c[3][0] + c[2][0])/2)/2
                    y = ((c[0][1] + c[3][1])/2 + (c[1][1] + c[2][1])/2)/2
                    marker_center = np.array([x, y])
                    true_center = np.array([320., 180.])
                    error = np.linalg.norm((marker_center-true_center)**2)
                    logging.info("Marker center estimation: %s " % marker_center)
                    logging.info("MSE: %s" % error)
                    dist = c[1][0] - c[0][0]
                    cv2.imshow('frame', frame)
                    cv2.waitKey(1)
                    return marker_center, dist
                else:
                    marker_center = None
                    dist = None
            except Exception as e:
                print(e)
                logging.error(str(e))

    else:
        logging.info("No landmark measurement found")
        marker_center = None
        dist = None

    cv2.imshow('frame', frame)
    cv2.waitKey(1)

    return marker_center, dist

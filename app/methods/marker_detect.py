def capture_screen(drone):
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


def measure(drone):
    """
    Computes a measurement vector
    """
    # Todo: We have to discuss where to put this... Does this belong to the drone?
    if not self.simulation:
        # todo: we need to know marker global position
        measurements = {'tvecs': []}
        # get current image
        frame = self.capture_screen()

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
        # Todo: check if 25 cm is the correct thing here:
        rvecs, tvecs = cv2.aruco.estimatePoseSingleMarkers(corners, 25, mtx, dist, rvecs, tvecs)
        if ids:
            for i, id in enumerate(ids):
                gray = cv2.aruco.drawAxis(frame, mtx, dist, rvecs[i][0], tvecs[i][0], 25)
                measurements['tvecs'].append(tvecs[i][0])
                logging.info("Landmark measurement found: %s" % tvecs[i][0])
        else:
            logging.info("No landmark measurement found")

        # fuse landmark measurements
        p = 1 / len(measurements['tvecs']) * sum(measurements['tvecs'])

        # velocity measurement
        v = self.get_velocity()

        # complete measurement
        m = np.r_[p, v]
        logging.info("Measurement vector: %s" % m)
        return m
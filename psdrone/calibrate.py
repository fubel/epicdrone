# Imports
import cv2
import numpy as np
import ps_drone
import time

# Termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Define object points for a 9x6 grid
objp = np.zeros((6 * 9, 3), np.float32)
objp[:, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2)
objp = objp * 26

# Arrays to store object points and image points
objpoints = []  # 3d point in real world space
imgpoints = []  # 2d points in image plane.

vd = cv2.VideoCapture(0)

# drone setup
drone = ps_drone.Drone()
drone.startup()
drone.reset()
while (drone.getBattery()[0]==-1):	time.sleep(0.1)
print "Battery: "+str(drone.getBattery()[0])+"%  "+str(drone.getBattery()[1])
drone.setConfigAllID()				# Go to multiconfiguration-mode
drone.sdVideo()						# Choose lower resolution (hdVideo() for...well, guess it)
drone.frontCam()					# Choose front view
CDC = drone.ConfigDataCount
while CDC == drone.ConfigDataCount:	time.sleep(0.0001)	# Wait until it is done (after resync is done)
drone.startVideo()					# Start video-function

IMC = 	 drone.VideoImageCount		# Number of encoded videoframes
stop =	 False
while not stop:
    while drone.VideoImageCount==IMC: time.sleep(0.01)	# Wait until the next video-frame
    IMC = drone.VideoImageCount
    key = drone.getKey()
    if key:		stop = True
    img  = drone.VideoImage					# Copy video-image
    # pImg = cv2.resize(img,(400,100))		# Process video-image
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, (9, 6), None)
    if ret:
        print("found")
        objpoints.append(objp)
        cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners)
        cv2.drawChessboardCorners(img, (9, 6), corners, ret)
        cv2.imshow('img', img)
        cv2.waitKey(500)
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints,
                                                   imgpoints, gray.shape[::-1],
                                                   None, None)

print "Camera calibration matrix\n\n", mtx
print "Camera dist, rvecs, tvecs", dist, rvecs, tvecs

# Save camera matrix and distortion coefficients to be used later
np.save('cam_broke_mtx', mtx)
np.save('cam_broke_dist', dist)
np.save('cam_broke_rvecs', rvecs)
np.save('cam_broke_tvecs', tvecs)

cv2.destroyAllWindows()

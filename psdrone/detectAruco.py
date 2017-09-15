import numpy as np
import cv2.aruco as aruco
import time, sys
import ps_drone													# Import PS-Drone-API
import cv2														# Import OpenCV

drone = ps_drone.Drone()										# Start using drone
drone.startup()													# Connects to drone and starts subprocesses

drone.reset()													# Sets drone's status to good (LEDs turn green when red)
while (drone.getBattery()[0]==-1):	time.sleep(0.1)				# Waits until drone has done its reset
print "Battery: "+str(drone.getBattery()[0])+"%  "+str(drone.getBattery()[1])	# Gives a battery-status
drone.useDemoMode(True)											# Just give me 15 basic dataset per second (is default anyway)

##### Mainprogram begin #####
drone.setConfigAllID()				# Go to multiconfiguration-mode
drone.sdVideo()						# Choose lower resolution (hdVideo() for...well, guess it)
drone.frontCam()					# Choose front view
CDC = drone.ConfigDataCount
while CDC == drone.ConfigDataCount:	time.sleep(0.0001)	# Wait until it is done (after resync is done)
drone.startVideo()					# Start video-function

##### And action !
IMC = 	 drone.VideoImageCount		# Number of encoded videoframes
stop =	 False

logs = []

while(True):
    # Capture frame-by-frame
    while drone.VideoImageCount == IMC: time.sleep(0.01)  # Wait until the next video-frame
    IMC = drone.VideoImageCount
    key = drone.getKey()
    if key:        stop = True
    frame = drone.VideoImage  # Copy video-image

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)
    parameters =  aruco.DetectorParameters_create()

    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    print(corners)
    gray = aruco.drawDetectedMarkers(gray, corners)

    # use calibration values to detect pose
    mtx = np.load('cam_broke_mtx.npy')
    dist = np.load('cam_broke_dist.npy')
    rvecs = np.load('cam_broke_rvecs.npy')
    tvecs = np.load('cam_broke_tvecs.npy')

    rvecs, tvecs = cv2.aruco.estimatePoseSingleMarkers(corners, 0.25, mtx, dist, rvecs, tvecs)

    marks = {
        1: np.array([2.95, 1.65, 2.85]),
        3: np.array([2.05, 1.65, 2.85])
    }
    if type(ids) == np.ndarray:
        merge = []
        for i, id in enumerate(ids):
            id = id[0]
            gray = cv2.aruco.drawAxis(gray, mtx, dist, rvecs[i], tvecs[i], 0.25)
            """
            Use Rodrigues to convert the landmarks rotation vector into a 
            rotation matrix R. If we invert R and reconvert it into a rotation vector,
            the result will be the rotation of the camera.
            Since R is orthogonal, R^(-1) = R^T, so we transpose instead of invert.
            """
            # initialize
            R = np.zeros((3, 3))
            camera_rotation = np.zeros((3, 3))
            # Convert rotation to rotation matrix
            R, _ = cv2.Rodrigues(rvecs[i][0])
            camera_rotation, _ = cv2.Rodrigues(R.transpose())
            # -R.t()*translation_vector;
            camera_translation = -R.transpose().dot(tvecs[i][0].transpose())
            # camera position with respect to the landmark
            v = camera_rotation + camera_translation
            print("Camera Rotation: %s" % camera_rotation)
            print("Camera Translation: %s" % camera_translation)
            print("TVECS: %s" % tvecs[i][0])
            print("APPROX: %s" % (marks[id] - tvecs[i][0]))


    cv2.imshow('frame',gray)
    if cv2.waitKey(1) and (drone.getKey() == ' '):
        break

# When everything done, release the capture
np.save('logs.npy', logs)
cv2.destroyAllWindows()
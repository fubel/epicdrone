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

    #print(parameters)

    '''    detectMarkers(...)
        detectMarkers(image, dictionary[, corners[, ids[, parameters[, rejectedI
        mgPoints]]]]) -> corners, ids, rejectedImgPoints
        '''
        #lists of ids and the corners beloning to each id
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    print(corners)

    gray = aruco.drawDetectedMarkers(gray, corners)

    #print(rejectedImgPoints)
    # Display the resulting frame
    cv2.imshow('frame',gray)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
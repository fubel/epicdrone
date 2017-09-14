import cv2
import numpy as np
import ps_drone
import time
import os
import logging

# logging settings:
logging.basicConfig(level=logging.INFO)

# directory to store images
directory = "screenshots"
stemp = 0

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
logging.info("Video started")

IMC = drone.VideoImageCount
while True:
    while drone.VideoImageCount == IMC: time.sleep(0.01)
    IMC = drone.VideoImageCount
    img = drone.VideoImage
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    cv2.imshow('img', gray)
    cv2.waitKey(1)
    if drone.getKey() == ' ':
        cv2.imwrite('screenshot/%s.png' % stemp, gray)
        stemp += 1
        logging.info("Screen captured and stored")
logging.info("Capture aborted")
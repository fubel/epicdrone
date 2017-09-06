"""
Anleitung:
1. Code ausfuehren
2. 9x6 Schachbrettmuster vor Webcam halten
3. So oft wie moeglich t druecken
4. Zum beenden s druecken
5. Ergebnis ist die Kalibrierungsmatrix 

"""

# Imports
import cv2
import numpy as np

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

while (True):

    ret, img = vd.read()
    cv2.imshow("Video cap", img)

    inp = cv2.waitKey(1)

    if inp == 115:  # If input is 's'
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (9, 6), None)

        # If found, add object points and image points
        if ret == True:
            print("found")
            objpoints.append(objp)

            # Refine image points
            cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners)

            # Draw and display the corners
            cv2.drawChessboardCorners(img, (9, 6), corners, ret)
            if ret == True:
                cv2.imshow('img', img)
                cv2.waitKey(500)

    elif inp == 116:
        break

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints,
                                                   imgpoints, gray.shape[::-1],
                                                   None, None)

print "Camera calibration matrix\n\n", mtx

# Save camera matrix and distortion coefficients to be used later
np.save('cam_broke_mtx', mtx)
np.save('cam_broke_dist', dist)

cv2.destroyAllWindows()
# Imports
import cv2
import numpy as np
import cv2
import glob

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

dim = (9, 7)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((dim[0]*dim[1],3), np.float32)
objp[:,:2] = np.mgrid[0:dim[1],0:dim[0]].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

images = glob.glob('screenshot/*.png')

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (dim[1],dim[0]),None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, (dim[1],dim[0]), corners2,ret)
        cv2.imshow('img',img)
        cv2.waitKey(500)

cv2.destroyAllWindows()


ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints,
                                                   imgpoints, gray.shape[::-1], None, None)

print "Camera calibration matrix\n\n", mtx
print "Camera dist, rvecs, tvecs", dist, rvecs, tvecs

# Save camera matrix and distortion coefficients to be used later
np.save('cam_broke_mtx', mtx)
np.save('cam_broke_dist', dist)
np.save('cam_broke_rvecs', rvecs)
np.save('cam_broke_tvecs', tvecs)

cv2.destroyAllWindows()

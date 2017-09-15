import numpy as np
import cv2
import cv2.aruco as aruco


cap = cv2.VideoCapture(0)

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    #print(frame.shape) #480x640
    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(cv2.aruco.DICT_6X6_1000)
    parameters =  aruco.DetectorParameters_create()

    #print(parameters)

    '''    detectMarkers(...)
        detectMarkers(image, dictionary[, corners[, ids[, parameters[, rejectedI
        mgPoints]]]]) -> corners, ids, rejectedImgPoints
        '''
        #lists of ids and the corners beloning to each id
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    #print(corners)

    #It's working.
    # my problem was that the cellphone put black all around it. The alrogithm
    # depends very much upon finding rectangular black blobs

    mtx = np.load('cam_broke_mtx.npy')
    dist = np.load('cam_broke_dist.npy')
   # rvecs = np.load('cam_broke_rvecs.npy')
   # tvecs = np.load('cam_broke_tvecs.npy')

    gray = aruco.drawDetectedMarkers(gray, corners)

    rvecs, tvecs = cv2.aruco.estimatePoseSingleMarkers(corners, 0.21, mtx, dist)

    if ids:
        for i, id in enumerate(ids):
            gray = cv2.aruco.drawAxis(gray, mtx, dist, rvecs[i], tvecs[i], 0.21)

            """
            Use Rodrigues to convert the landmarks rotation vector into a 
            rotation matrix R. If we invert R and reconvert it into a rotation vector,
            the result will be the rotation of the camera.
            Since R is orthogonal, R^(-1) = R^T, so we transpose instead of invert.
            """
            #initialize
            R = np.zeros((3, 3))
            camera_rotation = np.zeros((3, 3))
            # Convert rotation to rotation matrix
            R, _ = cv2.Rodrigues(rvecs[i][0])
            camera_rotation, _ = cv2.Rodrigues(R.transpose())
            # -R.t()*translation_vector;
            camera_translation = -R.transpose().dot(tvecs[i][0].transpose())

            print np.array([1.3, 1.0, 2.3]) - tvecs[i][0]


    #print(rejectedImgPoints)
    # Display the resulting frame
    cv2.imshow('frame',gray)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
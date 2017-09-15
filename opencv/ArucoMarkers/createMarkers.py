import cv2

idsToGenerate = range(5, 30)

for id in idsToGenerate:
    markerDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
    marker = cv2.aruco.drawMarker(markerDict, id, 500)


    cv2.imwrite("marker_#" + str(id) + ".png", marker)

    # display current marker
    #cv2.imshow('image',marker)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()

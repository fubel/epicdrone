#### Epic Drone Project

This is our awesome drone project :-)

#### Install openCV

Follow the instructions from ``opencv/tutorial.pdf``. If the pip package ``opencv-python`` is installed, video output won't work. In this case, you need to remove that package 

    pip uninstall opencv-python
   
After that, open the python console and import opencv:

    import cv2
    
Now you should be ready to get the drone video by using the ``psdrone/useVideo.py`` script.


### logs.npy

    import numpy as np

    data = np.load("logs.npy")

data contains an array with measurements per marker.
Every measurement consist of three objects:
First object is an array of x/y coordinates of the detected markers.
Second oject is rvecs and third is tvecs.

import cv2
import cv2.aruco as aruco
import numpy as np
from imutils.video import WebcamVideoStream
import imutils

import time
import os
import platform
import sys


#############################



# dimensions used when calibrating camera
#width = 640
#height = 480

# THESE NEED TO BE THE SAME AS WHAT YOU CALIBRATED WITH!!!!!!
# dimensions for optimal resolution (still 1.33 ratio)
width = 800
height = 600

cap = WebcamVideoStream(src = 0, height = height, width = width).start()

# set this to false if you want to allow the script to measure the frames per sec
viewVideo = True # to display marker
#viewVideo = False


if len(sys.argv) > 1:
    viewVideo = sys.argv[1]
    if viewVideo =='0' or viewVideo == 'False' or viewVideo == 'false':
        viewVideo = False


############ARUCO/CV2############


# aruco id
id_to_find = 72

# aruco size in cm
#marker_size = 24.6
marker_size = 47
#marker_size = 20 

# iterations/second are slower when the drone is flying. This accounts for that
realWorldEfficiency = .7 
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()

# default path
#calib_path = "/home/pi/video2calibration/calibrationFiles/"

# my path of stored calibration files
calib_path = "/home/sanperez/video2calibration/calibrationFiles/"

# load intrinsics from calibration files
cameraMatrix = np.loadtxt(calib_path+'cameraMatrix.txt', delimiter=',')
cameraDistortion = np.loadtxt(calib_path+'cameraDistortion.txt', delimiter=',')


#############################


# x11 
seconds = 0
if viewVideo == True:
    seconds = 1000000
    print("Showing video feed if X11 enabled.")
    print("Script will run until you exit.")
    print("-------------------------------")
    print("")
else:
    seconds = 5
counter = 0
counter = float(counter)

# detect aruco
start_time = time.time()
while time.time()-start_time < seconds:
    frame = cap.read() #for Threaded webcam
    
    # resize to calibration dimensions
#    frame = cv2.resize(frame,(width,height))
    
    # convert to gray scale and use cv methed
    frame_np = np.array(frame)
    gray_img = cv2.cvtColor(frame_np, cv2.COLOR_BGR2GRAY)
    ids=''
    corners, ids, rejected = aruco.detectMarkers(image=gray_img,dictionary=aruco_dict,parameters=parameters)
    
    # print the one we found
    if ids is not None:
        print("Found these IDs in the frame:")
        print(ids)

    # feed in aruco args and calibration params
    if ids is not None and ids[0] == id_to_find:

        # estimate the pose using the info from the calibration files,
        # then pull out distance info (x, y, z)
        ret = aruco.estimatePoseSingleMarkers(corners, marker_size, cameraMatrix = cameraMatrix, 
                                              distCoeffs = cameraDistortion)
        
        # distance (cm) from camera to marker, tvec = position
        rvec,tvec = ret[0][0,0,:], ret[1][0,0,:]
        x="{:.2f}".format(tvec[0])
        y="{:.2f}".format(tvec[1])
        z="{:.2f}".format(tvec[2])
        #print("FOUND ARUCO!")
        marker_position="MARKER POSITION: x = " + x + " y = " + y + " z = " + z
        print(marker_position)
        print("")

        # draw the marker and its axes (with x11 enabled) and display
        if viewVideo == True:
            
            aruco.drawDetectedMarkers(frame_np,corners)
            aruco.drawAxis(frame_np, cameraMatrix, cameraDistortion, rvec, tvec, 10)
            cv2.imshow('frame',frame_np)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    else:
        print("ARUCO "+str(id_to_find)+"NOT FOUND IN FRAME.")
        print("")
    counter=float(counter+1)

# report what the fps is (should be above ten)
if viewVideo == False:
    frequency=realWorldEfficiency*(counter/seconds)
    print("")
    print("")
    print("---------------------------")
    print("Loop iterations per second:")
    print(frequency)
    print("---------------------------")

    print("Performance Diagnosis:")
    if frequency > 10:
        print("Performance is more than enough for great precision landing.")
    elif frequency > 5:
        print("Performance likely still good enough for precision landing.")
        print("This resolution likely maximizes the detection altitude of the marker.")
    else:
        print("Performance likely not good enough for precision landing.")
        print("MAKE SURE YOU HAVE A HEAT SINK ON YOUR PI!!!")
    print("---------------------------")
cv2.destroyAllWindows()

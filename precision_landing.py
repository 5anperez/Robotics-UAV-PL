#!/usr/bin/python 
  
  ##############################################
################## DEPENDENCIES ##################
  ##############################################

import time
import socket
import exceptions
import math
import argparse

# OpenCV
import cv2
import cv2.aruco as aruco
import numpy as np
import imutils
from imutils.video import WebcamVideoStream

# DRONEKIT
from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
from pymavlink import mavutil




  ###########################################
################## VARIABLES ##################
  ###########################################

# ArUco of interest
id_to_find = 72

# The biggest marker: 47cm = 15.5in
marker_size = 47

# Original ArUco dictionary chosen to create a dictionary object
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()

# Camera resolution
horizontal_res = 640
vertical_res = 480

# imutils wrapper object, which is essentially cv2.VideoCapture, but this 
# library provides a threaded wrapper for more efficient frame capture. 
# src=0 grabs the first available camera and start begins the threaded vid cap.
cap = WebcamVideoStream(src=0, width=horizontal_res, height=vertical_res).start()

# Field of view for picam v2
horizontal_fov = 62.2 * (math.pi / 180 ) # NOTE: Pi cam V1: 53.5 & V2: 62.2
vertical_fov = 48.8 * (math.pi / 180)    # NOTE: Pi cam V1: 41.41 & V2: 48.8

# Calibration parameters
calib_path = "/home/sanperez/video2calibration/calibrationFiles/"
cameraMatrix = np.loadtxt(calib_path+'cameraMatrix.txt', delimiter=',')
cameraDistortion = np.loadtxt(calib_path+'cameraDistortion.txt', delimiter=',')


# Keeps a running total
found_count = 0
notfound_count = 0

# Used to set initial time of function to determine FPS
first_run = 0 
start_time = 0
end_time = 0

# Fully autonomous: set to 1 for arm and takeoff, 
# Semi autonomous: set to 2 for manual LOITER to GUIDED land 
script_mode = 1

#Set to 1 to trigger landing
ready_to_land = 0






  ##################################
############## FUNCTIONS #############
  ##################################


# Capable of launching and connecting to a virtual drone, or the real thing.
# If sitlInstance = True, then connect to sitl, o/w connect to RPi + Pixhawk.
def connectToDrone(sitlInstance):

    if sitlInstance:
        vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)
    else:
        vehicle = connect('/dev/ttyAMA0', baud = 57600, wait_ready = True)

    return vehicle
# connectDrone()



# Arms the drone and starts spinning the props, iff the drone
# passes all pre-flight safety checks and has a good gps signal.
def armDrone(vehicle):

    # Check if armable, which depends on pre-flight checks and gps lock.
    # If not armable, then run pre-flight checks again.
    if not vehicle.is_armable:

        # Run pre-flight checks.
        while not vehicle.is_armable:
            print "Waiting for drone to become armable..."
            time.sleep(1)
        
        # Passed all pre-flight checks.
        print "Vehicle is now armable!"

    # Once armable, arm i.e. start motors.
    print "Initiating arm stage..."
    vehicle.armed = True
    while not vehicle.armed:
        print "Waiting for drone to become armed..."
        time.sleep(1)
	
    print "Look out! Props are spinning!!"
    time.sleep(1)
    
    return None
# armDrone()




# Flight mode setter. Send your vehicle and a desired 
# flight mode and this method will enable it. 
def setMode(vehicle, flightMode):

    print "Entering", flightMode, "mode now..."
    
    vehicle.mode = VehicleMode(flightMode)
    while vehicle.mode != flightMode:
        print "Waiting to enter ", flightMode, " flight mode..."
        time.sleep(1)
    print "Vehicle is now in", flightMode, " mode!!"   

    return vehicle
# setMode()



# Simple lift-off method that utilizes the .alt attribute
# and the DK f(x) simple_takeoff. It sends an elevate 
# command and countinuously checks the attribute until
# the desired height is reached.
def takeoff(vehicle, targetHeight):

    # NOTE: Guided mode is required for most DK stuff
    setMode(vehicle, "GUIDED")
    armDrone(vehicle)
    
    # take off to the desired height
    vehicle.simple_takeoff(targetHeight)

    while True:
        # +/- .5 meters is an acceptable range
        acceptableHeight = targetHeight * 0.95
        currHeight = vehicle.location.global_relative_frame.alt
        print "Current Altitude: ", currHeight

        # Once within an acceptable height, terminate
        if currHeight >= acceptableHeight:
            break
        time.sleep(1)

    print "Target Altitude Reached!!"
    time.sleep(5)

    return None
# takeoffAndLand()




#*********************************************************************************
# NOTE: THIS F(X) HAS THE MOST UP-TO-DATE VELOCITY MSG THAT WORK! IT 
#       HAS BEEN TESTED, SO USE IT IN ALL SUBSEQUENT SCRIPTS.
#*********************************************************************************


# Sends a velocity message such that the arg's magnitude represents velocity and its parity
# represents direction about the corrisponding axis (uses the NED coords) in the drone's 
# (local) frame. Hack: It also enables yaw, but sets its rate (yaw_rate) to zero
# so that it doesnt yaw, unless fed a non-zero number.
# Usage: +vx for forward, -vx for backward,
#        +vy for right, -vy for left,
#        +vz for down, and -vz for up
def send_local_ned_velocity(vehicle, vx, vy, vz):
        
    # MAVLink message in the local/drone's frame, where forward is the drone's heading
	msg = vehicle.message_factory.set_position_target_local_ned_encode(
		0,                                          # time_boot_ms
		0, 0,                                       # target_system, target_component
		mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # coord_frame (local here)
		0b010111000111,                             # type_mask (decimal: 1479)
		0, 0, 0,                                    # position (x,y,z)
		vx, vy, vz,                                 # velocity (x,y,z)
		0, 0, 0,                                    # acceleration (x,y,z)
		0, 0                                        # yaw, yaw_rate
    )
        
	vehicle.send_mavlink(msg)
	vehicle.flush()
# send_local_ned_velocity()


#*********************************************************************************
# NOTE: THE ABOVE F(X) HAS THE MOST UP-TO-DATE VELOCITY MSG THAT WORK! IT 
#       HAS BEEN TESTED, SO USE IT IN ALL SUBSEQUENT SCRIPTS.
#*********************************************************************************






# Head north (+x), relative to drone
def moveForward(vehicle, velocity, duration):
    i = 0   # Each iteration moves the drone
    while (i < duration):
        send_local_ned_velocity(vehicle, velocity, 0, 0)
        print "Moving forward now!"
        time.sleep(1)
        i += 1



def forward_flight(vehicle, velocity, duration):

    print "Forward Flight engaging now..."
    print "Velocity:", velocity

    moveForward(vehicle, velocity, duration)

    print "Forward Flight finished!"
    print "Duration:", duration
    


def send_land_message(vehicle, x, y):
    msg = vehicle.message_factory.landing_target_encode(
        0,
        0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        x,
        y,
        0,
        0,
        0,)
    vehicle.send_mavlink(msg)
    vehicle.flush()
# send_land_message()



def lander(vehicle):

    print "Lander method engaged..."

    global first_run, notfound_count, found_count, marker_size, start_time
    
    if first_run == 0:
        print "First run of lander!!"
        first_run=1
        start_time=time.time()
        
    
    # Captures a single frame w/out the preceding bool
    frame = cap.read()

    ''' TEST THIS WHEN YOU GET A CHANCE!!! '''
    # Check the type for redundancy. If the output is 
    # <class 'numpy.ndarray'>, then the frame is already a NumPy array.
    print(type(frame))


    ''' TEST THIS WHEN YOU GET A CHANCE!!! '''
    # Check the resolution of the captured frame to see if 
    # it matches the desired resolution. The shape of the 
    # frame should be in the format (height, width, channels).
    # If the height and width match vertical_res and 
    # horizontal_res, respectively, then the resizing step is not needed.
    print(frame.shape)


    # This might be redundant since we already set this, 
    # or it might can act as a safegaurd to ensure its set.
    frame = cv2.resize(frame, (horizontal_res, vertical_res))

    # This might be redundant if the imutil and/or OpenCV 
    # are already NumPy arrays. To check, we will just 
    # print its data type after capturing a frame.
    frame_np = np.array(frame)

    
    # Convert the frame from BGR to grayscale for marker detection.
    gray_img = cv2.cvtColor(frame_np, cv2.COLOR_BGR2GRAY)

    '''
        Parameters like adaptiveThreshWinSizeMin, adaptiveThreshWinSizeMax, 
        and adaptiveThreshWinSizeStep control the window size and parameters 
        for this adaptive process.

        You can control how strictly the detector adheres to the expected marker size.
        minMarkerPerimeterRate and maxMarkerPerimeterRate set the allowable range of 
        marker perimeter sizes relative to the image size.
    '''
    
    # Detects ArUco markers in the grayscale image. 
    ids='' # This is probably redundant as well
    corners, ids, rejected = aruco.detectMarkers(image=gray_img, 
                                                 dictionary=aruco_dict, 
                                                 parameters=parameters)
    
    # Enable regular land mode
    # IDK WHAT THIS IS NEEDED FOR?!?!?
    # WE SHOULDNT BE TRYING TO LAND HERE!!
    # ITS PROB A DUMMY PLACEHOLDER LOL
    '''
    if vehicle.mode != 'LAND':

        vehicle.mode = VehicleMode("LAND")
        
        while vehicle.mode!='LAND':
            print('WAITING FOR DRONE TO ENTER LAND MODE')
            time.sleep(1)
    '''
    

    # Begin image processing
    try:

        # Check for the ArUco of interest (ArUco ID: 72)
        if ids is not None and ids[0] == id_to_find:

            # Estimates the 3D pose (rotation and translation) of the 
            # detected marker using camera calibration parameters 
            # (matrix and distortion) and the marker size. Another 
            # way to say rotation and translation is position and 
            # orientation.
            ret = aruco.estimatePoseSingleMarkers(corners, marker_size,             # positions of the markers corners in the img.
                                                  cameraMatrix = cameraMatrix,      # 3x3 intrinsic calibration matrix.
                                                  distCoeffs = cameraDistortion)    # distortion coefficients for the lens.
            

            ''' 
                The slicing [0, 0, :] accesses the first rotation vector. 
                
                The first 0 accesses the first set of rotation vectors 
                (assuming there could be multiple), the second 0 accesses 
                the first rotation vector within this set, and : selects 
                all the elements of this vector (all components of the 
                rotation vector).
            '''

            # Extracts the rotation vector (rvec) and translation vector 
            # (tvec) from the pose estimation results. Then, formats the 
            # position values as strings with two decimal places.
            (rvec, tvec) = (ret[0][0, 0, :], ret[1][0, 0, :])
            x = '{:.2f}'.format(tvec[0])
            y = '{:.2f}'.format(tvec[1])
            z = '{:.2f}'.format(tvec[2])
            
            y_sum = 0
            x_sum = 0
            
            x_sum = corners[0][0][0][0]+ corners[0][0][1][0]+ corners[0][0][2][0]+ corners[0][0][3][0]
            y_sum = corners[0][0][0][1]+ corners[0][0][1][1]+ corners[0][0][2][1]+ corners[0][0][3][1]
    
            x_avg = x_sum*.25
            y_avg = y_sum*.25
            
            x_ang = (x_avg - horizontal_res * .5) * (horizontal_fov / horizontal_res)
            y_ang = (y_avg - vertical_res * .5) * (vertical_fov / vertical_res)
            
            # Perform precision land w/ MAVLink msg
            if vehicle.mode != 'LAND':
                vehicle.mode = VehicleMode('LAND')
                while vehicle.mode!='LAND':
                    time.sleep(1)
                print("------------------------")
                print("Vehicle now in LAND mode")
                print("------------------------")
                send_land_message(vehicle, x_ang, y_ang)
            else:
                send_land_message(vehicle, x_ang, y_ang)
                pass
            
            print("X CENTER PIXEL: "+str(x_avg)+" Y CENTER PIXEL: "+str(y_avg))
            print("FOUND COUNT: "+str(found_count)+" NOTFOUND COUNT: "+str(notfound_count))
            print("MARKER POSITION: x=" +x+" y= "+y+" z="+z)

            found_count += 1
            print("")
        else:
            notfound_count=notfound_count+1
    except Exception as e:
        print('Target likely not found. Error: '+str(e))
        notfound_count=notfound_count+1
# lander()
    
     
 
 

##################################
 ############# MAIN #############
  #############################

if __name__=='__main__':


    # YOU SHOULD PUT A TRY-CATCH BLOCK HERE! (PER DK DOC)

    # set to false when field testing the real drone
    sitlInstance = False

    # connect to the drone
    vehicle = connectToDrone(sitlInstance)

    # Suite of flight modes
    flightMode0 = "STABILIZE"
    flightMode1 = "ALT_HOLD"
    flightMode2 = "LOITER"
    flightMode3 = "LAND"
    flightMode4 = "GUIDED"
    flightMode5 = "DISARM"
    flightMode6 = "RTL"

    # Enable stabilize mode
    setMode(vehicle, flightMode0)


    #*********************************************************************************
    # NOTE: THE BELOW SET OF PARAMETERS SEEM TO GUIDE THE DRONE IN THE VICINITY
    #       OF THE ARUCO MARKER. NOT DIRECTLY ABOVE, BUT WE WILL TRY THEM TO SEE
    #       IF THEY WORK I.E. ARE IN THE CAMERAS FOV.
    #*********************************************************************************
    desiredHeight = 4       # in meters
    desiredVelocity = 2     # m/s
    # Controls how long the movement lasts
    # NOTE: Set even numbers for simplicity
    desiredTravelDistance = 6

    # Elevate and hover
    takeoff(vehicle, desiredHeight)

    # Forward flight
    forward_flight(vehicle, desiredVelocity, desiredTravelDistance)



    # Precision landing parameters
    vehicle.parameters['PLND_ENABLED'] = 1      # Enable PL
    vehicle.parameters['PLND_TYPE'] = 1         # 1 for companion computer
    vehicle.parameters['PLND_EST_TYPE'] = 0     # 0 for raw sensor, 1 for kalman filter pos estimation
    vehicle.parameters['LAND_SPEED'] = 20       # Descend speed of 30cm/s


    # USAGE: 
    # 1. Fully autonomous arm and takeoff 
    # 2. Semi autonomous i.e. manual loiter to guided land
    if script_mode == 1:

        #arm_and_takeoff(takeoff_height)
        
        print(str(time.time()))
        
        #send_local_ned_velocity(velocity,velocity,0) ##Offset drone from target
        
        time.sleep(1)
        
        ready_to_land = 1

    elif script_mode == 2:
        
        while vehicle.mode != 'GUIDED':
            time.sleep(1)
            print("Waiting for manual change from mode "+str(vehicle.mode)+" to GUIDED")
        
        ready_to_land = 1


    if ready_to_land == 1:

        while vehicle.armed == True:
            lander(vehicle)

        end_time = time.time()
        total_time = end_time-start_time
        total_time = abs(int(total_time))

        total_count = found_count+notfound_count
        freq_lander = total_count/total_time
        print("Total iterations: "+str(total_count))
        print("Total seconds: "+str(total_time))
        print("------------------")
        print("lander function had frequency of: "+str(freq_lander))
        print("------------------")
        print("Vehicle has landed")
        print("------------------")

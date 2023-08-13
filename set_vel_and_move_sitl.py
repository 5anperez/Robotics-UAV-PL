#!/usr/bin/python


  #########################################
################## IMPORTS ##################
  #########################################

import time
import socket
import math
import os
import platform
import sys
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException






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
            print("Waiting for drone to become armable...")
            time.sleep(1)
        
        # Passed all pre-flight checks.
        print("Vehicle is now armable!")

    # Once armable, arm i.e. start motors.
    print("Initiating arm stage...")
    vehicle.armed = True
    while not vehicle.armed:
        print("Waiting for drone to become armed...")
        time.sleep(1)
	
    print("Look out! Props are spinning!!")
    time.sleep(1)
    
    return None
# armDrone()



# Flight mode setter. Send your vehicle and a desired 
# flight mode and this method will enable it. 
def setMode(vehicle, flightMode):

    print("Entering", flightMode, "mode now...")
    
    vehicle.mode = VehicleMode(flightMode)
    while vehicle.mode != flightMode:
        print("Waiting to enter", flightMode, "flight mode...")
        time.sleep(1)
    print("Vehicle is now in", flightMode, "mode!!")   

    return vehicle
# setMode()



# Simple lift-off method that utilizes the .alt attribute
# and the DK f(x) simple_takeoff. It sends an elevate 
# command and countinuously checks the attribute until
# the desired height is reached.
def takeoff(vehicle, targetHeight):

    
    # vehicle.mode = VehicleMode('GUIDED')

    # IDK WHY THE FOLLOWING 'ARM' BLOCK IS THERE. THE DRONE 
    # SHOULD ALREADY BE ARMED WHEN WE GET HERE, NO?
    # TYPE HERE WHEN YOU REMEMBER!

    '''
    if not vehicle.is_armable:
        # begin the arming phase
        while not vehicle.is_armable:
            print("Waiting for drone to become armable...")
            time.sleep(1)
        print("Vehicle is now armable!")
    '''

    # NOTE: Guided mode is required for most DK stuff
    setMode(vehicle, "GUIDED")
    armDrone(vehicle)
    
    # take off to the desired height
    vehicle.simple_takeoff(targetHeight)

    while True:
        
        # +/- .5 meters is an acceptable range
        acceptableHeight = targetHeight * 0.95
        currHeight = vehicle.location.global_relative_frame.alt
        print("Current Altitude: ", currHeight)

        # Once within an acceptable height, terminate
        if currHeight >= acceptableHeight:
            break
        time.sleep(1)

    print("Target Altitude Reached!!")
    time.sleep(5)

    return None
# takeoffAndLand()



# send velocity message (movement commands) in the drone's frame
def send_local_ned_velocity(vehicle, vx, vy, vz):
        
    # MAVLink message in the local/drone's frame, where forward is the drone's heading
	msg = vehicle.message_factory.set_position_target_local_ned_encode(
		0,                                          #
		0, 0,                                       #
		mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # local frame
		0b0000111111000111,                         # velocity bitmask
		0, 0, 0,                                    # position
		vx, vy, vz,                                 # velocity
		0, 0, 0,                                    # acceleration
		0, 0
    )
        
	vehicle.send_mavlink(msg)
	vehicle.flush()
# send_local_ned_velocity()



# send velocity message (movement commands) in a global frame
def send_global_ned_velocity(vehicle, vx, vy, vz):

    # MAVLink message in the global frame, where forward is true North
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,                                   #
        0, 0,                                #
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # global frame
        0b0000111111000111,                  # type_mask w/ only speed enabled
        0, 0, 0,                             # position
        vx, vy, vz,                          # velocity
        0, 0, 0,                             # acceleration
        0, 0                                 # yaw_rate
    )

    vehicle.send_mavlink(msg)
    vehicle.flush()
# send_global_ned_velocity()


# NOTE: velocity commands must be resent every one second to keep
# the drone in constant motion. sending one velocity message will
# only move the drone for one second.


# NOTE: if 'RNG' in the SITL console is greyed out, then it hasnt
# been enabled! to enable, do: 
#           param set RNGFND1_TYPE 100
#           reboot
# in the terminal running MAVProxy. it should turn green


# head north (+x), relative to drone
def moveForward(vehicle):
    i = 0   # each iteration moves the drone
    while (i < 12):
        send_local_ned_velocity(vehicle, 1, 0, 0)
        print("Moving forward now!")
        time.sleep(1)
        i += 1

# head south (-x), relative to drone
def moveBackwards(vehicle):
    i = 0   # each iteration moves the drone
    while (i < 6):
        send_local_ned_velocity(vehicle, -1, 0, 0)
        print("Moving backwards now!")
        time.sleep(1)
        i += 1

# head west (-y), relative to drone
def moveLeft(vehicle):
    i = 0   # each iteration moves the drone
    while (i < 6):
        send_local_ned_velocity(vehicle, 0, -1, 0)
        print("Moving to the left, now!")
        time.sleep(1)
        i += 1

# head east (+y), relative to drone
def moveRight(vehicle):
    i = 0   # each iteration moves the drone
    while (i < 12):
        send_local_ned_velocity(vehicle, 0, 1, 0)
        print("Moving to the right, now!")
        time.sleep(1)
        i += 1
    print("MISSION COMPLETE!!")



# head north (+x), relative to Earth
def moveForward_G(vehicle):
    i = 0   # each iteration moves the drone
    while (i < 12):
        send_global_ned_velocity(vehicle, 1, 0, 0)
        print("Moving forward now!")
        time.sleep(1)
        i += 1

# head south (-x), relative to Earth
def moveBackwards_G(vehicle):
    i = 0   # each iteration moves the drone
    while (i < 6):
        send_global_ned_velocity(vehicle, -1, 0, 0)
        print("Moving backwards now!")
        time.sleep(1)
        i += 1

# head west (-y), relative to Earth
def moveLeft_G(vehicle):
    i = 0   # each iteration moves the drone
    while (i < 6):
        send_global_ned_velocity(vehicle, 0, -1, 0)
        print("Moving to the left, now!")
        time.sleep(1)
        i += 1

# head east (+y), relative to Earth
def moveRight_G(vehicle):
    i = 0   # each iteration moves the drone
    while (i < 12):
        send_global_ned_velocity(vehicle, 0, 1, 0)
        print("Moving to the right, now!")
        time.sleep(1)
        i += 1
    print("GLOBAL MISSION COMPLETE!!")

############# MAIN #############


if __name__=='__main__':


    # YOU SHOULD PUT A TRY-CATCH BLOCK HERE! (PER DK DOC)

    # set to false when field testing the real thing
    sitlInstance = True

    # connect to the drone
    vehicle = connectToDrone(sitlInstance)

    # print drone data
    #fetchAndPrintAttributes(vehicle)

    # suite of flight modes to test
    flightMode0 = "STABILIZE"
    flightMode1 = "ALT_HOLD"
    flightMode2 = "LOITER"
    flightMode3 = "LAND"
    flightMode4 = "GUIDED"
    flightMode5 = "DISARM"
    flightMode6 = "RTL"

    # NOTE: for the simple takeoff dronekit f(x), you must 1st
    # be armable, then enter guided mode, and then call the f(x)

    # stab mode
    setMode(vehicle, flightMode0)

    # in meters
    desiredHeight = 4

    # elevate and hover
    takeoff(vehicle, desiredHeight)

    # Cross maneuver in the local frame
    '''moveForward(vehicle)
    moveBackwards(vehicle)
    moveLeft(vehicle)
    moveRight(vehicle)'''

    # Now we try the same manuevers in the global frame
    print("Now entering the global frame!!!!!")
    print("Now entering the global frame!!!!!")
    print("Now entering the global frame!!!!!")
    moveForward_G(vehicle)
    moveBackwards_G(vehicle)
    moveLeft_G(vehicle)
    moveRight_G(vehicle)

    # land the drone (and dont change altitude)
    vehicle.parameters['RTL_ALT'] = 0
    setMode(vehicle, flightMode6)

    vehicle.close()

  


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
        print "Current Altitude: ", currHeight

        # Once within an acceptable height, terminate
        if currHeight >= acceptableHeight:
            break
        time.sleep(1)

    print "Target Altitude Reached!!"
    time.sleep(5)

    return None
# takeoffAndLand()



# Capable of commanding the drone to rotate about the z-axis.
# In order to use this method, you must first call it's initializer.
def condition_yaw(vehicle, degrees, relative):

    # Establish the frame
    if relative:
        isRelative = 1      # Relative to direction of travel
    else: 
        isRelative = 0      # Uses absolute angles

    msg = vehicle.message_factory.command_long_encode(
        0, 0,                                   #
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,  #
        0,                                      #
        degrees,                                #
        0,                                      #
        1,                                      #
        isRelative,                             #
        0, 0, 0                                 #
    )

    vehicle.send_mavlink(msg)
    vehicle.flush()
# condition_yaw()



# Initializes the yaw condition above
def yaw_init(vehicle):

    lat = vehicle.location.global_relative_frame.lat
    lon = vehicle.location.global_relative_frame.lon
    alt = vehicle.location.global_relative_frame.alt

    location = LocationGlobalRelative(lat, lon, alt)

    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # frame
        0b0000111111111000,                                 # type_mask w/ speed enabled
        location.lat*1e7,                                   # x position
        location.lon*1e7,                                   # y position
        location.alt,                                       # z position
        0,                                                  # x velocity
        0,                                                  # y velocity
        0,                                                  # z velocity
        0, 0, 0,                                            # acceleration
        0, 0                                                # yaw_rate
    )

    vehicle.send_mavlink(msg)
    vehicle.flush()
# yaw_init()



#*********************************************************************************
# NOTE: THE NEXT TWO F(X)S HAVE THE MOST UP-TO-DATE VELOCITY F(X)S THAT WORK! THEY 
#       HAVE BEEN TESTED, SO USE THEM IN ALL SUBSEQUENT SCRIPTS.
#*********************************************************************************


# Sends a forward/backward velocity message in the drone's frame,
# which allows movement commands to be simulated. It enables yaw, but
# sets its rate (yaw_rate) to zero so that it doesnt yaw.
# Usage: +vx for forward and -vx for backward
def forward_backwards_velocity(vehicle, vx, vy, vz):
        
    # MAVLink message in the local/drone's frame, where forward is the drone's heading
	msg = vehicle.message_factory.set_position_target_local_ned_encode(
		0,                                          # time_boot_ms
		0, 0,                                       # target_system, target_component
		mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # coord_frame (local here)
		0b010111000111,                             # type_mask (decimal: 1479)
		0, 0, 0,                                    # position
		vx, vy, vz,                                 # velocity
		0, 0, 0,                                    # acceleration
		0, 0
    )
        
	vehicle.send_mavlink(msg)
	vehicle.flush()
# send_local_ned_velocity()



# Sends a right/left velocity message in the drone's frame,
# which allows movement commands to be simulated. It enables yaw, but
# sets its rate (yaw_rate) to zero so that it doesnt yaw.
# Usage: +vy for right and -vx for left
def side_to_side_velocity(vehicle, vx, vy, vz):
        
    # MAVLink message in the local/drone's frame, where forward is the drone's heading
	msg = vehicle.message_factory.set_position_target_local_ned_encode(
		0,                                          # time_boot_ms
		0, 0,                                       # target_system, target_component
		mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # coord_frame (local here)
		0b010111000111,                             # type_mask (decimal: 1479)
		0, 0, 0,                                    # position
		vx, vy, vz,                                 # velocity
		0, 0, 0,                                    # acceleration
		0, 0                                        # yaw, yaw_rate
    )
        
	vehicle.send_mavlink(msg)
	vehicle.flush()
# send_local_ned_velocity()



#*********************************************************************************
# NOTE: THE ABOVE TWO F(X)S HAVE THE MOST UP-TO-DATE VELOCITY F(X)S THAT WORK! THEY 
#       HAVE BEEN TESTED, SO USE THEM IN ALL SUBSEQUENT SCRIPTS.
#*********************************************************************************




# Sends a velocity message in a global frame, 
# which allows movement commands.
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
def moveForward(vehicle, velocity, duration):
    i = 0   # each iteration moves the drone
    while (i < duration):
        forward_backwards_velocity(vehicle, velocity, 0, 0)
        print "Moving forward now!"
        time.sleep(2)
        i += 1

# head south (-x), relative to drone
def moveBackwards(vehicle, velocity, duration):
    i = 0   # each iteration moves the drone
    while (i < duration):
        forward_backwards_velocity(vehicle, -velocity, 0, 0)
        print "Moving backwards now!"
        time.sleep(2)
        i += 1

# head west (-y), relative to drone
def moveLeft(vehicle, velocity, duration):
    i = 0   # each iteration moves the drone
    while (i < duration):
        side_to_side_velocity(vehicle, 0, -velocity, 0)
        print "Moving to the left, now!"
        time.sleep(2)
        i += 1

# head east (+y), relative to drone
def moveRight(vehicle, velocity, duration):
    i = 0   # each iteration moves the drone
    while (i < duration):
        side_to_side_velocity(vehicle, 0, velocity, 0)
        print "Moving to the right, now!"
        time.sleep(2)
        i += 1
    print "MISSION COMPLETE!!"









# head north (+x), relative to Earth
def moveForward_G(vehicle, velocity, duration):
    i = 0   # each iteration moves the drone
    while (i < duration):
        send_global_ned_velocity(vehicle, velocity, 0, 0)
        print "Moving forward now!"
        time.sleep(2)
        i += 1

# head south (-x), relative to Earth
def moveBackwards_G(vehicle, velocity, duration):
    i = 0   # each iteration moves the drone
    while (i < duration):
        send_global_ned_velocity(vehicle, -velocity, 0, 0)
        print "Moving backwards now!"
        time.sleep(2)
        i += 1

# head west (-y), relative to Earth
def moveLeft_G(vehicle, velocity, duration):
    i = 0   # each iteration moves the drone
    while (i < duration):
        send_global_ned_velocity(vehicle, 0, -velocity, 0)
        print "Moving to the left, now!"
        time.sleep(2)
        i += 1

# head east (+y), relative to Earth
def moveRight_G(vehicle, velocity, duration):
    i = 0   # each iteration moves the drone
    while (i < duration):
        send_global_ned_velocity(vehicle, 0, velocity, 0)
        print "Moving to the right, now!"
        time.sleep(2)
        i += 1
    print "GLOBAL MISSION COMPLETE!!"









def cross_maneuver(vehicle, velocity, duration):
    
    print "Performing Cross Maneuver now..."

    print "Moving forward for", duration, "seconds"
    moveForward(vehicle, velocity, duration)
    time.sleep(5)

    backwardsDuration = duration/2
    print "Now, I will move backwards for", backwardsDuration, "seconds"
    moveBackwards(vehicle, velocity, backwardsDuration)
    time.sleep(5)

    leftDuration = backwardsDuration
    print "Now, I will roll left for", leftDuration, "seconds"
    moveLeft(vehicle, velocity, leftDuration)
    time.sleep(5)

    print "To complete the cross, I will roll right for", duration, "seconds"
    moveRight(vehicle, velocity, duration)
    time.sleep(5)

    print "CROSS MANEUVER MISSION COMPLETE!!!! Getting ready to head home now..."



def forward_and_right(vehicle, velocity, duration):

    print "Performing the Forward-and-Right Maneuver now..."

    moveForward(vehicle, velocity, duration)



def move_forward_then_backwards(vehicle, velocity, duration):

    print "Performing a forwards, then backwards sequence now..."

    print "Moving forward for", duration, "seconds"
    moveForward(vehicle, velocity, duration)
    time.sleep(5)

    backwardsDuration = duration/2
    print "Now, I will move backwards for", backwardsDuration, "seconds"
    moveBackwards(vehicle, velocity, backwardsDuration)
    time.sleep(5)







############# MAIN #############


if __name__=='__main__':


    # YOU SHOULD PUT A TRY-CATCH BLOCK HERE! (PER DK DOC)

    # set to false when field testing the real thing
    sitlInstance = False

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
    desiredHeight = 2

    desiredVelocity = 2

    # Controls how long the movement lasts
    # NOTE: Set even numbers for simplicity
    desiredTravelDistance = 4


    # elevate and hover
    takeoff(vehicle, desiredHeight)

    #move_forward_then_backwards(vehicle, desiredVelocity, desiredTravelDistance)

    # Cross maneuver in the local frame
    cross_maneuver(vehicle, desiredVelocity, desiredTravelDistance)

    # Now we try the same manuevers in the global frame
    '''
    print("Now entering the global frame!!!!!")
    print("Now entering the global frame!!!!!")
    print("Now entering the global frame!!!!!")
    moveForward_G(vehicle)
    moveBackwards_G(vehicle)
    moveLeft_G(vehicle)
    moveRight_G(vehicle)
    '''


    '''
    # Trying yaw functionality
    print("Now entering the YAW function!!!!!")
    print("Now entering the YAW function!!!!!")
    print("Now entering the YAW function!!!!!")
    
    # Initialize the yaw method, then execute three maneuvers
    yaw_init(vehicle)

    condition_yaw(vehicle, 30, True)
    print('Yawing 30 degrees relative to current position!!!')
    time.sleep(7)

    condition_yaw(vehicle, 0, False)
    print('Yawing true North!!!')
    time.sleep(7)

    condition_yaw(vehicle, 270, False)
    print('Yawing true West!!!')
    time.sleep(7)
    '''
    
    print "Sleeping for 20 seconds before I come home..."
    time.sleep(20)

    # land the drone (and dont change altitude)
    print "Enabling RTL now..."
    vehicle.parameters['RTL_ALT'] = 0
    setMode(vehicle, flightMode6)

    vehicle.close()

  


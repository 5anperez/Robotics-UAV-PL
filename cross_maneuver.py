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



# NOTE: velocity commands must be resent every one second to keep
# the drone in constant motion. sending one velocity message will
# only move the drone for one second.


# NOTE: if 'RNG' in the SITL console is greyed out, then it hasnt
# been enabled! to enable, do: 
#           param set RNGFND1_TYPE 100
#           reboot
# in the terminal running MAVProxy. it should turn green


# Head north (+x), relative to drone
def moveForward(vehicle, velocity, duration):
    i = 0   # Each iteration moves the drone
    while (i < duration):
        send_local_ned_velocity(vehicle, velocity, 0, 0)
        print "Moving forward now!"
        time.sleep(2)
        i += 1

# Head south (-x), relative to drone
def moveBackwards(vehicle, velocity, duration):
    i = 0   # Each iteration moves the drone
    while (i < duration):
        send_local_ned_velocity(vehicle, -velocity, 0, 0)
        print "Moving backwards now!"
        time.sleep(2)
        i += 1

# Head west (-y), relative to drone
def moveLeft(vehicle, velocity, duration):
    i = 0   # Each iteration moves the drone
    while (i < duration):
        send_local_ned_velocity(vehicle, 0, -velocity, 0)
        print "Moving to the left, now!"
        time.sleep(2)
        i += 1

# Head east (+y), relative to drone
def moveRight(vehicle, velocity, duration):
    i = 0   # Each iteration moves the drone
    while (i < duration):
        send_local_ned_velocity(vehicle, 0, velocity, 0)
        print "Moving to the right, now!"
        time.sleep(2)
        i += 1




# This is a sequence of commands that allows the drone to 
# perform a 'cross-maneuver', which is simply to travel 
# North, then South, then East, and finally West. A top-down
# view reveals a path that takes the shape of a cross. 
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

    desiredHeight = 2       # in meters
    desiredVelocity = 2     # cm/s

    # Controls how long the movement lasts
    # NOTE: Set even numbers for simplicity
    desiredTravelDistance = 4


    # Elevate and hover
    takeoff(vehicle, desiredHeight)

    # Simple North, then South maneuver 
    #move_forward_then_backwards(vehicle, desiredVelocity, desiredTravelDistance)

    # Cross maneuver (N -> S -> E -> W -> RTL)
    cross_maneuver(vehicle, desiredVelocity, desiredTravelDistance)

    # When done w/ mission, hover in place
    print "Sleeping for 20 seconds before I come home..."
    time.sleep(20)

    # RTL: return to launch (and dont change altitude)
    print "Enabling RTL now..."
    vehicle.parameters['RTL_ALT'] = 0   # Keep the current altitude
    setMode(vehicle, flightMode6)

    # Clean up
    vehicle.close()

  


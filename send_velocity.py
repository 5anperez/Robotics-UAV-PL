from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time
import socket
import math
import os
import platform
import sys
from pymavlink import mavutil


############# FUNCTIONS #############



# returns a vehicle object that represents the drone and establishes the connection
def connectDrone():
    vehicle = connect('/dev/ttyAMA0', baud = 57600, wait_ready = True)
    return vehicle



# arm the drone and start props
def armDrone(vehicle):

    if not vehicle.is_armable:
        # begin the arming phase
        while not vehicle.is_armable:
            print("Waiting for drone to become armable...")
            time.sleep(1)
        print("Vehicle is now armable!")

    # once armable, enter desired flight mode
    print("Initiating arm stage...")
    
    vehicle.armed = True
    while not vehicle.armed:
	
        print("Waiting for drone to become armed...")
        time.sleep(1)
	
    print("Look out! Props are spinning!!")
    time.sleep(1)
    
    return None
# armDrone()



# switch flight modes
def setMode(vehicle, flightMode):

    print("Entering", flightMode, "mode now...")
    
    vehicle.mode = VehicleMode(flightMode)
    while vehicle.mode != flightMode:
        print("Waiting to enter", flightMode, "flight mode...")
        time.sleep(1)
    print("Vehicle is now in", flightMode, "mode!!")   

    return vehicle
# setMode()



def takeoff(vehicle, targetHeight):
    # make sure you are in guided mode!
    # vehicle.mode = VehicleMode('GUIDED')
    if not vehicle.is_armable:
        # begin the arming phase
        while not vehicle.is_armable:
            print("Waiting for drone to become armable...")
            time.sleep(1)
        print("Vehicle is now armable!")

    setMode(vehicle, "GUIDED")

    armDrone(vehicle)
    
    # take off to desired height
    vehicle.simple_takeoff(targetHeight)

    while True:
        
        acceptableHeight = targetHeight * 0.95
        currHeight = vehicle.location.global_relative_frame.alt

        # +/- .5 meters is an acceptable range
        print("Current Altitude: ", currHeight)

        if currHeight >= acceptableHeight:
            break
        time.sleep(1)

    print("Target Altitude Reached!!")
    time.sleep(5)

    return None
# takeoffAndLand()



# send movement commands
def send_local_ned_velocity(vehicle, vx, vy, vz):
	msg = vehicle.message_factory.set_position_target_local_ned_encode(
		0,
		0, 0,
		mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
		0b0000111111000111,
		0, 0, 0,
		vx, vy, vz,
		0, 0, 0,
		0, 0)
	vehicle.send_mavlink(msg)
	vehicle.flush()
# send_local_ned_velocity()



# head north (+x), relative to drone
def moveForward(vehicle):
    i = 0   # each iteration moves the drone
    while (i < 8):
        send_local_ned_velocity(vehicle, 1, 0, 0)
        print("Moving forward now!")
        time.sleep(1)
        i += 1

# head south (-x), relative to drone
def moveBackwards(vehicle):
    i = 0   # each iteration moves the drone
    while (i < 4):
        send_local_ned_velocity(vehicle, -1, 0, 0)
        print("Moving forward now!")
        time.sleep(1)
        i += 1

# head west (-y), relative to drone
def moveLeft(vehicle):
    i = 0   # each iteration moves the drone
    while (i < 4):
        send_local_ned_velocity(vehicle, 0, -1, 0)
        print("Moving to the left, now!")
        time.sleep(1)
        i += 1

# head east (+y), relative to drone
def movRight(vehicle):
    i = 0   # each iteration moves the drone
    while (i < 4):
        send_local_ned_velocity(vehicle, 0, 1, 0)
        print("Moving to the right, now!")
        time.sleep(1)
        i += 1



############# MAIN #############


if __name__=='__main__':

    # connect to the drone
    vehicle = connectDrone()

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

    moveForward(vehicle)
    moveLeft(vehicle)
    #moveBackwards(vehicle)

    # land the drone
    setMode(vehicle, flightMode6)

    vehicle.close()

  


from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time
import socket
import math
import os
import platform
import sys
from pymavlink import mavutil


############# FUNCTIONS #############



# returns a vehicle object that represents the REAL drone and establishes the connection
def connectDrone():
    vehicle = connect('/dev/ttyAMA0', baud = 57600, wait_ready = True)
    return vehicle



# arm the drone and start props
def armDrone(vehicle):
    
    vehicle.armed = True
    while not vehicle.armed:
	
        print("Waiting for drone to become armed...")
        time.sleep(1)
	
    print("Look out! Props are spinning!!")
    time.sleep(1)
    
    return None



# switch flight modes
def switchMode(vehicle, flightMode):

    print("Switching into", flightMode, "mode now...")

    # begin the arming phase
    while not vehicle.is_armable:
        print("Waiting for drone to become armable...")
        time.sleep(1)
    print("Vehicle is now armable!")

    # once armable, enter desired flight mode
    
    vehicle.mode = VehicleMode(flightMode)
    while vehicle.mode != flightMode:
        print("Waiting to enter", flightMode, "flight mode...")
        time.sleep(1)
    print("Vehicle is now in", flightMode, "mode!!")    

    return vehicle



def takeoffAndLand(targetHeight):
    
    vehicle.mode = VehicleMode('GUIDED')

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

    return None




# prints useful attribute data representative of our vehicle's state
# uses dronekit as an API to ardupilot to access most attributes
# usage: $ python attribute_fetch.py
def fetchAndPrintAttributes(vehicle):

    # version and attributes
    vehicle.wait_ready('autopilot_version')
    print('Autopilot version: %s'%vehicle.version)

    # bool check if the firmware supports our on-board 
    # computer for attitude setter capabilities
    print('Attitude set is supported: %s'%vehicle.capabilities.set_attitude_target_local_ned)

    # read the latitude, longitude, altitude, coords in the global frame
    print('Position: %s'%vehicle.location.global_relative_frame)

    # read attitude, roll, pitch, and yaw
    print('Attitude: %s'%vehicle.attitude)

    # read velocity (m/s) in NED coords
    print('Velocity: %s'%vehicle.velocity)

    # when was the last h.beat?
    print('Last heartbeat was: %s'%vehicle.last_heartbeat)

    # bool check the armable state
    print('Armable: %s'%vehicle.is_armable)

    # setter
    print('Groundspeed: %s'%vehicle.groundspeed)

    # print the flight mode
    print('Mode: %s'%vehicle.mode.name)

    # arm setter (returns 1 if props are spinning)
    print('Armed: %s'%vehicle.armed)

    # print the state estimation filter status
    print('Position: %s'%vehicle.ekf_ok)




# send movement commands
def send_local_ned_velocity(vx, vy, vz):
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

    switchMode(vehicle, flightMode0)

    vehicle.close()


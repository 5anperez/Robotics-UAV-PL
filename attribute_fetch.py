from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time
import socket
import exceptions
import math
import argparse ## parse cmd args


############# FUNCTIONS #############



# Capable of launching and connecting to a virtual drone, or the real thing.
# If sitlInstance = True, then connect to sitl, o/w connect to RPi + Pixhawk.
def connectToDrone(sitlInstance):

    if sitlInstance:
        vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)
    else:
        vehicle = connect('/dev/ttyAMA0', baud = 57600, wait_ready = True)

    return vehicle
# connectDrone()



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







############# MAIN #############

# flag to initialize sitl instance 
# (change to true for simulations)
#initSitl = True
initSitl = False

# connect to the drone
vehicle = connectToDrone(initSitl)

# print drone data
fetchAndPrintAttributes(vehicle)

vehicle.close()


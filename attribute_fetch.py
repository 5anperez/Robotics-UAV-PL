from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time
import socket
import exceptions
import math
import argparse ## parse cmd args


############# FUNCTIONS #############



# creates a vehicle object, which is connected via ip, to represent the drone. 
# the object allows mavlink commands and dronekit methods to be sent to the drone.  
# usage: >> python connection_template.py --connect <drone's ip>
def connectToDrone():

    # parser object
    parser = argparse.ArgumentParser(description='commands')
    
    # capture the drone's ip address
    parser.add_argument('--connect')
    args = parser.parse_args

    # store ip here
    connection_string = args.connect

    # if no ip was entered, then run a sitl instance
    if not connection_string: 
        import dronekit_sitl
        sitl = dronekit_sitl.start_default()
        connection_string = sitl.connection_string()

    # dronekit method to establish the connection
    vehicle = connect(connection_string, wait_ready = True)

    return vehicle



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

# connect to the drone
vehicle = connectToDrone()

# print drone data
fetchAndPrintAttributes(vehicle)

vehicle.close()


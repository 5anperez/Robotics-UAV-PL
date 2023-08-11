from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time
import socket
import exceptions
import math
import argparse ## parse cmd args


############# FUNCTIONS #############



# engage guided flight mode and then arm
def modeSetter(vehicle):
    
    while not vehicle.armable:
        print('Waiting to become armable...')
        time.sleep(1)

    print('Drone is now armable!')

    # enter guided mode
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode is not 'GUIDED':
        print("Waiting for drone to enter GUIDED mode...")
        time.sleep(1)

    print("Drone is now in GUIDED flight mode!")

    vehicle.armed = True
    while not vehicle.armed:
        print("Waiting for drone to arm...")
        time.sleep(1)

    print("The drone is now armed and the props are spinning!!")



# creates a vehicle object, which is connected via ip, to represent the drone. 
# the object allows mavlink commands and dronekit methods to be sent to the drone.  
# USAGE: >> python connection_template.py --connect <drone's ip>
def connectToDrone():

    # parser object
    parser = argparse.ArgumentParser(description='commands')
    
    # capture the drone's ip address
    parser.add_argument('--connect')
    args = parser.parse_args

    # store ip here
    connection_string = args.connect

    # if --connect nor ip was entered, then run a sitl instance 
    if not connection_string: 
        import dronekit_sitl
        sitl = dronekit_sitl.start_default()
        connection_string = sitl.connection_string()

    # dronekit method to establish the connection
    vehicle = connect(connection_string, wait_ready = True)

    return vehicle





############# MAIN #############


vehicle = connectToDrone()

# test the mode setter
modeSetter(vehicle)




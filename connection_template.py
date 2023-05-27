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

    # dronekit method to establish the connection
    vehicle = connect(connection_string, wait_ready = True)

    return vehicle





############# MAIN #############

vehicle = connectToDrone()


#!/usr/bin/python


  #########################################
################## IMPORTS ##################
  #########################################


import rospy
from sensor_msgs.msg import Image
import cv2
import cv2.aruco as aruco
import sys
import time
import math
import numpy as np
import ros_numpy as rnp
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, APIException
from pymavlink import mavutil
from array import array




  ########################################
################## DK VARS ##################
  ########################################


# init connection to sitl 
vehicle = connect('udp:127.0.0.1:14551', wait_ready=True)

# precision land params
vehicle.parameters['PLND_ENABLED'] = 1    # enable PL
vehicle.parameters['PLND_TYPE'] = 1       # companion computer mode
vehicle.parameters['PLND_EST_TYPE'] = 0   # disable EKF (use camera only)
vehicle.parameters['LAND_SPEED'] = 30     # landing speed (cm/sec)

# range finder params
vehicle.parameters['RNGFND1_TYPE'] = 1
vehicle.parameters['RNGFND1_MIN_CM'] = 0
vehicle.parameters['RNGFND1_MAX_CM'] = 4000
vehicle.parameters['RNGFND1_PIN'] = 0
vehicle.parameters['RNGFND1_SCALING'] = 12.12

# movement params
velocity = 0.5      # m/s
takeoff_height = 4  # 4m = 13.12ft




  #########################################
################## ROS VARS ##################
  #########################################


# our publishing obj to read in the raw img feed over Image messages
# our new topic is image_new which displays the detected marker in an
# informative manner i.e. with data we need to precision land
new_img_pub = rospy.Publisher('/camera/color/image_new', Image, queue_size=10)

# desired, pregenerated aruco id
id_to_find = 72

# aruco marker in cm
marker_size = 20

# define the dictionary obj and predefined aruco dict we sample from
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()

# image resolution to match RPi camv2 in pixels
horiz_res = 640
vert_res = 480

# image field of view to match RPi camv2, in rads
rads = (math.pi / 180)
horiz_fov = 62.2 * rads
vert_fov = 48.8 * rads

# counters to compute avg.
found_count = 0
not_found_count = 0



  ###################################################
################## CAMERA INTRINSICS ##################
  ###################################################



# simulated distortion coefficient and cam matrix from the image topic
# D: [0.0, 0.0, 0.0, 0.0, 0.0]
# K: [530.8269276712998, 0.0, 320.5, 0.0, 530.8269276712998, 240.5, 0.0, 0.0, 1.0]
dist_coeff = [0.0, 0.0, 0.0, 0.0, 0.0] # D
camera_matrix = [[530.8269276712998, 0.0, 320.5], [0.0, 530.8269276712998, 240.5], [0.0, 0.0, 1.0]] # K

# convert intrinsics to numpy arrays
np_dist_coeff = np.array(dist_coeff)
np_cam_mat = np.array(camera_matrix)

# global vars
time_last = 0
time_to_wait = 0.1 # ms




  #############################################
################## DK FUNCTIONS ##################
  ###########################################


# execute the drones required setup sequence, then takeoff
def arm_and_takeoff(target_height):
    
    # wait to be armable
    while not vehicle.is_armable:
         print("Waitning for drone to become armable...")
         time.sleep(1)

    print("Drone is now armable!")

    # enable guided mode
    vehicle.mode = VehicleMode('GUIDED')

    # wait to enter guided mode
    while vehicle.mode != 'GUIDED':
        print("Waiting for drone to enter GUIDED flight mode...")
        time.sleep(1)

    print("Drone has now entered GUIDED mode!")

    # arm the drone
    vehicle.armed = True
    while not vehicle.armed:
        print("Waiting to arm...")
        time.sleep(1)

    print("Look out virtual props now spinning!!!")

    # dronekit method to takeoff
    vehicle.simple_takeoff(target_height)

    # wait to reach desired height
    while True:
        
        # grab our altitude in the global frame
        current_altitude = vehicle.location.global_relative_frame.alt
        print('Current Altitude: %d'%current_altitude)

        # +/- 0.5 is fine
        acceptable_height = target_height * 0.95
        if current_altitude >= acceptable_height:
            break
        time.sleep(1)
    print("Target altitude reached!!!")

    return None



# send velocity command
def send_local_ned_velocity(vx, vy, vz):
    
    # create and send a mavlink message
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



# perform precision landing w/ xy coords
def send_land_message(x, y):
    # create and send a mavlink message
    msg = vehicle.message_factory.landing_target_encode(
        0,
        0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        x,
        y,
        0,0,0)
    vehicle.send_mavlink(msg)
    vehicle.flush()





  ##############################################
################## ROS FUNCTIONS ##################
  ##############################################



# define our ROS callback: detects an aruco marker and processes it into
# a human friendly version with a distinct green border and RGB color 
# coded xyz=GBR axis where z is vertical. it also displays supplemental data  
# that tells us the distance (xyz error terms) from camera to marker in cm.
def msg_receiver(message):

    # global counters 
    global not_found_count, found_count, time_last, time_to_wait, id_to_find

    # wait 100 millisecs, which is a realistic frame per seconds interval
    if (time.time() - time_last) > time_to_wait:
        
        # convert ros message to a required array to begin processing with opencv
        np_data = rnp.numpify(message)

        # opencv method to convert image to a gray scale
        gray_img = cv2.cvtColor(np_data, cv2.COLOR_BGR2GRAY)

        # opencv method to grab the current frame's corners and detected id  
        ids = ''
        (corners, ids, rejected) = aruco.detectMarkers(image = gray_img, 
                                                       dictionary = aruco_dict, 
                                                       parameters = parameters)


        # process the image (ids) and check for the desired marker id
        try:
            if ids is not None:
                if ids[0] == id_to_find:
                    # opencv method to grab and estimate the marker's pose
                    ret = aruco.estimatePoseSingleMarkers(corners, marker_size, 
                                                          cameraMatrix = np_cam_mat, 
                                                          distCoeffs = np_dist_coeff)
                    
                    # grab the rotational and translational vectors in cm
                    (rvec, tvec) = (ret[0][0,0,:], ret[1][0,0,:])

                    # grab the xyz translational components (dist between cam & aruco)
                    # i.e. this finds the markers estimated position in *cm*
                    x = '{:.2f}'.format(tvec[0])
                    y = '{:.2f}'.format(tvec[1])
                    z = '{:.2f}'.format(tvec[2])

                    # to align the the center of image with the center of the marker, take the avg of the corner coords
                    # to get the center of the aruco marker, then use the offsets to align with the center of the image 
                    x_sum = 0
                    y_sum = 0
                    x_sum = corners[0][0][0][0] + corners[0][0][1][0] + corners[0][0][2][0] + corners[0][0][3][0]
                    y_sum = corners[0][0][0][1] + corners[0][0][1][1] + corners[0][0][2][1] + corners[0][0][3][1]
                    x_avg = x_sum / 4
                    y_avg = y_sum / 4

                    # convert offsets to angles for the mavlink message:
                    # center of marker - center of image * 
                    x_theta = (x_avg - (horiz_res / 2)) * (horiz_fov / horiz_res)
                    y_theta = (y_avg - (vert_res / 2)) * (vert_fov / vert_res)
          

                    # once the marker is detected, tell the drone to land, if it isnt already
                    if vehicle.mode != 'LAND':
                        vehicle.mode = VehicleMode('LAND')

                        while vehicle.mode != 'LAND':
                            time.sleep(1)
                        print("Drone is now in LAND mode...")
                        
                        # send the drone the distance error
                        send_land_message(x_theta, y_theta)
                    else:
                        send_land_message(x_theta, y_theta)


                    # convert to a printable strings
                    marker_position1 = 'MARKER POSITION:' 
                    marker_position2 = 'x = ' + x + ', y = ' + y + ', z = ' + z

                    # opencv method to draw and display a border around detected markers (modify the raw image)
                    aruco.drawDetectedMarkers(np_data, corners)

                    # opencv method to draw axis
                    aruco.drawAxis(np_data, np_cam_mat, np_dist_coeff, rvec, tvec, 10)

                    # display marker data
                    text_position1 = (5, 50) # xy coords of bottom left corner of string in image
                    text_position2 = (5, 80) # xy coords of bottom left corner of string in image
                    text_font = cv2.FONT_HERSHEY_SIMPLEX
                    #text_font = 0
                    text_font_scale = 1
                    text_color = (255, 255, 255)
                    cv2.putText(np_data, marker_position1, text_position1, text_font, 
                                text_font_scale, text_color, thickness = 2)
                    cv2.putText(np_data, marker_position2, text_position2, text_font, 
                                text_font_scale, text_color, thickness = 2)
                    
                    # print some stats
                    stats = 'Found: ' + str(found_count) + ' and Not Found: ' + str(not_found_count)
                    print(marker_position1)
                    print(marker_position2)
                    print(stats)

                    # keep a running total
                    found_count += 1
                else:
                    not_found_count += 1

            else:
                not_found_count += 1

        except Exception as e:
            print('Target likely not found')
            print(e)
            not_found_count += 1

        # convert back into ros format to be publishable to topic
        new_msg = rnp.msgify(Image, np_data, encoding = 'rgb8')
        new_img_pub.publish(new_msg)

        # keep track of realistic 10 fps
        time_last = time.time()

    else:
        return None




# create a subscriber to the image raw topic
def subscriber():

    # init the node
    rospy.init_node('drone_node', anonymous=False)

    # subsribe to the raw image topic insert our callback
    rospy.Subscriber('/camera/color/image_raw', Image, msg_receiver)
    
    # loop
    rospy.spin()




  ######################################
################## MAIN ##################
  ######################################


# call our functions
if __name__=='__main__':
    try:
        # execute takeoff
        arm_and_takeoff(takeoff_height)
        time.sleep(1)

        # send a movement velocity command in the x direction, which will
        # fly in the x for 1 sec, then stop to begin surface tracking
        send_local_ned_velocity(velocity, 0, 0)
        time.sleep(1) 

        # execute aruco tracking & adjust the drone to get aligned 
        # with the aruco marker i.e. perform precision land  
        subscriber()
    except rospy.ROSInterruptException:
        pass



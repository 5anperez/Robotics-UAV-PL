from dronekit import connect, VehicleMode
import time

def arm_and_takeoff(vehicle, target_altitude):
    print("Arming and Taking Off...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(1)
    
    vehicle.simple_takeoff(target_altitude)
    while True:
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def send_local_ned_velocity(vehicle, vx, vy, vz, duration):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()
    time.sleep(duration)

def cross_maneuver(vehicle):
    print("Performing Cross Maneuver...")
    send_local_ned_velocity(vehicle, 1, 0, 0, 5)  # Forward for 5 seconds
    send_local_ned_velocity(vehicle, -1, 0, 0, 2) # Backward for 2 seconds
    send_local_ned_velocity(vehicle, 0, -1, 0, 3) # Left for 3 seconds
    send_local_ned_velocity(vehicle, 0, 1, 0, 6)  # Right for 6 seconds
    send_local_ned_velocity(vehicle, 0, -1, 0, 3) # Left for 3 seconds
    print("Cross Maneuver Complete!")

def main():
    connection_string = "YOUR_CONNECTION_STRING_HERE" # Replace with your connection string
    vehicle = connect(connection_string, wait_ready=True)
    
    arm_and_takeoff(vehicle, 10) # 10 meters altitude
    cross_maneuver(vehicle)

    print("Returning to Launch")
    vehicle.mode = VehicleMode("RTL")
    time.sleep(10)

    vehicle.close()

if __name__ == "__main__":
    main()

from pymavlink import mavutil
import time


# Creating a master var to initiate connection between drone and the computer
master = mavutil.mavlink_connection('udp:0.0.0.0:14550')
master.wait_heartbeat()
print(master.target_system, master.target_component)

# Switching to GUIDED mode
mode = 'GUIDED'
mode_id = master.mode_mapping()[mode]
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id
)


#Arming the drone
master.arducopter_arm()
print("Waiting for the vehicle to arm")
master.motors_armed_wait()
print('Armed!')





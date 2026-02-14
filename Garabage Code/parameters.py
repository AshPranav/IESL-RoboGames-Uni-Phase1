from pymavlink import mavutil
import time

# Connect to the flight controller (adjust port as needed, e.g., '/dev/ttyACM0' or 'udp:127.0.0.1:14550')
connection = mavutil.mavlink_connection('udp:0.0.0.0:14550')
connection.wait_heartbeat()
print("Connected to Flight Controller!")

# Define the target values based on your requirements
params_to_fix = {
    'ACRO_BAL_ROLL': 0.0,
    'ACRO_BAL_PITCH': 0.0,
    'ATC_RAT_RLL_I': 0.19,  # Set slightly above 0.18 to pass 'gt' condition
    'ATC_RAT_PIT_I': 0.19,  # Set slightly above 0.18 to pass 'gt' condition
    'ATC_RAT_YAW_I': 0.02   # Set slightly above 0.018 to pass 'gt' condition
}

def set_param(name, value):
    connection.mav.param_set_send(
        connection.target_system,
        connection.target_component,
        name.encode('utf-8'),
        value,
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )
    print(f"Setting {name} to {value}...")

# Apply the fixes
for param, val in params_to_fix.items():
    set_param(param, val)
    time.sleep(0.1) # Short delay to prevent flooding the telemetry link

print("Parameters updated successfully.")
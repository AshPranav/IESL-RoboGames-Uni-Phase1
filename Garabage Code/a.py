from pymavlink import mavutil
import time

# --------------------------------------------------
# CONNECT
# --------------------------------------------------
master = mavutil.mavlink_connection('udp:0.0.0.0:14550')
master.wait_heartbeat()
print("Connected")

# --------------------------------------------------
# 🔴 WAIT FOR FULL PARAM DOWNLOAD
# --------------------------------------------------
print("Waiting for parameters...")

master.mav.param_request_list_send(
    master.target_system,
    master.target_component
)

while True:
    msg = master.recv_match(type='PARAM_VALUE', blocking=True)
    if msg.param_index == msg.param_count - 1:
        break

print("All parameters received")

# --------------------------------------------------
# PARAM SETTER WITH CONFIRM
# --------------------------------------------------
def set_param(name, value):
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        name.encode(),
        float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )

    # wait confirmation
    while True:
        msg = master.recv_match(type='PARAM_VALUE', blocking=True)
        if msg.param_id.decode().strip('\x00') == name:
            print(f"Set {name} = {value}")
            break

# --------------------------------------------------
# FIX ACRO BALANCE (THIS IS THE BLOCKER)
# --------------------------------------------------
set_param("ACRO_BAL_ROLL", 1)
set_param("ACRO_BAL_PITCH", 1)

# optional safety disables for testing
set_param("ARMING_CHECK", 0)
set_param("BRD_SAFETYENABLE", 0)
set_param("FS_THR_ENABLE", 0)

time.sleep(1)

# --------------------------------------------------
# MODE
# --------------------------------------------------
mode = 'GUIDED'
mode_id = master.mode_mapping()[mode]

master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id
)

time.sleep(2)

# --------------------------------------------------
# ZERO THROTTLE
# --------------------------------------------------
for _ in range(50):
    master.mav.manual_control_send(master.target_system,0,0,0,0,0)
    time.sleep(0.02)

# --------------------------------------------------
# ARM
# --------------------------------------------------
print("Arming...")

master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1,
    0,0,0,0,0,0
)

# confirm
while True:
    msg = master.recv_match(type='HEARTBEAT', blocking=True)
    if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
        print("ARMED SUCCESS")
        break

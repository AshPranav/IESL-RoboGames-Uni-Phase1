#!/usr/bin/env python3
from pymavlink import mavutil
import time

# --------------------------------------------------
# CONNECT
# --------------------------------------------------
print("Connecting...")
master = mavutil.mavlink_connection('udp:0.0.0.0:14550')
master.wait_heartbeat()
print("Connected")

# --------------------------------------------------
# SITL STABILIZE (IMPORTANT)
# --------------------------------------------------
print("Waiting for SITL sensors to stabilize...")
time.sleep(5)

# Disable all pre-arm checks (fix accel inconsistent)
def set_param(name, value):
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        name.encode(),
        float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )
    time.sleep(0.2)

set_param("ARMING_CHECK", 0)

# --------------------------------------------------
# WAIT EKF READY
# --------------------------------------------------
print("Waiting EKF ready...")
while True:
    msg = master.recv_match(type='EKF_STATUS_REPORT', blocking=True)
    if msg.flags & mavutil.mavlink.EKF_ATTITUDE:
        print("EKF ready")
        break

# --------------------------------------------------
# GUIDED MODE
# --------------------------------------------------
mode = "GUIDED"
mode_id = master.mode_mapping()[mode]

master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id
)
time.sleep(2)
print("GUIDED mode set")

# --------------------------------------------------
# ARM
# --------------------------------------------------
print("Arming...")
master.arducopter_arm()
master.motors_armed_wait()
print("Armed")

# --------------------------------------------------
# TAKEOFF
# --------------------------------------------------
target_alt = 5
print(f"Taking off to {target_alt}m")

master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0,
    0, 0, 0, 0,
    0, 0,
    target_alt
)

# Monitor altitude
while True:
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    alt = msg.relative_alt / 1000.0
    print(f"Alt: {alt:.2f}m")

    if alt >= target_alt - 0.3:
        print("Reached altitude")
        break

# --------------------------------------------------
# HOVER
# --------------------------------------------------
print("Hovering 5 seconds...")
time.sleep(5)

# --------------------------------------------------
# LAND
# --------------------------------------------------
print("Landing...")

mode_id = master.mode_mapping()['LAND']
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id
)

# Wait touchdown
while True:
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    alt = msg.relative_alt / 1000.0
    print(f"Alt: {alt:.2f}m")

    if alt < 0.3:
        print("Landed")
        break

# --------------------------------------------------
# DISARM
# --------------------------------------------------
print("Disarming...")
master.arducopter_disarm()
master.motors_disarmed_wait()
print("Done")

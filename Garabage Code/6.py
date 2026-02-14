#!/usr/bin/env python3

from pymavlink import mavutil
import time

def takeoff(m, alt):
    print(f'Takeoff to {alt} m...')
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0,
        0, 0,
        float(alt)
    )
    # wait for altitude near target using GLOBAL_POSITION_INT
    end = time.time() + 20
    while time.time() < end:
        msg = m.recv_match(type='GLOBAL_POSITION_INT', timeout=1)
        if not msg:
            continue
        alt_m = msg.relative_alt / 1000.0  # millimeters, relative to home
        print(f' Alt {alt_m:.2f} m')
        if alt_m >= alt - 0.5:
            print('Reached takeoff altitude')
            return True, alt_m
    print('Takeoff confirmation timed out; continuing')
    return False


def land_and_disarm(master):
    # 1. SWITCH TO LAND MODE
    print("Creating LAND command...")
    # Get the ID for 'LAND' mode from the available modes
    mode_id = master.mode_mapping()['LAND']
    
    # Send the mode switch command
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    print("Sent LAND mode command.")

    # 2. MONITOR ALTITUDE (Wait for touchdown)
    print("Waiting for landing...")
    while True:
        # Listen for altitude data (GLOBAL_POSITION_INT is reliable)
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        
        # relative_alt is in millimeters. Convert to meters.
        alt_m = msg.relative_alt / 1000.0
        print(f"  Current Altitude: {alt_m:.2f}m")

        # If we are close to the ground (e.g., < 20cm), break loop
        if alt_m < 0.2:
            print("Touchdown detected!")
            break
        time.sleep(0.5)

    # 3. DISARM MOTORS
    print("Disarming...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,    # Confirmation
        0,    # Param1: 0 = DISARM, 1 = ARM
        0, 0, 0, 0, 0, 0 # Unused parameters
    )

    # 4. WAIT FOR CONFIRMATION
    master.motors_disarmed_wait()
    print("✓ Drone Disarmed successfully.")



# Connect to vehicle
print("Connecting to vehicle...")
master = mavutil.mavlink_connection('udp:0.0.0.0:14550')
master.wait_heartbeat()
print(f"Connected to system {master.target_system}, component {master.target_component}")

# --------------------------------------------------
# Set required parameters (no checking, no confirm)
# --------------------------------------------------
print("Setting parameters...")

params = {
    'ACRO_BAL_ROLL': 0.0,
    'ACRO_BAL_PITCH': 0.0,
    'ATC_RAT_RLL_I': 0.18,
    'ATC_RAT_PIT_I': 0.18,
    'ATC_RAT_YAW_I': 0.018,
}

for name, value in params.items():
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        name.encode('utf-8'),
        value,
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )
    time.sleep(0.2)  # small delay to avoid flooding

print("Parameters sent.")

# --------------------------------------------------
# Switch to GUIDED mode
# --------------------------------------------------
mode = 'GUIDED'
mode_id = master.mode_mapping()[mode]

master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id
)

time.sleep(1)
print("Mode set to GUIDED.")

# --------------------------------------------------
# Arm the drone (no confirmation checks)
# --------------------------------------------------
master.arducopter_arm()
master.motors_armed_wait()
print("Arm command sent.")


print("Drone is armed.")

print("Sending throttle...")
takeoff(master, 5)
land_and_disarm(master)


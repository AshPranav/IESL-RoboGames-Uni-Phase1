import cv2
from pymavlink import mavutil
import time
import numpy as np

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

def hover(seconds):
    print(f"Hovering for {seconds} seconds...")
    time.sleep(seconds)


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
        if alt_m < 0.5:
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



def main():
    # Create a connection to the MAVLink device (e.g., a drone)
    print("Connecting to vehicle...")
    master = mavutil.mavlink_connection('udp:0.0.0.0:14550')

    # Wait for the heartbeat message to confirm the connection    print("Waiting for heartbeat...")
    master.wait_heartbeat()

    print(f"Connected to system {master.target_system}, component {master.target_component}")

    # --- SITL stabilization ---
    print("Waiting for SITL sensors to stabilize...")
    time.sleep(5)

    
    # Disable arming checks (fixes accel inconsistent in SITL)

    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        b'ARMING_CHECK',
        0,
        mavutil.mavlink.MAV_PARAM_TYPE_INT32
    )
    time.sleep(1)
    print("Arming checks disabled for SITL")

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

    master.arducopter_arm()
    print("Arm command sent.")

    start_time = time.time()
    timeout = 10  # seconds

    while time.time() - start_time < timeout:
        master.recv_match(type='HEARTBEAT', blocking=True)
        if master.motors_armed():
            print("Armed normally")
            break
        
    



    if not master.motors_armed():
        print("Motors not armed after timeout, forcing arm...")
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,  # 1 = ARM
            21196,  # Force arm code
            0, 0, 0, 0, 0
        )
        master.motors_armed_wait()
        print("Drone is armed (forced).")

    print("Sending throttle...")

    k = takeoff(master, 3)
    if (k[0]):
        print(k[1])

    control_window = np.zeros((100, 300), dtype=np.uint8)

    while True:
        # Show the control window
        cv2.imshow("Drone Control (Press x to Land)", control_window)
        
        # Check for key press (wait 100ms)
        key = cv2.waitKey(100) & 0xFF
        
        # If 'x' is pressed
        if key == ord('x'):
            print("❌ 'x' pressed! Landing now...")
            break


    land_and_disarm(master)
    
if __name__ == "__main__":
    main()
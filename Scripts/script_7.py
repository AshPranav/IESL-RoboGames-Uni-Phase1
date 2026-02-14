import cv2
from pymavlink import mavutil
import time
import socket
import struct
import numpy as np

def recvall(sock, size):
    data = b""
    while len(data) < size:
        packet = sock.recv(size - len(data))
        if not packet: return None
        data += packet
    return data

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
            break
    print('Takeoff confirmation timed out; continuing')
    

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

#PHASE 1

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


#PHASE 2

    # ---- INITIALIZE CAMERA SOCKET ----
    HOST = "127.0.0.1" 
    PORT = 5599
    cam_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    cam_sock.connect((HOST, PORT))
    print("✅ Connected to camera stream")


#PHASE 3

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
    takeoff(master, 3) # [cite: 108]
    

     # --- PHASE 4: NAVIGATION ---
    forward_speed = 0.4  # m/s
    kp = 0.004           # Steering sensitivity (tuned for 0.6 speed)
    print("🚀 Starting Autonomous Line Following...")

    try:
        while True:
            # Receive Frame
            header = recvall(cam_sock, 4)
            if not header: break
            width, height = struct.unpack("<HH", header)

            payload = recvall(cam_sock, width * height)
            if not payload: break

            # Process Image
            frame = np.frombuffer(payload, dtype=np.uint8).reshape((height, width)).copy()
            blurred = cv2.GaussianBlur(frame, (5, 5), 0)
            _, mask = cv2.threshold(blurred, 150, 255, cv2.THRESH_BINARY)
            
            # Portrait Filter (Ignore side wall banners)
            margin = int(width * 0.25)
            mask[:, 0:margin] = 0
            mask[:, width-margin:width] = 0

            # Line Detection
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            error = 0
            line_found = False

            for cnt in contours:
                if 100 < cv2.contourArea(cnt) < 6000:
                    M = cv2.moments(cnt)
                    if M["m00"] > 0:
                        cx = int(M["m10"] / M["m00"])
                        error = cx - (width // 2)
                        cv2.circle(frame, (cx, height // 2), 10, (255), -1)
                        line_found = True
                        break

                else:
                    # If line is lost (shadow or gap), stop moving forward to stay safe
                    error = 0
                    forward_speed = 0.1
 # 3. SEND VELOCITY COMMAND (The "Action")
        # Ensure we are using MAV_FRAME_BODY_NED so X is always FORWARD
            side_speed = -(error * kp) 

            # Bitmask 0b0000111111000111 (Decimal: 4039) 
            # This tells ArduPilot: "Ignore position/accel, use Velocity X, Y, Z"
            master.mav.set_position_target_local_ned_send(
                0,                          # time_boot_ms
                master.target_system,       # target_system
                master.target_component,    # target_component
                mavutil.mavlink.MAV_FRAME_BODY_NED, # Relative to drone front
                0b0000111111000111,         # Type Mask: Enable Vx, Vy, Vz
                0, 0, 0,                    # Position X, Y, Z (Ignored)
                forward_speed,              # Velocity X (Forward)
                side_speed,                 # Velocity Y (Right)
                0,                          # Velocity Z (Down)
                0, 0, 0,                    # Acceleration (Ignored)
                0, 0                        # Yaw / Yaw Rate (Ignored)
            )




            # Telemetry & Exit
            cv2.imshow("Drone Camera", frame)
            cv2.imshow("Drone Vision (Line Following)", mask)

            if cv2.waitKey(1) & 0xFF == ord('x'):
                break

    except KeyboardInterrupt:
        print("\nManual Interrupt.")

    # --- PHASE 5: CLEANUP ---
    print("Cleaning up...")
    cam_sock.close()
    cv2.destroyAllWindows()
    land_and_disarm(master)


if __name__ == "__main__":
    main()
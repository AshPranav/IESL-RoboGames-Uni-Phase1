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
    

    forward_speed = 0.6  # m/s
    kp = 0.007           # Sensitivity
             # Initial state

#PHASE 4
    while True:
        header = recvall(cam_sock, 4)
        if header is None: break
        width, height = struct.unpack("<HH", header)

        img_size = width * height
        payload = recvall(cam_sock, img_size)
        if payload is None: break

        # 1. Step 1: Create writable copy [cite: 136]
        frame = np.frombuffer(payload, dtype=np.uint8).reshape((height, width)).copy()

        # 2. Step 2: Portrait Masking (Ignore side banners) [cite: 155]
        blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        _, mask = cv2.threshold(blurred, 150, 255, cv2.THRESH_BINARY)
        
        margin = int(width * 0.25)
        mask[:, 0:margin] = 0
        mask[:, width-margin:width] = 0

        # 3. Step 3: Differentiate Line vs Pad 
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        

        line_target = None
        error = 0

        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            # Narrow strip is the line [cite: 155]
            if 100 < area < 5000:
                line_target = cnt
            # Large block is the Takeoff/Landing Pad [cite: 154]
            elif area >= 5000:
                cv2.drawContours(frame, [cnt], -1, (127), 2) # Outline the pad

        # 4. Step 4: Calculate Error for ArduPilot 
        if line_target is not None:
            M = cv2.moments(line_target)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                cv2.circle(frame, (cx, cy), 10, (255), -1)
                
                error = cx - (width // 2)
                # You will send this 'error' to ArduPilot via MAVLink [cite: 219]
                print(f"Line Center: {cx} | Steering Error: {error}")

        cv2.imshow("Drone Camera", frame)
        cv2.imshow("Mask", mask)

        if cv2.waitKey(1) == 27: break   


        # 1. GET THE CAMERA DATA (Add your camera connection/recv logic here)
        # For now, let's assume you've integrated the camera loop and have 'error'
        
        # 2. CALCULATE STEERING (PID Control)
        # If error is positive, drone moves right. If negative, moves left.
        side_speed = -(error * kp) 

        # 3. SEND VELOCITY COMMAND TO ARDUPILOT
        master.mav.set_position_target_local_ned_send(
            0,       # time_boot_ms
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED, # Relative to drone heading
            0b0000111111000111, # Type mask: use only velocities
            0, 0, 0,            # x, y, z positions (ignored)
            forward_speed,      # Forward velocity (X)
            side_speed,         # Right/Left velocity (Y)
            0,                  # Down velocity (Z)
            0, 0, 0,            # accelerations (ignored)
            0, 0                # yaw, yaw_rate (ignored)
        )

        # 4. SAFETY EXIT

        if cv2.waitKey(1) & 0xFF == ord('x'):
            break
        cam_sock.close()
        cv2.destroyAllWindows()

    land_and_disarm(master)
    
if __name__ == "__main__":
    main()
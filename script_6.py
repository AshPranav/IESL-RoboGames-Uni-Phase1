import cv2
from pymavlink import mavutil
import time
import socket
import struct
import numpy as np

def recvall(sock, size):
    """Ensures full data payload is received from the TCP stream."""
    data = b""
    while len(data) < size:
        packet = sock.recv(size - len(data))
        if not packet: return None
        data += packet
    return data

def takeoff(m, alt):
    """Arms and takes off to specified altitude using MAVLink commands."""
    print(f'🚀 Takeoff initiated: Target {alt}m')
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, float(alt)
    )

    # Monitor altitude until target is reached
    end = time.time() + 20
    while time.time() < end:
        msg = m.recv_match(type='GLOBAL_POSITION_INT', timeout=1)
        if not msg: continue
        alt_m = msg.relative_alt / 1000.0
        print(f'  Current Alt: {alt_m:.2f}m')
        if alt_m >= alt - 0.3: # Threshold for 'reached'
            print('✅ Target altitude reached.')
            return True
    return False

def land_and_disarm(master):
    """Switches to LAND mode and confirms disarm."""
    print("🛬 Landing sequence started...")
    mode_id = master.mode_mapping()['LAND']
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )

    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        alt_m = msg.relative_alt / 1000.0
        if alt_m < 0.3:
            print("🏁 Touchdown detected.")
            break
        time.sleep(0.5)

    # Disarm motors
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0 
    )
    master.motors_disarmed_wait()
    print("🔒 Motors Disarmed. Mission Complete.")

def main():
    # --- PHASE 1: SYSTEM INIT ---
    print("Connecting to ArduPilot...")
    master = mavutil.mavlink_connection('udp:0.0.0.0:14550')
    master.wait_heartbeat()
    print(f"Connected (SysID: {master.target_system})")

    # Disable arming checks for SITL stability
    master.mav.param_set_send(master.target_system, master.target_component, b'ARMING_CHECK', 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    time.sleep(1)

    # --- PHASE 2: CAMERA INIT ---
    HOST, PORT = "127.0.0.1", 5599
    cam_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        cam_sock.connect((HOST, PORT))
        print("📷 Camera Stream Connected.")
    except Exception as e:
        print(f"❌ Camera Failed: {e}")
        return

    # --- PHASE 3: TAKEOFF ---
    mode_id = master.mode_mapping()['GUIDED']
    master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)
    time.sleep(1)
    
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
    master.motors_armed_wait()
    takeoff(master, 3)

    # --- PHASE 4: NAVIGATION ---
    forward_speed = 0.6  # m/s
    kp = 0.007           # Steering sensitivity (tuned for 0.6 speed)
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
            cv2.imshow("Drone Vision (Line Following)", frame)
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
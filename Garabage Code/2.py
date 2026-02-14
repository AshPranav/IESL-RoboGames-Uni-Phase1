from pymavlink import mavutil
import time

# CONNECT
master = mavutil.mavlink_connection('udp:0.0.0.0:14550')
master.wait_heartbeat()
print("Connected to ArduCopter")

def set_param(name, value, param_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32):
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        name.encode('utf-8'),
        float(value),
        param_type
    )
    time.sleep(0.3)
    # Wait for param_value confirmation
    msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
    if msg and msg.param_id == name:
        print(f"Set {name} = {value}")
    else:
        print(f"Warning: {name} may not have been set")

print("Configuring parameters...")
set_param("ARMING_CHECK", 0)  # Disable arming checks
set_param("FS_THR_ENABLE", 0)  # Disable throttle failsafe
set_param("BRD_SAFETYENABLE", 0)  # Disable safety switch
set_param("DISARM_DELAY", 0)  # Disable auto-disarm
time.sleep(1)

# SET GUIDED MODE
print("Switching to GUIDED mode...")
mode = 'GUIDED'
mode_id = master.mode_mapping()[mode]
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id
)

# Wait for mode change confirmation
for _ in range(10):
    msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
    if msg:
        current_mode = msg.custom_mode
        if current_mode == mode_id:
            print("GUIDED mode confirmed")
            break
time.sleep(1)

# Send neutral RC commands
print("Sending neutral RC input...")
for _ in range(30):
    master.mav.manual_control_send(
        master.target_system,
        0,  # roll
        0,  # pitch
        0,  # throttle (neutral/low)
        0,  # yaw
        0   # buttons
    )
    time.sleep(0.05)

# ARM THE VEHICLE
print("Arming motors (normal)...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,  # confirmation
    1,  # 1 = arm, 0 = disarm
    0,  # normal arm (not force)
    0, 0, 0, 0, 0
)

# Wait for arm confirmation
armed = False
for i in range(10):
    msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
    if msg:
        armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        if armed:
            print("✓ ARM SUCCESS")
            break
    time.sleep(0.3)

if not armed:
    print("✗ Normal arm failed. Checking pre-arm messages...")
    # Try to get STATUSTEXT messages that explain why
    status_messages = []
    for _ in range(10):
        msg = master.recv_match(type='STATUSTEXT', blocking=True, timeout=0.5)
        if msg:
            status_messages.append(msg.text.decode('utf-8', errors='ignore') if isinstance(msg.text, bytes) else msg.text)
            print(f"  → {status_messages[-1]}")
    
    # If we got "pre arm roll pitch", try force arm
    if any("roll" in str(m).lower() or "pitch" in str(m).lower() for m in status_messages):
        print("\nRoll/Pitch error detected. Trying force arm...")
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,      # 1 = arm
            21196,  # force arm magic number
            0, 0, 0, 0, 0
        )
        time.sleep(1)
        # Check if force arm worked
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg:
            armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            if armed:
                print("✓ FORCE ARM SUCCESS")
            else:
                print("✗ Force arm also failed")
                exit(1)
        else:
            print("✗ No response to force arm")
            exit(1)
    else:
        exit(1)

# SPIN MOTORS FOR 3 SECONDS
print("\nSpinning motors for 3 seconds...")
start_time = time.time()
while time.time() - start_time < 3.0:
    master.mav.manual_control_send(
        master.target_system,
        0,      # roll
        0,      # pitch
        300,    # throttle (adjust 0-1000 range)
        0,      # yaw
        0       # buttons
    )
    time.sleep(0.05)

# STOP MOTORS
print("Stopping motors...")
for _ in range(20):
    master.mav.manual_control_send(
        master.target_system,
        0, 0, 0, 0, 0
    )
    time.sleep(0.05)

# DISARM
print("Disarming...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0,  # 0 = disarm
    0,
    0, 0, 0, 0, 0
)
time.sleep(1)
print("Done!")
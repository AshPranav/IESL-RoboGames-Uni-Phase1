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
    time.sleep(0.5)
    print(f"Set {name} = {value}")

print("Configuring parameters...")
set_param("ARMING_CHECK", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
set_param("ACRO_BAL_ROLL", 0)
set_param("ACRO_BAL_PITCH", 0)
set_param("FS_THR_ENABLE", 0)
set_param("BRD_SAFETYENABLE", 0)
set_param("DISARM_DELAY", 0)
set_param("ARMING_REQUIRE", 0)

print("\n🔄 Rebooting autopilot...")
master.reboot_autopilot()
time.sleep(5)

print("Reconnecting...")
master.close()
master = mavutil.mavlink_connection('udp:0.0.0.0:14550')
master.wait_heartbeat()
print("✓ Reconnected\n")

# WAIT FOR EKF TO INITIALIZE
print("⏳ Waiting for EKF to initialize...")
ekf_ready = False
for i in range(30):  # Wait up to 30 seconds
    msg = master.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=1)
    if msg:
        # Check if EKF is healthy (flags bit 0 = attitude valid, bit 3 = horizontal position valid)
        if msg.flags & 0x09:  # Basic flags
            ekf_ready = True
            print(f"✓ EKF ready after {i+1} seconds")
            break
    
    # Also check STATUSTEXT for "origin set" messages
    status = master.recv_match(type='STATUSTEXT', blocking=False)
    if status and 'origin set' in status.text:
        print(f"  {status.text}")
    
    time.sleep(1)

if not ekf_ready:
    print("⚠ EKF not fully ready, but proceeding anyway...")

time.sleep(2)  # Extra safety margin

# SET GUIDED MODE
print("\nSwitching to GUIDED mode...")
mode = 'GUIDED'
mode_id = master.mode_mapping()[mode]
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id
)
time.sleep(2)

msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
if msg and msg.custom_mode == mode_id:
    print("✓ GUIDED mode confirmed")

# Send neutral RC commands
print("\nSending neutral RC input...")
for _ in range(30):
    master.mav.manual_control_send(
        master.target_system,
        0, 0, 0, 0, 0
    )
    time.sleep(0.05)

# ARM THE VEHICLE
print("\n⚡ Arming motors...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1,  # arm
    0,  # normal arm
    0, 0, 0, 0, 0
)

# Wait and check armed status
time.sleep(2)
armed = False
for i in range(15):
    msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
    if msg:
        armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        if armed:
            print("✓ ARMED SUCCESSFULLY!\n")
            break
    
    # Check for any status messages
    status_msg = master.recv_match(type='STATUSTEXT', blocking=False)
    if status_msg:
        print(f"  {status_msg.text}")
    
    time.sleep(0.5)

if not armed:
    print("\n✗ Normal ARM FAILED - checking messages...")
    for _ in range(5):
        msg = master.recv_match(type='STATUSTEXT', blocking=True, timeout=1)
        if msg:
            print(f"  → {msg.text}")
    exit(1)

# SPIN MOTORS FOR 5 SECONDS
print("🚁 Spinning motors for 5 seconds...")
start_time = time.time()
throttle = 350  # Start with lower throttle

while time.time() - start_time < 5.0:
    master.mav.manual_control_send(
        master.target_system,
        0,         # roll
        0,         # pitch
        throttle,  # throttle (0-1000)
        0,         # yaw
        0
    )
    time.sleep(0.05)

# STOP MOTORS
print("\n⏹ Stopping motors...")
for _ in range(40):
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
    0,  # disarm
    0,
    0, 0, 0, 0, 0
)
time.sleep(2)

msg = master.recv_match(type='HEARTBEAT', blocking=True)
disarmed = not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
print("✓ DISARMED" if disarmed else "⚠ Still armed (may auto-disarm)")

print("\n✅ Test complete!")
from pymavlink import mavutil
import time

# --------------------------------------------------
# CONNECT
# --------------------------------------------------
master = mavutil.mavlink_connection('udp:0.0.0.0:14550')
master.wait_heartbeat()
print("Connected to ArduCopter")

# --------------------------------------------------
# Helper: set parameter
# --------------------------------------------------
'''def set_param(name, value):
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        name.encode('utf-8'),
        float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )
    time.sleep(0.3)
    print(f"Set {name} -> {value}")

# --------------------------------------------------
# BASIC FIXES (SAFE FOR SITL/BENCH)
# --------------------------------------------------
print("Configuring parameters...")

set_param("ARMING_CHECK", 0)      # disable all arming checks (for testing)
set_param("FS_THR_ENABLE", 0)     # disable throttle failsafe
set_param("BRD_SAFETYENABLE", 0)  # disable safety switch

time.sleep(1)'''

# --------------------------------------------------
# SET MODE GUIDED
# --------------------------------------------------
mode = 'GUIDED'
mode_id = master.mode_mapping()[mode]

master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id
)

print("Switching to GUIDED...")
time.sleep(2)

# --------------------------------------------------
# WAIT FOR EKF (important)
# --------------------------------------------------
print("Waiting for EKF to be ready...")

start = time.time()
while time.time() - start < 10:
    msg = master.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=1)
    if msg:
        # bit 0 = attitude valid
        if msg.flags & 1:
            print("EKF ready")
            break

# --------------------------------------------------
# TRY NORMAL ARM
# --------------------------------------------------
print("Arming motors (normal)...")

master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1,  # arm
    0,  # normal arm
    0, 0, 0, 0, 0
)

time.sleep(3)

# --------------------------------------------------
# CHECK IF ARMED
# --------------------------------------------------
msg = master.recv_match(type='HEARTBEAT', blocking=True)
armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED

if not armed:
    print("Normal arm failed → force arming")

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,
        21196,  # force arm
        0, 0, 0, 0, 0
    )
    time.sleep(2)

print("Armed!")

# --------------------------------------------------
# SPIN MOTORS ONLY
# --------------------------------------------------
print("Sending throttle...")

master.mav.manual_control_send(
    master.target_system,
    0,      # roll
    0,      # pitch
    300,    # throttle (0-1000)
    0,      # yaw
    0
)

print("Motors should spin slowly.")

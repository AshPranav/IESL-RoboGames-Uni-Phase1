from pymavlink import mavutil
import time

print("Connecting...")
master = mavutil.mavlink_connection('udp:0.0.0.0:14550')
master.wait_heartbeat()
print("Connected")

# ensure autopilot component
master.target_component = 1

# --------------------------------------------------
# helper
# --------------------------------------------------
def set_param(name, value):
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        name.encode(),
        float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )
    time.sleep(0.5)

# --------------------------------------------------
# FIX COMMON BLOCKERS
# --------------------------------------------------
print("Fixing common blockers...")

set_param("ACRO_BAL_ROLL", 1)
set_param("ACRO_BAL_PITCH", 1)
set_param("BRD_SAFETYENABLE", 0)   # disable safety switch
set_param("ARMING_CHECK", 1)       # keep checks ON for diagnostics

time.sleep(2)

# --------------------------------------------------
# LISTEN FOR PREARM MESSAGES
# --------------------------------------------------
print("\nListening for pre-arm messages...")
print("Try to arm now...\n")

start = time.time()
while time.time() - start < 5:
    msg = master.recv_match(blocking=False)
    if msg and msg.get_type() == "STATUSTEXT":
        print(msg.text)

# --------------------------------------------------
# TRY STABILIZE MODE ARM
# --------------------------------------------------
print("\nSwitching to STABILIZE mode")

mode = 'STABILIZE'
mode_id = master.mode_mapping()[mode]

master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id
)

time.sleep(2)

print("Attempting arm...")

master.arducopter_arm()

try:
    master.motors_armed_wait(timeout=5)
    print("\n✅ ARMED SUCCESSFULLY")
except:
    print("\n❌ ARM FAILED")

# --------------------------------------------------
# PRINT FINAL ERRORS
# --------------------------------------------------
print("\nFinal status messages:\n")

for _ in range(20):
    msg = master.recv_match(blocking=False)
    if msg and msg.get_type() == "STATUSTEXT":
        print(msg.text)
    time.sleep(0.1)

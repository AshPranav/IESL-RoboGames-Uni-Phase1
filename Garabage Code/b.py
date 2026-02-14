from pymavlink import mavutil
import time

# -----------------------------------------
# CONNECT
# -----------------------------------------
master = mavutil.mavlink_connection('udp:0.0.0.0:14550')
master.wait_heartbeat()
print("Connected:", master.target_system, master.target_component)
master.target_component = 1

# -----------------------------------------
# PARAM SET FUNCTION
# -----------------------------------------
def set_param(name, value):
    print("Setting", name, "->", value)
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        name.encode(),
        float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )
    time.sleep(0.5)

# -----------------------------------------
# FIX RATE CONTROLLER PARAMS (THE FIX!)
# -----------------------------------------
print("=== Setting Rate Controller Params ===")
set_param("ATC_RAT_RLL_I", 0.135)
set_param("ATC_RAT_RLL_P", 0.135)
set_param("ATC_RAT_RLL_D", 0.0036)

set_param("ATC_RAT_PIT_I", 0.135)
set_param("ATC_RAT_PIT_P", 0.135)
set_param("ATC_RAT_PIT_D", 0.0036)

set_param("ATC_RAT_YAW_I", 0.018)
set_param("ATC_RAT_YAW_P", 0.180)
set_param("ATC_RAT_YAW_D", 0.0)

# -----------------------------------------
# DISABLE CHECKS
# -----------------------------------------
set_param("ARMING_CHECK", 0)
set_param("FS_THR_ENABLE", 0)
set_param("BRD_SAFETYENABLE", 0)
set_param("ACRO_BAL_ROLL", 0)
set_param("ACRO_BAL_PITCH", 0)
set_param("ARMING_REQUIRE", 0)
time.sleep(2)

# -----------------------------------------
# SET NEUTRAL RC VALUES
# -----------------------------------------
print("Setting neutral RC...")
for i in range(10):
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        1500, 1500, 1000, 1500, 0, 0, 0, 0
    )
    time.sleep(0.1)

# -----------------------------------------
# SET MODE STABILIZE
# -----------------------------------------
mode = "STABILIZE"
mode_id = master.mode_mapping()[mode]
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id
)
print("Switching to STABILIZE...")
time.sleep(2)

# -----------------------------------------
# WAIT EKF READY
# -----------------------------------------
print("Waiting EKF...")
while True:
    msg = master.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=1)
    if msg:
        print("EKF OK")
        break

# -----------------------------------------
# ARM
# -----------------------------------------
print("Arming motors...")
master.arducopter_arm()
time.sleep(2)

if not master.motors_armed():
    print("Normal arm failed → force arm")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 21196, 0, 0, 0, 0, 0
    )
    master.motors_armed_wait()

print("ARMED")

# -----------------------------------------
# SPIN MOTORS SLOW
# -----------------------------------------
print("Sending throttle...")
for i in range(50):
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        1500, 1500, 1100, 1500, 0, 0, 0, 0
    )
    time.sleep(0.1)

# -----------------------------------------
# DISARM
# -----------------------------------------
print("Stopping")
master.arducopter_disarm()
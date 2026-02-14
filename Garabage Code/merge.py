from pymavlink import mavutil
import time

# --------------------------------------------------
# CONNECT
# --------------------------------------------------
master = mavutil.mavlink_connection('udp:0.0.0.0:14550')
master.wait_heartbeat()
print("Connected to ArduCopter")

# --------------------------------------------------
# PARAM SETTER
# --------------------------------------------------
def set_param(name, value):
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        name.encode(),
        float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )
    time.sleep(0.3)
    print(f"Set {name} -> {value}")

# --------------------------------------------------
# BASIC SAFE PARAMS (SITL/BENCH ONLY)
# --------------------------------------------------
print("Configuring parameters...")

set_param("ARMING_CHECK", 0)
set_param("FS_THR_ENABLE", 0)
set_param("BRD_SAFETYENABLE", 0)
set_param("DISARM_DELAY", 0)

# allow arming without GPS
set_param("GPS_TYPE", 0)
set_param("EK3_SRC1_POSXY", 0)
set_param("EK3_SRC1_POSZ", 0)
set_param("EK3_SRC1_VELXY", 0)
set_param("EK3_SRC1_VELZ", 0)

time.sleep(1)

# --------------------------------------------------
# MODE SWITCH (IMPORTANT)
# --------------------------------------------------
def set_mode(name):
    mode_id = master.mode_mapping()[name]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    print("Switching to", name)
    time.sleep(2)

set_mode("STABILIZE")
set_mode("GUIDED")

# --------------------------------------------------
# WAIT FOR EKF
# --------------------------------------------------
print("Waiting for EKF...")

while True:
    msg = master.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=1)
    if msg and (msg.flags & 1):
        print("EKF OK")
        break

# --------------------------------------------------
# 🔥 SET HOME MANUALLY (CRITICAL FIX)
# --------------------------------------------------
print("Setting home position...")

master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_HOME,
    0,
    1,  # use current position
    0,0,0,0,0,0
)

time.sleep(2)
print("Home position set")

# --------------------------------------------------
# ARM
# --------------------------------------------------
print("Arming motors...")

master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1,
    0,0,0,0,0,0
)

# confirm arm
while True:
    msg = master.recv_match(type='HEARTBEAT', blocking=True)
    if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
        print("ARMED SUCCESS")
        break

# --------------------------------------------------
# SPIN MOTORS
# --------------------------------------------------
print("Spinning motors...")

start = time.time()
while time.time() - start < 3:
    master.mav.manual_control_send(
        master.target_system,
        0,
        0,
        300,   # throttle
        0,
        0
    )
    time.sleep(0.05)

# --------------------------------------------------
# STOP
# --------------------------------------------------
print("Stopping motors...")

for _ in range(20):
    master.mav.manual_control_send(master.target_system,0,0,0,0,0)
    time.sleep(0.05)

# --------------------------------------------------
# DISARM
# --------------------------------------------------
print("Disarming...")

master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0,
    0,0,0,0,0,0
)

print("Done.")

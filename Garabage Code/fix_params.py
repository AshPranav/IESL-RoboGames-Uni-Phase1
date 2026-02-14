from pymavlink import mavutil
import time

master = mavutil.mavlink_connection('udp:0.0.0.0:14550')
master.wait_heartbeat()

print("Connected")

def set_param(name, value):
    print("Setting", name)
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        name.encode(),
        float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )
    time.sleep(1)

# ---- FIX THE ERROR ----
set_param("ACRO_BAL_ROLL", 1.0)
set_param("ACRO_BAL_PITCH", 1.0)

print("Rebooting autopilot...")

# reboot autopilot
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
    0,
    1,0,0,0,0,0,0
)

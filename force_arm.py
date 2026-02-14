from pymavlink import mavutil
import time

master = mavutil.mavlink_connection('udp:0.0.0.0:14550')
master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))

mode = "GUIDED"

mode_id = master.mode_mapping()[mode]
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id
)

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
master.motors_armed_wait()
print("Armed!")

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
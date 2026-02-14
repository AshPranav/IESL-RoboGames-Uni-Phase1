"""MAVLink flight control helpers extracted from the original script.

Functions provided:
- connect(uri)
- disable_arming_checks(master)
- set_params(master, params)
- set_mode(master, mode)
- arm(master, timeout=10, force=False)
- takeoff(master, alt)
- land_and_disarm(master)
- set_velocity_body(master, vx, vy, vz)
"""
import time
from pymavlink import mavutil


def connect(uri: str):
    master = mavutil.mavlink_connection(uri)
    master.wait_heartbeat()
    return master


def disable_arming_checks(master):
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        b'ARMING_CHECK',
        0,
        mavutil.mavlink.MAV_PARAM_TYPE_INT32,
    )
    time.sleep(0.5)


def set_params(master, params: dict):
    for name, value in params.items():
        master.mav.param_set_send(
            master.target_system,
            master.target_component,
            name.encode('utf-8'),
            float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        )
        time.sleep(0.15)


def set_mode(master, mode: str):
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id,
    )


def arm(master, timeout: float = 10, force: bool = False):
    master.arducopter_arm()
    start = time.time()
    while time.time() - start < timeout:
        master.recv_match(type='HEARTBEAT', blocking=True)
        if master.motors_armed():
            return True
    if not master.motors_armed() and force:
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,  # ARM
            21196,
            0, 0, 0, 0, 0,
        )
        master.motors_armed_wait()
        return True
    return master.motors_armed()


def takeoff(master, alt: float):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0,
        0, 0,
        float(alt),
    )
    # wait up to 20s for altitude
    end = time.time() + 20
    while time.time() < end:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', timeout=1)
        if not msg:
            continue
        alt_m = msg.relative_alt / 1000.0
        if alt_m >= alt - 0.5:
            break


def land_and_disarm(master):
    mode_id = master.mode_mapping().get('LAND')
    if mode_id is not None:
        master.mav.set_mode_send(
            master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )
    # wait for touchdown by monitoring relative_alt
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if not msg:
            time.sleep(0.2)
            continue
        alt_m = msg.relative_alt / 1000.0
        if alt_m < 0.5:
            break
        time.sleep(0.3)
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0,
        0, 0, 0, 0, 0, 0,
    )
    try:
        master.motors_disarmed_wait()
    except Exception:
        time.sleep(1)


def set_velocity_body(master, vx: float, vy: float, vz: float):
    # Use MAV_FRAME_BODY_NED so X is forward
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,  # enable vx, vy, vz only
        0, 0, 0,
        float(vx), float(vy), float(vz),
        0, 0, 0,
        0, 0,
    )

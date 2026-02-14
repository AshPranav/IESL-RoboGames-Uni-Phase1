#!/usr/bin/env python3
"""
Fix ArduPilot parameters that cause prearm check failures.

Fixes:
- ACRO_BAL_ROLL and ACRO_BAL_PITCH should be 0 for multicopters
- ATC_RAT_RLL_I, ATC_RAT_PIT_I, ATC_RAT_YAW_I must be > 0

Usage: ./venv/bin/python Task/fix_acro_params.py
"""
import time
from pymavlink import mavutil


def read_param(m, param_name):
    """Read a parameter value from the vehicle."""
    if isinstance(param_name, bytes):
        param_name = param_name.decode('utf-8')

    m.mav.param_request_read_send(
        m.target_system,
        m.target_component,
        param_name.encode('utf-8'),
        -1
    )

    timeout = time.time() + 5
    while time.time() < timeout:
        msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
        if msg and msg.param_id.strip('\x00') == param_name:
            return msg.param_value
    return None


def set_param(m, param_name, value):
    """Set a parameter value on the vehicle."""
    if isinstance(param_name, bytes):
        param_name = param_name.decode('utf-8')

    m.mav.param_set_send(
        m.target_system,
        m.target_component,
        param_name.encode('utf-8'),
        value,
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )

    # Wait for confirmation
    timeout = time.time() + 5
    while time.time() < timeout:
        msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
        if msg and msg.param_id.strip('\x00') == param_name:
            print(f"  ✓ {param_name} set to {msg.param_value}")
            return True
    print(f"  ✗ Failed to confirm {param_name}")
    return False


def main():
    print("Connecting to vehicle...")
    m = mavutil.mavlink_connection('udp:0.0.0.0:14550')

    print('Waiting for heartbeat...')
    m.wait_heartbeat()
    print(f'Connected to system {m.target_system}, component {m.target_component}')
    print()

    # Parameters to check and fix
    params_to_check = {
        'ACRO_BAL_ROLL': {'expected': 0.0, 'condition': 'eq'},
        'ACRO_BAL_PITCH': {'expected': 0.0, 'condition': 'eq'},
        'ATC_RAT_RLL_I': {'expected': 0.18, 'condition': 'gt'},
        'ATC_RAT_PIT_I': {'expected': 0.18, 'condition': 'gt'},
        'ATC_RAT_YAW_I': {'expected': 0.018, 'condition': 'gt'},
    }

    # Read current values
    print("Reading current parameter values...")
    current_values = {}
    for param_name in params_to_check:
        value = read_param(m, param_name)
        current_values[param_name] = value
        print(f"  {param_name}: {value}")
    print()

    # Check if they need to be fixed
    needs_fix = []
    for param_name, config in params_to_check.items():
        value = current_values[param_name]
        if value is None:
            continue
        
        if config['condition'] == 'eq':
            if value != config['expected']:
                needs_fix.append(param_name)
                print(f"⚠ {param_name} is {value}, should be {config['expected']}")
        elif config['condition'] == 'gt':
            if value <= 0:
                needs_fix.append(param_name)
                print(f"⚠ {param_name} is {value}, must be > 0 (will set to {config['expected']})")

    if not needs_fix:
        print("✓ All parameters are correct!")
        return

    print()
    print("Fixing parameters...")

    # Fix parameters
    for param_name in needs_fix:
        config = params_to_check[param_name]
        set_param(m, param_name, config['expected'])

    print()
    print("Verifying changes...")
    time.sleep(1)
    
    all_fixed = True
    for param_name in needs_fix:
        new_value = read_param(m, param_name)
        print(f"  {param_name}: {new_value}")
        config = params_to_check[param_name]
        if config['condition'] == 'eq' and new_value != config['expected']:
            all_fixed = False
        elif config['condition'] == 'gt' and new_value <= 0:
            all_fixed = False

    print()
    if all_fixed:
        print("✓ All parameters fixed successfully!")
        print("  The prearm checks should now pass.")
    else:
        print("✗ Some parameters failed to update. Try restarting ArduPilot SITL.")


if __name__ == '__main__':
    main()
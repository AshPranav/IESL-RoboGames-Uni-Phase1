from pymavlink import mavutil
import time

# CONNECT
master = mavutil.mavlink_connection('udp:0.0.0.0:14550')
master.wait_heartbeat()
print("Connected to ArduCopter\n")

def get_param(name):
    """Request and read a specific parameter"""
    master.mav.param_request_read_send(
        master.target_system,
        master.target_component,
        name.encode('utf-8'),
        -1
    )
    msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
    if msg and msg.param_id == name:
        return msg.param_value, msg.param_type
    return None, None

def get_all_params():
    """Request all parameters from the autopilot"""
    print("📥 Requesting all parameters...\n")
    
    # Request all parameters
    master.mav.param_request_list_send(
        master.target_system,
        master.target_component
    )
    
    params = {}
    start_time = time.time()
    
    while time.time() - start_time < 10:  # 10 second timeout
        msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
        if msg:
            param_id = msg.param_id
            params[param_id] = {
                'value': msg.param_value,
                'type': msg.param_type,
                'index': msg.param_index,
                'count': msg.param_count
            }
            
            # Show progress
            if len(params) % 50 == 0:
                print(f"  Received {len(params)} parameters...")
    
    return params

# METHOD 1: Check specific parameters we modified
print("=" * 60)
print("METHOD 1: CHECK SPECIFIC PARAMETERS")
print("=" * 60)

important_params = [
    "ARMING_CHECK",
    "ACRO_BAL_ROLL",
    "ACRO_BAL_PITCH",
    "FS_THR_ENABLE",
    "BRD_SAFETYENABLE",
    "DISARM_DELAY",
    "ARMING_REQUIRE",
    "FS_EKF_ACTION",
    "FS_EKF_THRESH"
]

print("\n📋 Checking important parameters:\n")
for param_name in important_params:
    value, param_type = get_param(param_name)
    if value is not None:
        type_names = {
            1: "UINT8",
            2: "INT8", 
            3: "UINT16",
            4: "INT16",
            5: "UINT32",
            6: "INT32",
            9: "REAL32"
        }
        type_str = type_names.get(param_type, f"Type {param_type}")
        print(f"  {param_name:20} = {value:10.2f}  ({type_str})")
    else:
        print(f"  {param_name:20} = NOT FOUND")

# METHOD 2: Get ALL parameters and save to file
print("\n" + "=" * 60)
print("METHOD 2: DOWNLOAD ALL PARAMETERS")
print("=" * 60)

all_params = get_all_params()

print(f"\n✓ Downloaded {len(all_params)} parameters\n")

# Save to file
filename = "autopilot_parameters.txt"
with open(filename, 'w') as f:
    f.write("=" * 70 + "\n")
    f.write(f"ArduCopter Parameters - {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
    f.write("=" * 70 + "\n\n")
    
    # Sort parameters alphabetically
    for param_name in sorted(all_params.keys()):
        param = all_params[param_name]
        f.write(f"{param_name:25} = {param['value']:12.4f}\n")

print(f"✓ All parameters saved to: {filename}\n")

# METHOD 3: Search for parameters by keyword
print("=" * 60)
print("METHOD 3: SEARCH PARAMETERS BY KEYWORD")
print("=" * 60)

def search_params(keyword):
    """Search for parameters containing a keyword"""
    print(f"\n🔍 Searching for '{keyword}':\n")
    found = False
    for param_name in sorted(all_params.keys()):
        if keyword.upper() in param_name.upper():
            param = all_params[param_name]
            print(f"  {param_name:25} = {param['value']:10.2f}")
            found = True
    if not found:
        print(f"  No parameters found containing '{keyword}'")

# Search examples
search_params("ACRO")
search_params("ARM")
search_params("EKF")

# METHOD 4: Show parameters different from defaults (if you know defaults)
print("\n" + "=" * 60)
print("METHOD 4: MODIFIED PARAMETERS")
print("=" * 60)

print("\n📝 Parameters we expect to have changed:\n")
expected_changes = {
    "ARMING_CHECK": 0,
    "ACRO_BAL_ROLL": 1,
    "ACRO_BAL_PITCH": 1,
    "FS_THR_ENABLE": 0,
    "DISARM_DELAY": 0,
    "ARMING_REQUIRE": 0
}

for param_name, expected_value in expected_changes.items():
    if param_name in all_params:
        actual_value = all_params[param_name]['value']
        status = "✓" if abs(actual_value - expected_value) < 0.01 else "✗"
        print(f"  {status} {param_name:20} = {actual_value:8.2f} (expected {expected_value})")
    else:
        print(f"  ✗ {param_name:20} = NOT FOUND")

print("\n" + "=" * 60)
print("DONE!")
print("=" * 60)

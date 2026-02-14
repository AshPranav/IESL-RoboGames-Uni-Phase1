import time
from pymavlink import mavutil

def main():
    print("Connecting...")
    m = mavutil.mavlink_connection('udp:0.0.0.0:14550')
    
    print("Waiting for heartbeat...")
    m.wait_heartbeat()
    print("Connected.")
    
    print("Requesting full parameter list...")
    
    # Ask for ALL parameters
    m.mav.param_request_list_send(
        m.target_system,
        m.target_component
    )
    
    params = {}
    
    while True:
        msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=5)
        
        if msg is None:
            break
        
        param_name = msg.param_id.strip('\x00')
        param_value = msg.param_value
        
        params[param_name] = param_value
        
        print(f"{param_name} = {param_value}")
        
        # Stop when all received
        if len(params) >= msg.param_count:
            break
    
    print(f"\nTotal parameters received: {len(params)}")
    
    # Save to file
    with open("full_param_list.txt", "w") as f:
        for k, v in sorted(params.items()):
            f.write(f"{k} = {v}\n")
    
    print("Saved to full_param_list.txt")

if __name__ == "__main__":
    main()
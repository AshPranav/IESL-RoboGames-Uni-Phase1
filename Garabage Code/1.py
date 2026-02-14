from pymavlink import mavutil

master = mavutil.mavlink_connection('udp:0.0.0.0:14550')
master.wait_heartbeat()
print("Connected")

# request ACRO_BAL params
for param in [b'ACRO_BAL_ROLL', b'ACRO_BAL_PITCH']:
    master.mav.param_request_read_send(master.target_system, master.target_component, param, -1)

# listen for param values
for i in range(2):
    msg = master.recv_match(type='PARAM_VALUE', blocking=True)
    print(msg)

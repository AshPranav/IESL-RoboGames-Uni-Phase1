from pymavlink import mavutil

master = mavutil.mavlink_connection('udp:0.0.0.0:14550')
master.wait_heartbeat()

print("Listening for errors...\n")

while True:
    msg = master.recv_match(blocking=True)
    if msg and msg.get_type() == "STATUSTEXT":
        print(msg.text)

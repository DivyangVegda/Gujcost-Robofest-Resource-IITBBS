from pymavlink import mavutil
import time

# 1. Create the connection
# For a simulator (SITL) running on the same machine:
connection_string = 'udpin:localhost:14550'

# For a USB connection (Linux):
# connection_string = '/dev/ttyACM0'
# For a USB connection (Windows):
# connection_string = 'COM3'

print(f"Connecting to {connection_string}...")
master = mavutil.mavlink_connection(connection_string)

# 2. Wait for the first heartbeat
# This sets the system and component ID of remote system for the link
print("Waiting for heartbeat...")
master.wait_heartbeat()

# 3. Success! Print the system details
print(f"Heartbeat from system (system {master.target_system} component {master.target_component})")
print("Connection successful!")

# 4. Optional: Request and print attitude data once to prove two-way data flow
while True:
    try:
        # Wait for an attitude message (timeout after 1 second)
        msg = master.recv_match(type='ATTITUDE', blocking=True, timeout=1)
        if msg:
            print(f"Roll: {msg.roll:.2f} | Pitch: {msg.pitch:.2f} | Yaw: {msg.yaw:.2f}")
            break # Exit after printing once
    except Exception as e:
        print(f"Error: {e}")
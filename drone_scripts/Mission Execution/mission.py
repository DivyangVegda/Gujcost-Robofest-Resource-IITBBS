from pymavlink import mavutil
import time
import math

# --- CONFIGURATION ---
connection_string = 'udpin:localhost:14551'
target_altitude = 5

# 1. CONNECT
print(f"Connecting to {connection_string}...")
master = mavutil.mavlink_connection(connection_string)
master.wait_heartbeat()
print(f"Connected to System {master.target_system}")

# --- FIX: Request Data Stream so we don't hang ---
master.mav.request_data_stream_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_POSITION, 10, 1)
print("Data stream requested.") 

# --- HELPER FUNCTIONS ---

def get_current_location():
    # Fetch the current location
    # timeout=1 prevents it from hanging forever if data stops
    msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
    if not msg:
        return 0, 0, 0 # Return 0s if no data
    return msg.x, msg.y, msg.z

def get_distance_metres(loc1, loc2):
    d_x = loc2[0] - loc1[0]
    d_y = loc2[1] - loc1[1]
    d_z = loc2[2] - loc1[2] 
    return math.sqrt(d_x**2 + d_y**2 + d_z**2)

def goto_location(north, east, down):
    print(f"\nCommanding movement to N:{north}, E:{east}, D:{down}...")
    
    # Send command
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b110111111000, # Position only mask
        north, east, down,
        0, 0, 0, 0, 0, 0, 0, 0)

    # Monitor movement
    while True:
        current_pos = get_current_location()
        dist = get_distance_metres(current_pos, (north, east, down))
        
        # Print status so you know it's working
        print(f" > Dist: {dist:.1f}m | Pos: N{current_pos[0]:.1f}, E{current_pos[1]:.1f}   ", end='\r')
        
        if dist < 1:
            print("\nTarget Reached!")
            break
        time.sleep(0.5)

def arm_and_takeoff(altitude):
    print("\nSetting Mode to GUIDED...")
    mode_id = master.mode_mapping()['GUIDED']
    master.mav.set_mode_send(
        master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)
    
    print("Arming...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0)
    master.motors_armed_wait()
    print("Armed!")

    print(f"Taking off to {altitude}m...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, altitude)
    
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg:
            alt = msg.relative_alt / 1000.0
            print(f" > Altitude: {alt:.1f}m", end='\r')
            if alt >= altitude * 0.95:
                print("\nTakeoff Complete!")
                break
        time.sleep(0.2)

# --- MAIN MISSION ---
try:
    arm_and_takeoff(target_altitude)
    time.sleep(2)

    # 1. Forward
    goto_location(20, 0, -target_altitude)
    
    # 2. Right
    goto_location(20, 20, -target_altitude)

    # 3. Home
    print("\nReturning Home...")
    goto_location(0, 0, -target_altitude)

    # Land
    print("\nLanding...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0)

except KeyboardInterrupt:
    print("\nStopped by user")
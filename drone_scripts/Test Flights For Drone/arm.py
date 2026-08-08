from pymavlink import mavutil
import time

# 1. Connect
connection_string = 'udpin:localhost:14551' # Update if using USB
print(f"Connecting to {connection_string}...")
master = mavutil.mavlink_connection(connection_string)
master.wait_heartbeat()
print(f"Connected to System {master.target_system}")

# 2. Function to check if the drone is ready to arm
def wait_until_ready():
    # In a real scenario, you check for Pre-Arm checks here
    # For now, we just ensure we are getting data
    print("Waiting for drone to be ready...")
    while True:
        msg = master.recv_match(type='SYS_STATUS', blocking=True)
        # Check specific flags if needed, for now we just wait a moment
        break

# 3. Arm Function
def arm_drone():
    print("Arming motors...")
    # MAV_CMD_COMPONENT_ARM_DISARM command
    # param1: 1 = ARM, 0 = DISARM
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0) # 1 here means ARM

    # Wait until the drone confirms it is armed
    master.motors_armed_wait()
    print("MOTORS ARMED! ⚠️")

# 4. Disarm Function
def disarm_drone():
    print("Disarming motors...")
    # param1: 0 = DISARM
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0) # 0 here means DISARM

    # Wait until the drone confirms it is disarmed
    master.motors_disarmed_wait()
    print("MOTORS DISARMED.")

# --- MAIN EXECUTION ---

# First, force GUIDED mode (best for script control)
# MAV_MODE_GUIDED = 4 (in custom_mode) or use set_mode helper
# We will use the helper method to request GUIDED mode
mode = 'GUIDED'
mode_id = master.mode_mapping()[mode]
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id)
print(f"Mode set to {mode}")

# Run the sequence
arm_drone()

print("Keeping motors spinning for 5 seconds...")
time.sleep(5)

disarm_drone()

print("Test Complete.")
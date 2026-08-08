import asyncio
import math
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)

# --- CONFIGURATION ---
CONNECTION_STRING = "udp://:14551"
BASE_ALTITUDE = 10.0    # The "average" height
WAVE_LENGTH = 40.0      # How long the dragon flies forward (meters)
SWAY_WIDTH = 5.0        # How far left/right it weaves (Amplitude Y)
SWOOP_HEIGHT = 2.0      # How much it dips up/down (Amplitude Z)
WAVE_DENSITY = 0.5      # Distance between points (lower = smoother)
SPEED_DELAY = 0.2       # Lower = Faster dragon

async def run():
    drone = System()
    print(f"-- Connecting to {CONNECTION_STRING}...")
    await drone.connect(system_address=CONNECTION_STRING)

    print("-- Waiting for connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Connected!")
            break

    print("-- Arming & Taking Off")
    await drone.action.arm()
    await drone.action.set_takeoff_altitude(BASE_ALTITUDE)
    await drone.action.takeoff()

    # Wait for altitude
    async for position in drone.telemetry.position():
        if position.relative_altitude_m > BASE_ALTITUDE * 0.95:
            break
    await asyncio.sleep(2)

    # --- DRAGON MODE ---
    print("-- Unleashing the Dragon (Offboard Mode)...")
    
    # 1. Initialize at current spot
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -BASE_ALTITUDE, 0.0))
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"Failed to start offboard: {e}")
        return

    # 2. Calculate the Path
    # We create points from X=0 to X=WAVE_LENGTH
    steps = int(WAVE_LENGTH / WAVE_DENSITY)
    
    # Frequency of the wave (how many wiggles in the distance)
    # 2 * PI is one full wave. We want maybe 2 full waves over the distance.
    frequency = (2 * math.pi) / (WAVE_LENGTH / 2) 

    previous_y = 0 # Used to calculate where to look (Yaw)

    for i in range(steps):
        # Calculate Forward Progress (North)
        x_north = i * WAVE_DENSITY
        
        # Calculate Sway (East/West) using SIN
        y_east = SWAY_WIDTH * math.sin(frequency * x_north)
        
        # Calculate Swoop (Up/Down) using COS (starts high, dips low)
        # Note: 'Down' is negative for UP. 
        # We start at BASE_ALTITUDE and add/subtract the SWOOP_HEIGHT
        z_down = -(BASE_ALTITUDE + (SWOOP_HEIGHT * math.cos(frequency * x_north)))

        # Calculate Yaw (Look where you fly)
        # We look at the difference between current Y and previous Y
        delta_y = y_east - previous_y
        yaw_angle = math.degrees(math.atan2(delta_y, WAVE_DENSITY))
        previous_y = y_east

        # Send command
        print(f" > Dragon Pos: N{x_north:.1f}, E{y_east:.1f}, Alt{-z_down:.1f}m, Yaw{yaw_angle:.0f}")
        await drone.offboard.set_position_ned(
            PositionNedYaw(x_north, y_east, z_down, yaw_angle))
        
        await asyncio.sleep(SPEED_DELAY)

    # --- THE "ROAR" (VICTORY SPIN) ---
    print("-- Dragon Roar (360 Spin) --")
    last_pos = (WAVE_LENGTH, y_east, z_down)
    for yaw in range(0, 361, 20):
        await drone.offboard.set_position_ned(
            PositionNedYaw(last_pos[0], last_pos[1], last_pos[2], float(yaw)))
        await asyncio.sleep(0.1)

    # --- RETURN HOME ---
    print("-- Returning to Lair (Home)...")
    await drone.offboard.set_position_ned(
        PositionNedYaw(0.0, 0.0, -BASE_ALTITUDE, 0.0))
    await asyncio.sleep(5) # Give it time to fly back

    print("-- Landing")
    try:
        await drone.offboard.stop()
    except:
        pass
    await drone.action.land()

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())
import asyncio
import socket
import json
import math
from mavsdk import System

slave_state = {"lat": 0.0, "lon": 0.0, "alt": 0.0, "rel_alt": 0.0}

async def update_slave_telemetry(drone):
    async for pos in drone.telemetry.position():
        slave_state["lat"] = pos.latitude_deg
        slave_state["lon"] = pos.longitude_deg
        slave_state["alt"] = pos.absolute_altitude_m
        slave_state["rel_alt"] = pos.relative_altitude_m

async def run():
    drone = System(port=50052)
    await drone.connect(system_address="udpin://127.0.0.1:14560")

    async for state in drone.core.connection_state():
        if state.is_connected: break

    asyncio.create_task(update_slave_telemetry(drone))

    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok: break

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 5005))
    sock.setblocking(False)
    loop = asyncio.get_running_loop()

    master_connected = False
    while not master_connected:
        try:
            data, addr = await asyncio.wait_for(loop.sock_recvfrom(sock, 1024), timeout=1.0)
            if "rel_alt" in json.loads(data.decode()): master_connected = True
        except asyncio.TimeoutError: continue

    reply_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    reply_sock.sendto(json.dumps({"status": "CONNECTED"}).encode(), ("127.0.0.1", 5004))
    reply_sock.sendto(json.dumps({"status": "READY"}).encode(), ("127.0.0.1", 5004))

    while True:
        try:
            data, addr = await asyncio.wait_for(loop.sock_recvfrom(sock, 1024), timeout=1.0)
            if json.loads(data.decode())["rel_alt"] > 2.0: break
        except asyncio.TimeoutError: pass

    while True:
        try:
            await drone.action.arm()
            break
        except: await asyncio.sleep(1.0)

    await drone.action.set_takeoff_altitude(5.0)
    while True:
        try:
            await drone.action.takeoff()
            break
        except: await asyncio.sleep(1.0)

    while True:
        if slave_state["rel_alt"] > 4.5:
            print("Slave at altitude. Stabilizing for 3s to clear Takeoff state...")
            await asyncio.sleep(3.0)
            print("Unlocking master for forward flight.")
            reply_sock.sendto(json.dumps({"status": "AT_ALTITUDE"}).encode(), ("127.0.0.1", 5004))
            break
        await asyncio.sleep(0.5)

    slave_cruise_alt = slave_state["alt"]
    print(f"Following Master... Locked cruise altitude at {slave_cruise_alt}m")

    last_master_tgt_lat = 0.0
    slave_tgt_lat = 0.0
    slave_tgt_lon = 0.0
    deg_to_m = 111139.0

    DESIRED_FOLLOW_DIST = 2.0
    last_cmd_time = 0.0  # Tracks when we last sent a command to ArduPilot

    while True:
        try:
            # FIX 1: The Buffer Drain Loop. Throw away old packets instantly!
            latest_data = None
            while True:
                try:
                    data, _ = sock.recvfrom(1024)
                    latest_data = data
                except BlockingIOError:
                    break # Buffer is empty, we have the freshest packet
            
            # If the buffer was totally empty, wait normally
            if latest_data is None:
                latest_data, _ = await asyncio.wait_for(loop.sock_recvfrom(sock, 1024), timeout=3.0)
            
            master = json.loads(latest_data.decode())

            if master.get("landing", False):
                print("\nMaster landing signal received. Slave landing immediately.")
                await drone.action.land()
                break

            if slave_state["lat"] == 0.0:
                await asyncio.sleep(0.1)
                continue
            
            master_tgt_lat = master.get("tgt_lat", 0.0)
            master_tgt_lon = master.get("tgt_lon", 0.0)

            if master_tgt_lat == 0.0:
                await asyncio.sleep(0.1)
                continue

            if master_tgt_lat != last_master_tgt_lat:
                yaw_rad = math.radians(master.get("yaw", 0.0))
                
                offset_n = (-DESIRED_FOLLOW_DIST * math.cos(yaw_rad))
                offset_e = (-DESIRED_FOLLOW_DIST * math.sin(yaw_rad))

                slave_tgt_lat = master_tgt_lat + (offset_n / deg_to_m)
                slave_tgt_lon = master_tgt_lon + (offset_e / (deg_to_m * math.cos(math.radians(master_tgt_lat))))

                last_master_tgt_lat = master_tgt_lat
                print("-- Master generated new waypoint. Calculating parallel route.")

            # FIX 2: Periodic Command Dispatcher
            if slave_tgt_lat != 0.0:
                dist_to_tgt_y = (slave_tgt_lat - slave_state["lat"]) * deg_to_m
                dist_to_tgt_x = (slave_tgt_lon - slave_state["lon"]) * deg_to_m * math.cos(math.radians(slave_state["lat"]))
                dist_to_tgt = math.sqrt(dist_to_tgt_x**2 + dist_to_tgt_y**2)

                current_time = loop.time()
                # Pulse the command every 1.5 seconds if we aren't at the destination yet
                if dist_to_tgt > 1.0 and (current_time - last_cmd_time) > 1.5:
                    try:
                        await drone.action.goto_location(slave_tgt_lat, slave_tgt_lon, slave_cruise_alt, master.get("yaw", 0.0))
                        print("-- Command sent to ArduPilot! Executing flight.")
                        last_cmd_time = current_time
                    except Exception:
                        pass 

            await asyncio.sleep(0.1)

        except asyncio.TimeoutError:
            print("\n[FAILSAFE] Master UDP stream lost. Landing immediately.")
            await drone.action.land()
            break

    print("Slave is descending. Waiting for touchdown...")
    while True:
        if slave_state["rel_alt"] < 0.5: break
        await asyncio.sleep(1.0)

    print("Slave touchdown confirmed. Notifying Master...")
    reply_sock.sendto(json.dumps({"status": "LANDED"}).encode(), ("127.0.0.1", 5004))
    print("Ending script.")

if __name__ == "__main__":
    asyncio.run(run())
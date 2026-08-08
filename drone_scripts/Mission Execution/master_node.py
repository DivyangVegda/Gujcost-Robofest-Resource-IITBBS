import asyncio
import socket
import json
from mavsdk import System

master_state = {
    "lat": 0.0,
    "lon": 0.0,
    "alt": 0.0,
    "rel_alt": 0.0,
    "yaw": 0.0,
    "tgt_lat": 0.0,  # Added: Broadcasts destination
    "tgt_lon": 0.0,  # Added: Broadcasts destination
    "landing": False,
}
slave_connected = False
slave_ready = False
slave_at_alt = False
slave_landed = False

async def update_position(drone):
    async for pos in drone.telemetry.position():
        master_state["lat"] = pos.latitude_deg
        master_state["lon"] = pos.longitude_deg
        master_state["alt"] = pos.absolute_altitude_m
        master_state["rel_alt"] = pos.relative_altitude_m

async def update_heading(drone):
    async for heading in drone.telemetry.heading():
        master_state["yaw"] = heading.heading_deg

async def broadcast_udp():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    slave_address = ('127.0.0.1', 5005)
    while True:
        if master_state["lat"] != 0.0:
            sock.sendto(json.dumps(master_state).encode(), slave_address)
        await asyncio.sleep(0.2)

async def listen_for_slave():
    global slave_connected, slave_ready, slave_at_alt, slave_landed
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 5004))
    sock.setblocking(False)
    loop = asyncio.get_running_loop()

    while True:
        try:
            data, addr = await asyncio.wait_for(loop.sock_recvfrom(sock, 1024), timeout=1.0)
            msg = json.loads(data.decode())
            status = msg.get("status")
            if status == "CONNECTED":
                slave_connected = True
            elif status == "READY":
                slave_ready = True
            elif status == "AT_ALTITUDE":
                slave_at_alt = True
            elif status == "LANDED":
                slave_landed = True
        except asyncio.TimeoutError:
            pass

async def run():
    drone = System(port=50051)
    await drone.connect(system_address="udpin://127.0.0.1:14550")

    async for state in drone.core.connection_state():
        if state.is_connected:
            break

    asyncio.create_task(update_position(drone))
    asyncio.create_task(update_heading(drone))
    asyncio.create_task(broadcast_udp())
    asyncio.create_task(listen_for_slave())

    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            break

    print("Waiting for slave connection...")
    while not slave_connected:
        await asyncio.sleep(0.5)

    print("Waiting for slave READY signal...")
    while not slave_ready:
        await asyncio.sleep(0.5)

    print("Arming and taking off...")
    while True:
        try:
            await drone.action.arm()
            break
        except:
            await asyncio.sleep(1.0)

    ground_alt = master_state["alt"]
    await drone.action.set_takeoff_altitude(5.0)

    while True:
        try:
            await drone.action.takeoff()
        except:
            pass
        await asyncio.sleep(2.0)
        if master_state["rel_alt"] > 1.0:
            break

    target_alt = ground_alt + 5.0
    while master_state["rel_alt"] < 4.5:
        await asyncio.sleep(0.5)

    print("Master at altitude. Waiting for slave to finish climbing...")
    while not slave_at_alt:
        await asyncio.sleep(1.0)

    print("Both drones synchronized. Moving 20 meters forward...")
    target_lat = master_state["lat"] + (20 * 0.000009)
    target_lon = master_state["lon"]
    
    # Broadcast the destination so the Slave can fly smoothly parallel
    master_state["tgt_lat"] = target_lat
    master_state["tgt_lon"] = target_lon
    
    await drone.action.goto_location(target_lat, target_lon, target_alt, master_state["yaw"])

    await asyncio.sleep(20)

    print("Mission complete. Sending land signal...")
    master_state["landing"] = True
    await drone.action.land()

    print("Master is descending. Waiting for touchdown...")
    while master_state["rel_alt"] > 0.5:
        await asyncio.sleep(1.0)

    print("Master touchdown confirmed. Waiting for Slave to report safe landing...")
    while not slave_landed:
        await asyncio.sleep(1.0)

    print("Swarm touchdown complete. Ending script.")

if __name__ == "__main__":
    asyncio.run(run())
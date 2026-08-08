import cv2
import numpy as np
import time
import math
import heapq
import os
from datetime import datetime
import asyncio
from mavsdk import System
from mavsdk.offboard import PositionNedYaw, OffboardError

# --- Gazebo Native Transport ---
try:
    from gz.transport13 import Node
    from gz.msgs10.image_pb2 import Image
except ImportError:
    from gz.transport12 import Node
    from gz.msgs9.image_pb2 import Image

# --- Central Config ---
known_mines = [] 
SAFETY_DISTANCE = 0.35 # Meters (Collision Avoidance)
TRAVERSE_ALT = -1.5    # NED Z is negative for 'up'
DESCEND_ALT = -0.5
STEP_DIST = 1.1

# --- Path Planning Config ---
master_path = [(0.0, 0.0)] # Starts at master's origin (fwd, rt)
last_planned_x = 0.0
CHUNK_SIZE = 5.0           # Master plans 5 meters at a time
PATH_SAFETY_RADIUS = 0.8   # Meters to stay away from known mines
GRID_RES = 0.5             # Planner resolution (0.5 meters per grid cell)

# --- Master Drone Config ---
master_state = {
    'port': 14550,       # Custom port
    'grpc_port': 50050,  # Unique backend port
    'drone': None,
    'x': 0.0, 'y': 0.0, 'z': 0.0,
    'wp_index': 1        # Start at 1 to skip the (0.0, 0.0) origin
}

# --- Swarm State Dictionary ---
slaves = {
    1: {'port': 14560, 'grpc_port': 50051, 'cam_topic': '/slave_1/camera/image', 'drone': None, 
        'x': 0.0, 'y': 0.0, 'z': 0.0, 'lat': 0.0, 'lon': 0.0, 'hdg': 0, 
        'start_x': None, 'start_y': None, 'target_x': 0.0, 'target_y': 0.0, 'target_z': TRAVERSE_ALT,
        'state': 0, 'frame': None, 'anomaly': False, 'cx': 320, 'cy': 240, 
        'timer': 0, 'safety_hold': False, 'comms_hold': False, 'offset_fwd': 2.0, 'offset_rt': 0.0, 'finished': False},
        
    2: {'port': 14570, 'grpc_port': 50052, 'cam_topic': '/slave_2/camera/image', 'drone': None, 
        'x': 0.0, 'y': 0.0, 'z': 0.0, 'lat': 0.0, 'lon': 0.0, 'hdg': 0,
        'start_x': None, 'start_y': None, 'target_x': 0.0, 'target_y': 0.0, 'target_z': TRAVERSE_ALT,
        'state': 0, 'frame': None, 'anomaly': False, 'cx': 320, 'cy': 240, 
        'timer': 0, 'safety_hold': False, 'comms_hold': False, 'offset_fwd': 2.0, 'offset_rt': 2.0, 'finished': False},
        
    3: {'port': 14580, 'grpc_port': 50053, 'cam_topic': '/slave_3/camera/image', 'drone': None, 
        'x': 0.0, 'y': 0.0, 'z': 0.0, 'lat': 0.0, 'lon': 0.0, 'hdg': 0,
        'start_x': None, 'start_y': None, 'target_x': 0.0, 'target_y': 0.0, 'target_z': TRAVERSE_ALT,
        'state': 0, 'frame': None, 'anomaly': False, 'cx': 320, 'cy': 240, 
        'timer': 0, 'safety_hold': False, 'comms_hold': False, 'offset_fwd': 2.0, 'offset_rt': -2.0, 'finished': False}
}

# --- Camera & Math Helpers (Synchronous) ---
def gps_distance(lat1, lon1, lat2, lon2):
    if lat1 == 0 or lat2 == 0: return 999.0 
    dx = (lon2 - lon1) * 111320.0 * math.cos(math.radians((lat1 + lat2) / 2))
    dy = (lat2 - lat1) * 111320.0
    return math.hypot(dx, dy)

def is_target_clear(est_fwd, est_rt, my_id, radius=1.5):
    """ Checks local grid to prevent redundant mine mapping """
    for mine in known_mines:
        dist = math.hypot(est_fwd - mine['ui_fwd'], est_rt - mine['ui_rt'])
        if dist < radius: return False 
        
    for other_id, other_s in slaves.items():
        if other_id == my_id: continue 
        if other_s['state'] in [3, 4]: 
            other_fwd = other_s['x'] + other_s['offset_fwd']
            other_rt = other_s['y'] + other_s['offset_rt']
            dist = math.hypot(est_fwd - other_fwd, est_rt - other_rt)
            if dist < radius: return False 
    return True

def make_callback(slave_id):
    def image_callback(msg):
        img_array = np.frombuffer(msg.data, dtype=np.uint8)
        img_array = img_array.reshape((msg.height, msg.width, 3))
        frame = cv2.cvtColor(img_array, cv2.COLOR_RGB2BGR)
        slaves[slave_id]['frame'] = frame.copy()
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([10, 100, 100]), np.array([40, 255, 255]))
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        found = False
        for cnt in contours:
            if cv2.contourArea(cnt) > 80: 
                M = cv2.moments(cnt)
                if M['m00'] != 0:
                    slaves[slave_id]['cx'] = int(M['m10']/M['m00'])
                    slaves[slave_id]['cy'] = int(M['m01']/M['m00'])
                found = True
                cv2.circle(slaves[slave_id]['frame'], (slaves[slave_id]['cx'], slaves[slave_id]['cy']), 10, (0, 0, 255), 2)
                break
        slaves[slave_id]['anomaly'] = found
    return image_callback

def enforce_safety_radius():
    for id in slaves: slaves[id]['safety_hold'] = False 
    pairs = [(1,2), (2,3), (1,3)]
    for a, b in pairs:
        if slaves[a]['lat'] != 0 and slaves[b]['lat'] != 0:
            if gps_distance(slaves[a]['lat'], slaves[a]['lon'], slaves[b]['lat'], slaves[b]['lon']) < SAFETY_DISTANCE:
                if slaves[a]['x'] < slaves[b]['x']: slaves[a]['safety_hold'] = True
                else: slaves[b]['safety_hold'] = True

def enforce_comms_range():
    """ Keeps slaves within a 9-meter leash of the master drone """
    mx = master_state['x']
    my = master_state['y']
    for s_id, s in slaves.items():
        if s['start_x'] is None: continue
        sx = s['x'] + s['offset_fwd']
        sy = s['y'] + s['offset_rt']
        dist = math.hypot(sx - mx, sy - my)
        if dist > 9.0: s['comms_hold'] = True
        else: s['comms_hold'] = False

def is_point_safe(fwd, rt):
    for mine in known_mines:
        dist = math.hypot(fwd - mine['ui_fwd'], rt - mine['ui_rt'])
        if dist < PATH_SAFETY_RADIUS:
            return False
    return True

def plan_path_chunk(start_pos, target_x, lateral_min_rt, lateral_max_rt):
    start_grid = (round(start_pos[0]/GRID_RES)*GRID_RES, round(start_pos[1]/GRID_RES)*GRID_RES)
    
    queue = [(0, 0, start_grid, [start_grid])]
    visited = set()
    
    while queue:
        _, cost, current, path = heapq.heappop(queue)
        
        if current[0] >= target_x:
            return path
            
        if current in visited: continue
        visited.add(current)
        
        for df, dr in [(GRID_RES,0), (0,GRID_RES), (0,-GRID_RES), (GRID_RES,GRID_RES), (GRID_RES,-GRID_RES), (-GRID_RES,0), (-GRID_RES,GRID_RES), (-GRID_RES,-GRID_RES)]:
            nxt = (current[0] + df, current[1] + dr)
            
            if nxt[1] < lateral_min_rt or nxt[1] > lateral_max_rt:
                continue
                
            if nxt not in visited and is_point_safe(nxt[0], nxt[1]):
                new_cost = cost + math.hypot(df, dr)
                heuristic = target_x - nxt[0] 
                heapq.heappush(queue, (new_cost + heuristic, new_cost, nxt, path + [nxt]))
                
    return []

# --- Async MAVSDK Tasks ---

async def update_telemetry(s_id):
    drone = slaves[s_id]['drone']
    
    async def get_local():
        async for pos in drone.telemetry.position_velocity_ned():
            slaves[s_id]['x'] = pos.position.north_m
            slaves[s_id]['y'] = pos.position.east_m
            slaves[s_id]['z'] = pos.position.down_m
            if slaves[s_id]['start_x'] is None:
                slaves[s_id]['start_x'] = pos.position.north_m
                slaves[s_id]['start_y'] = pos.position.east_m
                slaves[s_id]['target_y'] = pos.position.east_m

    async def get_global():
        async for pos in drone.telemetry.position():
            slaves[s_id]['lat'] = pos.latitude_deg
            slaves[s_id]['lon'] = pos.longitude_deg
            
    async def get_heading():
        async for hdg in drone.telemetry.heading():
            slaves[s_id]['hdg'] = hdg.heading_deg

    await asyncio.gather(get_local(), get_global(), get_heading())

async def run_slave(s_id):
    s = slaves[s_id]
    drone = System(port=s['grpc_port'])
    s['drone'] = drone
    
    print(f"Connecting Slave {s_id} on udpin://127.0.0.1:{s['port']}...")
    await drone.connect(system_address=f"udpin://127.0.0.1:{s['port']}")
    
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Slave {s_id} Connected!")
            break

    asyncio.create_task(update_telemetry(s_id))

    print(f"Slave {s_id} arming and taking off...")
    await drone.action.arm()
    await drone.action.set_takeoff_altitude(abs(TRAVERSE_ALT))
    await drone.action.takeoff()
    await asyncio.sleep(8) 

    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, TRAVERSE_ALT, 0.0))
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Slave {s_id} Offboard start failed: {error}")
        return

    S_CALC = 0; S_MOVE = 1; S_SCAN = 2; S_CENTER = 3; S_DESCEND = 4; S_ASCEND = 5

    while True:
        if s['start_x'] is None:
            await asyncio.sleep(0.1)
            continue
            
        if (s['x'] - s['start_x'] >= 20.0 or s['finished']) and s['state'] in [S_CALC, S_MOVE, S_SCAN]:
            if not s['finished']:
                print(f"Slave {s_id} reached boundary. Aligning to finish line.")
                s['finished'] = True
            s['target_x'] = s['start_x'] + 20.0
            s['target_z'] = TRAVERSE_ALT
        
        elif s['safety_hold'] or s['comms_hold']:
            # Hold current position for collision or tether
            s['target_x'] = s['x']
            s['target_y'] = s['y']
            s['target_z'] = s['z']
            
        else:
            if s['state'] == S_CALC:
                s['target_x'] = s['x'] + STEP_DIST 
                s['target_z'] = TRAVERSE_ALT
                s['state'] = S_MOVE

            elif s['state'] == S_MOVE:
                if abs(s['x'] - s['target_x']) < 0.20 and abs(s['z'] - TRAVERSE_ALT) < 0.20:
                    s['timer'] = time.time()
                    s['state'] = S_SCAN

            elif s['state'] == S_SCAN:
                if time.time() - s['timer'] > 2.0:
                    if s['anomaly']:
                        fwd_m = (240 - s['cy']) * 0.005 
                        rt_m  = (s['cx'] - 320) * 0.005
                        
                        est_fwd = s['x'] + s['offset_fwd'] + fwd_m
                        est_rt = s['y'] + s['offset_rt'] + rt_m
                        
                        if is_target_clear(est_fwd, est_rt, s_id, radius=1.5):
                            print(f"Slave {s_id} claimed new anomaly. Centering...")
                            s['state'] = S_CENTER
                        else:
                            s['state'] = S_CALC
                    else:
                        s['state'] = S_CALC

            elif s['state'] == S_CENTER:
                err_x = s['cx'] - 320
                err_y = 240 - s['cy'] 
                if abs(err_x) < 20 and abs(err_y) < 20:
                    s['target_x'] = s['x'] 
                    s['target_y'] = s['y']
                    s['target_z'] = DESCEND_ALT
                    s['state'] = S_DESCEND
                else:
                    s['target_x'] = s['x'] + (err_y * 0.001) 
                    s['target_y'] = s['y'] + (err_x * 0.001)

            elif s['state'] == S_DESCEND:
                if abs(s['z'] - DESCEND_ALT) < 0.15:
                    if s['timer'] == 0: s['timer'] = time.time()
                    elif time.time() - s['timer'] > 4.0:
                        known_mines.append({
                            'lat': s['lat'], 'lon': s['lon'],
                            'ui_fwd': s['x'] + s['offset_fwd'], 'ui_rt': s['y'] + s['offset_rt']
                        })
                        s['timer'] = 0 
                        s['target_z'] = TRAVERSE_ALT
                        s['state'] = S_ASCEND

            elif s['state'] == S_ASCEND:
                if abs(s['z'] - TRAVERSE_ALT) < 0.20:
                    s['target_y'] = s['start_y'] 
                    s['state'] = S_CALC

        await drone.offboard.set_position_ned(PositionNedYaw(s['target_x'], s['target_y'], s['target_z'], 0.0))
        await asyncio.sleep(0.05) 

async def run_master():
    """ State machine for the Master drone to follow the path """
    drone = System(port=master_state['grpc_port'])
    master_state['drone'] = drone
    
    print(f"Connecting Master on udpin://127.0.0.1:{master_state['port']}...")
    await drone.connect(system_address=f"udpin://127.0.0.1:{master_state['port']}")
    
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Master Connected!")
            break

    async def update_master_telemetry():
        async for pos in drone.telemetry.position_velocity_ned():
            master_state['x'] = pos.position.north_m
            master_state['y'] = pos.position.east_m
            master_state['z'] = pos.position.down_m
            
    asyncio.create_task(update_master_telemetry())

    print("Master arming and taking off...")
    await drone.action.arm()
    await drone.action.set_takeoff_altitude(abs(TRAVERSE_ALT))
    await drone.action.takeoff()
    await asyncio.sleep(8) 

    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, TRAVERSE_ALT, 0.0))
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Master Offboard start failed: {error}")
        return

    while True:
        if master_state['wp_index'] < len(master_path):
            target_fwd, target_rt = master_path[master_state['wp_index']]
            await drone.offboard.set_position_ned(PositionNedYaw(target_fwd, target_rt, TRAVERSE_ALT, 0.0))
            
            dist = math.hypot(master_state['x'] - target_fwd, master_state['y'] - target_rt)
            if dist < 0.3:
                master_state['wp_index'] += 1
        else:
            if len(master_path) > 0:
                target_fwd, target_rt = master_path[-1]
                await drone.offboard.set_position_ned(PositionNedYaw(target_fwd, target_rt, TRAVERSE_ALT, 0.0))

        await asyncio.sleep(0.05)

async def master_ui_loop():
    global last_planned_x, master_path
    
    while True:
        enforce_safety_radius()
        enforce_comms_range()
        
        # --- PATH PLANNING TRIGGER ---
        active_x_positions = [s['x'] + s['offset_fwd'] for s in slaves.values() if s['start_x'] is not None]
        if active_x_positions:
            min_x_progress = min(active_x_positions)
            
            if min_x_progress > (last_planned_x + CHUNK_SIZE):
                next_boundary = last_planned_x + CHUNK_SIZE
                print(f"Master: Planning safe route up to {next_boundary}m...")

                scanned_rts = [s['offset_rt'] for s in slaves.values() if s['start_x'] is not None]
                if scanned_rts:
                    corridor_min_rt = min(scanned_rts)
                    corridor_max_rt = max(scanned_rts)
                    
                    new_segment = plan_path_chunk(master_path[-1], next_boundary, corridor_min_rt, corridor_max_rt)
                    if new_segment:
                        master_path.extend(new_segment[1:])
                        last_planned_x = next_boundary
                    else:
                        print("Master WARNING: No safe path found through this chunk!")

        # --- UI DRAWING ---
        map_img = np.ones((400, 800, 3), dtype=np.uint8) * 40 
        for i in range(0, 800, 40): cv2.line(map_img, (i, 0), (i, 400), (80, 80, 80), 1)
        for i in range(0, 400, 40): cv2.line(map_img, (0, i), (800, i), (80, 80, 80), 1)
        
        def to_pixels(fwd, rt): 
            return int(50 + fwd * 30), int(200 + rt * 30)

        # Draw the safe path
        if len(master_path) > 1:
            for i in range(len(master_path) - 1):
                pt1 = to_pixels(*master_path[i])
                pt2 = to_pixels(*master_path[i+1])
                cv2.line(map_img, pt1, pt2, (255, 255, 0), 2)
                cv2.circle(map_img, pt2, 3, (255, 255, 0), -1)

        # Draw Master Drone & Comms Tether
        mx, my = to_pixels(master_state['x'], master_state['y'])
        
        # Faint circle to show the 10-meter comms leash
        cv2.circle(map_img, (mx, my), 300, (60, 60, 60), 1, lineType=cv2.LINE_AA)
        
        cv2.circle(map_img, (mx, my), 8, (255, 255, 255), -1)
        cv2.putText(map_img, "MASTER", (mx - 20, my - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        for mine in known_mines:
            sx, sy = to_pixels(mine['ui_fwd'], mine['ui_rt'])
            cv2.circle(map_img, (sx, sy), 8, (0, 0, 255), -1)   
            buffer_px = int(PATH_SAFETY_RADIUS * 30)
            cv2.circle(map_img, (sx, sy), buffer_px, (0, 100, 255), 1) 

        colors = {1: (255, 100, 0), 2: (0, 255, 0), 3: (255, 0, 255)} 
        for s_id, s in slaves.items():
            if s['start_x'] is not None:
                dx, dy = to_pixels(s['x'] + s['offset_fwd'], s['y'] + s['offset_rt'])
                
                # Make the drone flash white if it is holding for comms
                drone_color = (255, 255, 255) if s['comms_hold'] and int(time.time() * 4) % 2 == 0 else colors[s_id]
                cv2.circle(map_img, (dx, dy), 6, drone_color, -1)
                cv2.putText(map_img, f"S{s_id}", (dx - 10, dy - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, drone_color, 2)
                
            if s['frame'] is not None:
                cv2.imshow(f"Slave {s_id}", s['frame'])

        cv2.putText(map_img, "MAVSDK Swarm Control Map", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.imshow("Master Control Map", map_img)
        cv2.waitKey(1)

        # --- MISSION COMPLETION LOGIC ---
        slaves_done = all(s['finished'] for s in slaves.values())
        master_done = (master_state['wp_index'] >= len(master_path)) and (master_state['x'] >= 18.0)

        if slaves_done and master_done:
            print("\n*** SWARM MISSION COMPLETE ***")
            print("Master has safely exited the minefield. Landing all drones...")
            
            if master_state['drone']:
                await master_state['drone'].action.land()
            for s_id, s in slaves.items():
                if s['drone']:
                    await s['drone'].action.land()
            
            time_str = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
            filename = f"Master_Control_Map_{time_str}.png"
            
            print(f"Saving final mission map as: {filename}")
            if cv2.imwrite(filename, map_img): 
                print("Map successfully saved!")
            
            print("Shutting down MAVSDK Swarm Controller and closing all windows...")
            cv2.destroyAllWindows()
            os._exit(0)
            
        await asyncio.sleep(0.05)

async def main():
    node = Node()
    for s_id in slaves:
        if not node.subscribe(Image, slaves[s_id]['cam_topic'], make_callback(s_id)):
            print(f"Failed to subscribe to Slave {s_id} camera.")

    await asyncio.gather(
        run_slave(1),
        run_slave(2),
        run_slave(3),
        run_master(),
        master_ui_loop()
    )

if __name__ == '__main__':
    asyncio.run(main())
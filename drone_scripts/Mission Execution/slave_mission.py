import cv2
import numpy as np
import time
import math
from pymavlink import mavutil

# --- Gazebo Native Transport ---
try:
    from gz.transport13 import Node
    from gz.msgs10.image_pb2 import Image
except ImportError:
    from gz.transport12 import Node
    from gz.msgs9.image_pb2 import Image

# --- Shared Global Variables ---
latest_frame = None
anomaly_detected = False
target_cx = 320
target_cy = 240

# Memory Map and Position Variables 
known_mines = []
current_x = 0.0
current_y = 0.0

def is_new_mine(x, y, radius=1.5):
    """ Checks if the coordinates are far enough away from previously logged mines """
    global known_mines
    for mx, my in known_mines:
        distance = math.hypot(x - mx, y - my)
        if distance < radius:
            return False 
    return True

def image_callback(msg):
    global latest_frame, anomaly_detected, target_cx, target_cy, current_x, current_y
    
    img_array = np.frombuffer(msg.data, dtype=np.uint8)
    img_array = img_array.reshape((msg.height, msg.width, 3))
    frame = cv2.cvtColor(img_array, cv2.COLOR_RGB2BGR)
    latest_frame = frame.copy()
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_orange = np.array([10, 100, 100])
    upper_orange = np.array([40, 255, 255])
    
    mask = cv2.inRange(hsv, lower_orange, upper_orange)
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    found = False
    for cnt in contours:
        if cv2.contourArea(cnt) > 100: 
            M = cv2.moments(cnt)
            if M['m00'] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                
                # --- SPATIAL FILTERING LOGIC ---
                # Estimate the physical location of this contour based on drone's altitude
                # At 5m altitude, 1 pixel is roughly 0.012 meters.
                est_target_x = current_x + (240 - cy) * 0.012 # Forward/Back
                est_target_y = current_y + (cx - 320) * 0.012 # Left/Right
                
                # If this pixel matches a mine we already found, IGNORE IT and check next contour
                if not is_new_mine(est_target_x, est_target_y, radius=1.5):
                    # Draw a gray box to show it's being ignored
                    cv2.putText(latest_frame, "IGNORED", (cx-30, cy-20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 100, 100), 1)
                    continue 
                
                # If we made it here, it's a completely NEW mine!
                target_cx = cx
                target_cy = cy
                found = True
                
                # Draw red targeting reticle
                cv2.circle(latest_frame, (target_cx, target_cy), 10, (0, 0, 255), 2)
                cv2.line(latest_frame, (target_cx - 15, target_cy), (target_cx + 15, target_cy), (0, 0, 255), 2)
                cv2.line(latest_frame, (target_cx, target_cy - 15), (target_cx, target_cy + 15), (0, 0, 255), 2)
                break
            
    anomaly_detected = found

def goto_position(vehicle, x, y, z):
    vehicle.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
        10, vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b110111111000, 
        x, y, z, 0, 0, 0, 0, 0, 0, 0, 0))

def draw_grid_map(start_x, start_y):
    global current_x, current_y, known_mines
    map_img = np.ones((400, 800, 3), dtype=np.uint8) * 40 
    
    for i in range(0, 800, 40):
        cv2.line(map_img, (i, 0), (i, 400), (80, 80, 80), 1)
    for i in range(0, 400, 40):
        cv2.line(map_img, (0, i), (800, i), (80, 80, 80), 1)

    if start_x is None or start_y is None:
        cv2.imshow("Live Mine Map", map_img)
        return

    def to_pixels(px, py):
        screen_x = int(50 + (px - start_x) * 40)
        screen_y = int(200 + (py - start_y) * 40)
        return screen_x, screen_y

    for mx, my in known_mines:
        sx, sy = to_pixels(mx, my)
        cv2.circle(map_img, (sx, sy), 8, (0, 0, 255), -1)   
        cv2.circle(map_img, (sx, sy), 14, (0, 255, 255), 2) 
        cv2.putText(map_img, f"X:{mx-start_x:.1f}", (sx - 15, sy + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

    dx, dy = to_pixels(current_x, current_y)
    cv2.circle(map_img, (dx, dy), 6, (255, 100, 0), -1)
    cv2.putText(map_img, "UAV", (dx - 12, dy - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    cv2.putText(map_img, "20m x 5m Search Grid", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(map_img, f"Mines Validated: {len(known_mines)}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    cv2.imshow("Live Mine Map", map_img)

def main():
    global target_cx, target_cy, current_x, current_y, known_mines
    
    node = Node()
    if not node.subscribe(Image, "/slave/camera/image", image_callback): return

    print("Connecting to Slave Drone...")
    slave = mavutil.mavlink_connection('udpin:127.0.0.1:14560')
    slave.wait_heartbeat()
    print("Connected!")

    mode_id = slave.mode_mapping()['GUIDED']
    slave.mav.set_mode_send(slave.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)
    time.sleep(1)
    slave.mav.command_long_send(slave.target_system, slave.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    slave.mav.command_long_send(slave.target_system, slave.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 5.0)
    
    print("Taking off. Waiting 10 seconds...")
    time.sleep(10)

    STATE_CALCULATE_NEXT = 0
    STATE_MOVING = 1
    STATE_SCANNING = 2
    STATE_CENTERING = 3   
    STATE_DESCENDING = 4
    STATE_ASCENDING = 5

    current_state = STATE_CALCULATE_NEXT
    
    current_z = 0.0
    start_x = None
    target_x = 0.0
    target_y = 0.0
    scan_timer = 0
    validation_timer = 0
    position_received = False
    
    print("\n--- INITIATING PRECISION SURVEY ---")

    while True:
        msg = slave.recv_match(type='LOCAL_POSITION_NED', blocking=False)
        if msg:
            current_x, current_y, current_z = msg.x, msg.y, msg.z
            position_received = True
            
            if start_x is None:
                start_x = current_x
                target_y = current_y
                
            distance_traveled = current_x - start_x

            if distance_traveled >= 20.0 and current_state in [STATE_CALCULATE_NEXT, STATE_MOVING, STATE_SCANNING]:
                print(f"\nReached 20m geofence. Survey complete.")
                break

        if not position_received:
            time.sleep(0.05)
            continue

        if latest_frame is not None:
            cv2.line(latest_frame, (320, 230), (320, 250), (255, 255, 255), 1)
            cv2.line(latest_frame, (310, 240), (330, 240), (255, 255, 255), 1)
            
            status_text = ["CALCULATING", "MOVING +1.5m", "SCANNING", "CENTERING ON TARGET", "VALIDATING (2m)", "ASCENDING (5m)"][current_state]
            display_frame = latest_frame.copy()
            cv2.putText(display_frame, f"STATE: {status_text}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.imshow("Drone Feed", display_frame)
            
            draw_grid_map(start_x, target_y)
            if cv2.waitKey(1) & 0xFF == ord('q'): break

        # --- LOGIC FLOW ---
        if current_state == STATE_CALCULATE_NEXT:
            target_x = current_x + 1.5
            print(f"\nStepping forward to X: {target_x:.2f}")
            goto_position(slave, target_x, target_y, -5.0)
            current_state = STATE_MOVING

        elif current_state == STATE_MOVING:
            goto_position(slave, target_x, target_y, -5.0) 
            if abs(current_x - target_x) < 0.20 and abs(current_z - (-5.0)) < 0.20:
                scan_timer = time.time()
                current_state = STATE_SCANNING

        elif current_state == STATE_SCANNING:
            goto_position(slave, target_x, target_y, -5.0) 
            if time.time() - scan_timer > 2.0:
                # Removed is_new_mine check here, because it is now handled directly by the camera logic!
                if anomaly_detected:
                    print("*** NEW ANOMALY FOUND *** Initiating Visual Servoing...")
                    current_state = STATE_CENTERING
                else:
                    current_state = STATE_CALCULATE_NEXT

        elif current_state == STATE_CENTERING:
            err_x_pixels = target_cx - 320
            err_y_pixels = 240 - target_cy 
            
            if abs(err_x_pixels) < 20 and abs(err_y_pixels) < 20:
                print("Target Centered! Descending for final validation.")
                target_x = current_x 
                target_y = current_y
                current_state = STATE_DESCENDING
            else:
                dx = err_y_pixels * 0.002 
                dy = err_x_pixels * 0.002 
                target_x = current_x + dx
                target_y = current_y + dy
                goto_position(slave, target_x, target_y, -5.0)

        elif current_state == STATE_DESCENDING:
            goto_position(slave, target_x, target_y, -2.0) 
            if abs(current_z - (-2.0)) < 0.20:
                if validation_timer == 0:
                    print("Axis Locked at 2m. Validating Thermal Inertia (5s)...")
                    validation_timer = time.time()
                
                if time.time() - validation_timer > 5.0:
                    print("Validation Complete! Logging accurate 2m coordinate and ascending.")
                    known_mines.append((current_x, current_y)) 
                    validation_timer = 0 
                    current_state = STATE_ASCENDING

        elif current_state == STATE_ASCENDING:
            goto_position(slave, target_x, target_y, -5.0) 
            if abs(current_z - (-5.0)) < 0.20:
                target_y = start_x # Lock back into grid
                current_state = STATE_CALCULATE_NEXT

        time.sleep(0.05) 

    # --- RTL ---
    print("\n--- TRIGGERING RTL ---")
    rtl_mode = slave.mode_mapping()['RTL']
    slave.mav.set_mode_send(slave.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, rtl_mode)
    print("Map Complete. Returning to Launch.")
    
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
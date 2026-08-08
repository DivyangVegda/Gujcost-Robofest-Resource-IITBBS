import cv2
import numpy as np
from pymavlink import mavutil
import time

# 1. Connect to the Slave Drone
# Using the port you specified for the slave instance
slave_conn = mavutil.mavlink_connection('udpin:127.0.0.1:14561') 
slave_conn.wait_heartbeat()
print("Connected to Slave Drone (ID 2)")

def set_mode(mode):
    mode_id = slave_conn.mode_mapping()[mode]
    slave_conn.mav.set_mode_send(
        slave_conn.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)

def get_video_stream():
    # This captures the virtual window from Gazebo Image Display or a GStreamer pipeline
    # For simulation, we often use a GStreamer bridge or screen capture of the Gazebo Topic
    cap = cv2.VideoCapture(0) # Adjust index if using a virtual video device
    return cap

def mission_logic():
    cap = get_video_stream()
    
    # Mission State
    found_mine = False
    
    print("Starting Mine Search Pattern...")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Convert to HSV to find the Orange Mines
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_orange = np.array([5, 150, 150])
        upper_orange = np.array([15, 255, 255])
        
        mask = cv2.inRange(hsv, lower_orange, upper_orange)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 500:
                # Get center of the mine in pixels
                M = cv2.moments(cnt)
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                
                print(f"Mine Detected at Pixel: {cx}, {cy}")
                
                # If mine is roughly centered, descend to 'Validate' (Thermal Inertia test)
                if 300 < cx < 340 and not found_mine:
                    print("Centering complete. Descending for thermal validation...")
                    # Command: Go to 2 meters altitude
                    slave_conn.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                        10, slave_conn.target_system, slave_conn.target_component,
                        mavutil.mavlink.MAV_FRAME_BODY_NED, 0b110111111000, 
                        0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0))
                    found_mine = True

        cv2.imshow('Slave Camera Feed', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    mission_logic()
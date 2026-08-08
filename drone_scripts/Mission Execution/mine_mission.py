import cv2
import numpy as np
from pymavlink import mavutil
import time

# 1. Connect to Slave Instance
print("Connecting to Slave Drone...")
slave = mavutil.mavlink_connection('udpin:127.0.0.1:14560')
slave.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (slave.target_system, slave.target_component))

def descend_to_target():
    print("Anomaly Detected! Descending for validation...")
    # Change altitude to 2 meters relative to ground
    slave.mav.command_long_send(
        slave.target_system, slave.target_component,
        mavutil.mavlink.MAV_CMD_CONDITION_CHANGE_ALT, 0,
        1, # Descent rate
        2, # Target altitude (meters)
        0, 0, 0, 0, 0)

def main():
    # Capture the video feed (Assuming you route Gazebo to /dev/video0)
    # Note: For testing without routing, you can use Gazebo's "Image Display" plugin visually.
    cap = cv2.VideoCapture(0) 
    
    validated = False

    while True:
        ret, frame = cap.read()
        if not ret:
            continue
            
        # Convert BGR to HSV for color detection
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Define the exact orange color of the mines in the SDF
        lower_orange = np.array([5, 150, 150])
        upper_orange = np.array([15, 255, 255])
        
        # Threshold the image
        mask = cv2.inRange(hsv, lower_orange, upper_orange)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            # If the object is large enough in the camera frame
            if area > 1000 and not validated:
                M = cv2.moments(cnt)
                if M['m00'] != 0:
                    cx = int(M['m10']/M['m00'])
                    
                    # If target is roughly in the center of the camera
                    if 250 < cx < 390:
                        descend_to_target()
                        validated = True
                        time.sleep(5) # Wait during validation

        cv2.imshow('Slave RGB Feed', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
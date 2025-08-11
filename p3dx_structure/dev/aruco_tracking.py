import cv2
import numpy as np
import socket
import json
import time
from socketAPI.myRobotController import SimpleRobotController


if __name__ == "__main__":
    robot = SimpleRobotController(is_simulation=False)
    
    # --- Camera setup ---
    cap = cv2.VideoCapture(0) # Use 0 for default camera
    
    # --- ArUco setup ---
    # Define the ArUco dictionary you are using
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

    # --- P-Controller constants (tune these for your specific robot) ---
    kp_linear = 0.01 # Proportional gain for linear velocity
    kp_angular = 0.005 # Proportional gain for angular velocity
    speed_scale = 0.1
    # --- Main loop ---
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
                
            # Convert to grayscale for detection
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Detect markers
            corners, ids, rejectedImgPoints = detector.detectMarkers(gray)
            
            linear_speed = 0.0
            angular_speed = 0.0

            if ids is not None:
                # Assuming you are looking for a specific marker, e.g., ID 42
                desired_id = 5
                if desired_id in ids:
                    marker_index = np.where(ids == desired_id)[0][0]
                    marker_corners = corners[marker_index][0]
                    
                    # Calculate center of the marker
                    marker_center_x = int(np.mean(marker_corners[:, 0]))
                    marker_center_y = int(np.mean(marker_corners[:, 1]))
                    
                    # Calculate image center
                    image_center_x = frame.shape[1] // 2
                    image_center_y = frame.shape[0] // 2
                    
                    # Calculate errors
                    error_x = image_center_x - marker_center_x # For turning
                    error_y = image_center_y - marker_center_y # For moving forward
                    
                    # Apply P-controller to determine speeds
                    angular_speed = (kp_angular * error_x)*speed_scale
                    linear_speed = (kp_linear * error_y)*speed_scale
                    
                    # Send commands to the robot
                    robot.set_velocity(linear_speed, angular_speed)
                    
                    # Optional: Draw on the frame for visualization
                    cv2.aruco.drawDetectedMarkers(frame, corners)
                    cv2.circle(frame, (marker_center_x, marker_center_y), 5, (0, 255, 0), -1)
                    cv2.circle(frame, (image_center_x, image_center_y), 5, (0, 0, 255), -1)
            else:
                # Stop the robot if no marker is detected
                robot.stop()

            # Display the result
            cv2.imshow('Robot Vision', frame)
            
            # Break loop on 'q' key press
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    except KeyboardInterrupt:
        print("Stopping robot and closing connections...")
    finally:
        robot.close()
        cap.release()
        cv2.destroyAllWindows()

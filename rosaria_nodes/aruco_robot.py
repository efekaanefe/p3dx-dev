#!/usr/bin/env python3

"""
1. Bu script, robotun aruco marker takip edebilmesi için kullanılır.
2. Robota bağlı olan bilgisardaki catkin_ws'in içinde kullanın.
3. 4*4'lük aruco markerlardan 1 id numaralı markeri kullanın.
"""

import cv2
import cv2.aruco as aruco
import time
import numpy as np
import rospy
from geometry_msgs.msg import Twist

# Constants
CAM_RESOLUTION_WIDTH  = 640
CAM_RESOLUTION_HEIGHT = 480

MARKER_ID_TO_TRACK = 1

MARKER_TIMEOUT = 0.2      # seconds to retain marker display after brief loss

BACK_UP_THRESHOLD = 180   # min number of pixels to back up
MOVE_THRESHOLD = 140      # max number of pixels to move

MAX_LINEAR_X = 2
MAX_ANGULAR_Z = 2

K_LINEAR = 0.005
K_ANGULAR = 0.005

class ArucoDetector():
    def __init__(self, simulation=True):
        dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(dictionary, parameters)
        rospy.loginfo("Tracking ArUco marker ID: %d", MARKER_ID_TO_TRACK)
    
        # Initialize ROS node
        rospy.init_node('aruco_node', anonymous=True)

        # Create publisher based on mode
        if simulation:
            self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        else:
            self.pub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=10)
        
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            rospy.logerr("Error: Cannot open camera.")
            return

        # Set camera resolution
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_RESOLUTION_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_RESOLUTION_HEIGHT)

        # Buffer to store last seen info for marker ID 1
        self.last_seen_corners = None
        self.last_seen_time = 0
        self.corners = None

        # Control variables
        self.current_linear_x = 0
        self.current_angular_z = 0
        self.linear_vel = 0
        self.angle_to_target = 0

        # Target position
        self.target_middle_x = 0
        self.target_middle_y = 0

        self.gray = None
        self.frame = None

    def draw_middle_point(self):
        self.corners = self.last_seen_corners.reshape(4, 2)
        #   print(self.corners)
            
        # Reset coordinates before calculation
        # x,y = 0,0 -> left upper corner
        # x,y = 640,480 -> right bottom corner
        self.target_middle_x = 0 # width
        self.target_middle_y = 0 # height
        
        for i in range(4):
            self.target_middle_x += self.corners[i][0]
        self.target_middle_x = int(self.target_middle_x / 4)

        for i in range(4):
            self.target_middle_y += self.corners[i][1]
        self.target_middle_y = int(self.target_middle_y / 4)

        # Draw circle on the frame (not grayscale image)
        cv2.circle(self.frame, (self.target_middle_x, self.target_middle_y), 6, (0, 0, 255), 10)
        #print(self.target_middle_x, self.target_middle_y)

        
    def check_location(self):
        if self.last_seen_corners is None:
            return
    
        else:
            top_edge = np.linalg.norm(self.corners[0] - self.corners[1])
            right_edge = np.linalg.norm(self.corners[1] - self.corners[2])
            #print(top_edge, right_edge)

            closeness_to_target = round((top_edge + right_edge) / 2)
            
            if closeness_to_target > BACK_UP_THRESHOLD:
                rospy.loginfo("Too close, backing up..")
            elif MOVE_THRESHOLD < closeness_to_target < BACK_UP_THRESHOLD:
                self.linear_vel = 0
                rospy.loginfo("Staying here..")
                return
            else:
                rospy.loginfo("Moving to target...")

            self.linear_vel = round(K_LINEAR * ((BACK_UP_THRESHOLD + MOVE_THRESHOLD) / 2 - closeness_to_target))
            
    def find_angle(self):
        self.angle_to_target = -round(K_ANGULAR * (self.target_middle_x - CAM_RESOLUTION_WIDTH / 2))
        print(self.angle_to_target)

    def create_and_send_twist(self):

        twist = Twist()

        twist.linear.x  = min(self.linear_vel, MAX_LINEAR_X)
        twist.angular.z = min(self.angle_to_target, MAX_ANGULAR_Z)

        # Publish to output topic
        self.pub.publish(twist)
        

    def detect(self):
        ret, self.frame = self.cap.read()
        if not ret:
            rospy.logwarn("Camera frame not received.")
            return

        self.gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        current_time = time.time()

        # Detect markers
        corners, ids, _ = self.detector.detectMarkers(self.gray)

        found_marker = False

        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                if marker_id == MARKER_ID_TO_TRACK:
                    self.last_seen_corners = corners[i]
                    self.last_seen_time = current_time
                    found_marker = True
                    break  # Only track one marker

        if self.last_seen_corners is not None and current_time - self.last_seen_time <= MARKER_TIMEOUT:
            aruco.drawDetectedMarkers(self.frame, [self.last_seen_corners], ids=np.array([[MARKER_ID_TO_TRACK]]))
            self.draw_middle_point() # draw middle point of the aruco marker and find the corners
            self.check_location()    # based on the edge length, find the distance of the aruco marker and linear velocity
            self.find_angle()        # based on the width value of the center point, find the angle and angular velocity
            self.create_and_send_twist() # create twist message and send

        cv2.imshow("Tracking ArUco Marker ID 1", self.frame)

def main():
    simulation = True
    detector = ArucoDetector(simulation=simulation)

    rate = rospy.Rate(30)  # 30 Hz
    while not rospy.is_shutdown():        
        detector.detect()

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        rate.sleep()

    detector.cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

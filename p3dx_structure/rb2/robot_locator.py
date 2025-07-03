import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import cv2
import numpy as np

# === Camera Intrinsics ===
camera_matrix = np.array([
    [706.27551722,   0.0,         340.16251222],
    [0.0,           702.67322938, 160.47309905],
    [0.0,             0.0,           1.0]
])

dist_coeffs = np.array([
    [-0.517312238, 0.350424110, 0.00521738158, 0.0000174925985, -0.146103916]
])

# === Marker Setup ===
MARKER_SIZE = 0.12           # one side of the square marker in meters

ceiling_height = 3.4         # height of the ceiling in meters -> ölçüldü
camera_height = 1            # height of the camera in meters -> ölçülmedi 
Z_MARKER = ceiling_height - camera_height  # height of the markers in meters
DIST_BETWEEN_MARKERS = 2.5  # distance between markers in meters

KNOWN_MARKERS = {
    0: {"position": np.array([0.0, 0.0, Z_MARKER]), "rotation": np.eye(3)},
    1: {"position": np.array([DIST_BETWEEN_MARKERS, 0.0, Z_MARKER]), "rotation": np.eye(3)},
    2: {"position": np.array([2*DIST_BETWEEN_MARKERS, 0.0, Z_MARKER]), "rotation": np.eye(3)},
    3: {"position": np.array([DIST_BETWEEN_MARKERS/2, 0.0, Z_MARKER]), "rotation": np.eye(3)},
    4: {"position": np.array([3*DIST_BETWEEN_MARKERS/2, -0.2, Z_MARKER]), "rotation": np.eye(3)},
}

def compute_robot_pose_fixed_height(marker_rvec, marker_tvec, marker_id):
    if marker_id not in KNOWN_MARKERS:
        return None, None

    R_cm, _ = cv2.Rodrigues(marker_rvec)
    t_cm = marker_tvec.reshape(3)

    marker_pose = KNOWN_MARKERS[marker_id]
    t_wm = marker_pose["position"]
    R_wm = marker_pose["rotation"]

    t_wc = t_wm - R_wm @ t_cm
    t_wc[2] = 0.0

    yaw_rad = np.arctan2(R_cm[1, 0], R_cm[0, 0])

    return t_wc, yaw_rad

class ArucoPosePublisher(Node):
    def __init__(self):
        super().__init__('aruco_pose_publisher')
        self.publisher_ = self.create_publisher(Vector3, 'robot_pose', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

        self.cap = cv2.VideoCapture(2)
        if not self.cap.isOpened():
            self.get_logger().error("Could not open video capture device.")
            return

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.detector_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.detector_params)
        
        # Variable to store last known pose
        self.last_known_pose = None

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("Failed to grab frame.")
            return

        corners, ids, _ = self.detector.detectMarkers(frame)
        robot_data = []

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            for i, marker_id in enumerate(ids.flatten()):
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners[i], MARKER_SIZE, camera_matrix, dist_coeffs
                )

                robot_pos, yaw_angle = compute_robot_pose_fixed_height(rvec[0], tvec[0], marker_id)
                if robot_pos is not None:
                    robot_data.append(np.array([robot_pos[0], robot_pos[1], yaw_angle]))

        if robot_data:
            avg_data = np.mean(np.array(robot_data), axis=0)

            msg = Vector3()
            msg.x = round(avg_data[0], 3)
            msg.y = round(avg_data[1], 3)
            msg.z = round(avg_data[2], 3)  # yaw

            self.publisher_.publish(msg)
            self.last_known_pose = msg  # Store the last known pose
            self.get_logger().info(f"Published Pose: x={msg.x}, y={msg.y}, yaw={msg.z} rad")
        elif self.last_known_pose is not None:
            # Publish last known pose if no markers detected
            self.publisher_.publish(self.last_known_pose)
            self.get_logger().info(f"Published Last Known Pose: x={self.last_known_pose.x}, y={self.last_known_pose.y}, yaw={self.last_known_pose.z} rad")

        cv2.imshow("Robot Camera View", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("Shutting down camera stream...")
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ArucoPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

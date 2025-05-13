import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import open3d as o3d

class PointCloudProcessor(Node):
    def __init__(self):
        super().__init__('pointcloud_processor')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/depth/color/points',
            self.pointcloud_callback,
            10)
        self.camera_tilt_deg = -30  # Adjust this based on your setup

    def pointcloud_callback(self, msg):
        # Convert PointCloud2 to numpy array
        points = np.array([
            p for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        ])
        if points.shape[0] == 0:
            self.get_logger().warn("No points in point cloud")
            return

        # Apply inverse tilt (rotate upward around x-axis)
        theta = np.radians(-self.camera_tilt_deg)
        R = np.array([
            [1, 0, 0],
            [0, np.cos(theta), -np.sin(theta)],
            [0, np.sin(theta),  np.cos(theta)]
        ])
        rotated_points = points @ R.T

        # Outlier removal
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(rotated_points)
        filtered_cloud, _ = cloud.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        rotated_points = np.asarray(filtered_cloud.points)

        # Further Y-Z filtering to isolate floor candidates
        thresh_y = (rotated_points[:, 1].max() - rotated_points[:, 1].min()) * 0.1 + rotated_points[:, 1].min()
        mask_y = rotated_points[:, 1] < thresh_y
        filtered_points = rotated_points[mask_y]
        thresh_z = (filtered_points[:, 2].min() - filtered_points[:, 2].max()) * 0.1 + filtered_points[:, 2].max()
        mask_z = filtered_points[:, 2] > thresh_z
        filtered_points = filtered_points[mask_z]

        if filtered_points.shape[0] < 10:
            self.get_logger().warn("Not enough filtered points for floor plane detection")
            return

        filtered_cloud = o3d.geometry.PointCloud()
        filtered_cloud.points = o3d.utility.Vector3dVector(filtered_points)

        try:
            plane_model, _ = filtered_cloud.segment_plane(
                distance_threshold=0.02, ransac_n=3, num_iterations=1000)
        except RuntimeError:
            self.get_logger().warn("Plane segmentation failed")
            return

        a, b, c, d = plane_model
        normal = np.array([a, b, c]) / np.linalg.norm([a, b, c])
        dot_to_y = np.dot(normal, [0, 1, 0])
        self.get_logger().info(f"Plane normal: {normal}, dot to y: {dot_to_y:.2f}")

        if dot_to_y < 0.8:
            self.get_logger().warn("Detected plane is not horizontal enough. Aborting classification.")
            return

        # Compute distances to the plane
        distances = np.abs((rotated_points @ normal) + d)
        threshold = 0.08
        inliers = distances < threshold

        floor_points = rotated_points[inliers]
        obstacle_points = np.delete(rotated_points, inliers, axis=0)

        self.get_logger().info(f"Classified {len(floor_points)} floor points, {len(obstacle_points)} obstacle points")

        # Optional: store or process the floor and obstacle points
        # e.g., publish, visualize, save, etc.

        obstacle_xy = obstacle_points[:, :2]
        
def main(args=None):
    rclpy.init(args=args)
    node = PointCloudProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

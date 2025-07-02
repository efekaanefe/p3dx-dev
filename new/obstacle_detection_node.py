import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import socket
import json
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
import numpy as np
import open3d as o3d
import cv2


class PointCloudProcessor(Node):
    def __init__(self):
        super().__init__('pointcloud_processor')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/depth/color/points',
            self.pointcloud_callback,
            10)
        self.obstacle_pub = self.create_publisher(PoseArray, "obstacle_points", 10)
        self.camera_tilt_deg = -30  # Adjust this based on your setup
        self.last_plane = None  # To store (normal, d)
        self.floor_value = 0.87
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name='Live PointCloud', width=800, height=600)
        self.vis_geometry_added = False
        self.vc = self.vis.get_view_control()
        self.pcd = o3d.geometry.PointCloud()
        self.vis.add_geometry(self.pcd)

    def pointcloud_callback(self, msg):
        try:
            points = []
            for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                points.append(p)
            points = np.array(points)

            if len(points) == 0:
                self.get_logger().warn("No valid points received")
                return

            sample = self.sample_the_points(points)
            normal, d = self.fit_plane(sample)
            floor_points, obstacle_points = self.compute_dist(points, normal, d)
            floor_points_n = self.rotation_to_z(normal, floor_points)
            obstacle_points_n = self.rotation_to_z(normal, obstacle_points)
            img, coords = self.project(obstacle_points_n)
            if img is not None:
                cv2.imshow("Obstacle Projection", img)
                cv2.waitKey(1)  # Wait 1ms to process GUI events
            if np.count_nonzero(img) == 0:
                self.get_logger().warn("Projected image is completely black")
            self.publish_obstacle_coords(coords)
            self.update_viewer(floor_points_n, obstacle_points_n)

        except Exception as e:
            self.get_logger().error(f"Error in pointcloud callback: {str(e)}")

    def sample_the_points(self, pts: np.ndarray) -> np.ndarray:
        dists = np.linalg.norm(pts, axis=1)
        horizontal_limit = np.min(pts[:, 2]) + 0.1
        mask_dist = dists < np.linalg.norm([self.floor_value, horizontal_limit])
        mask_y = pts[:, 1] > self.floor_value - 0.8

        mask = mask_dist & mask_y

        sample_points = pts[mask]
        return sample_points

    def fit_plane(self, points):
        if len(points)<= 0:
            self.get_logger().warn("Not enough sample points for segmentation")
            return None, None
        try:
            cloud = o3d.geometry.PointCloud()
            cloud.points = o3d.utility.Vector3dVector(points)
            plane_model, _ = cloud.segment_plane(
                distance_threshold=0.05, ransac_n=3, num_iterations=5)
        except RuntimeError:
            self.get_logger().warn("Plane segmentation failed")
            return None, None

        a, b, c, d = plane_model
        normal = np.array([a, b, c]) / np.linalg.norm([a, b, c])
        dot_to_y = np.dot(normal, [0, 1, 0])
        self.get_logger().info(f"Plane normal: {normal}, dot to y: {dot_to_y:.2f}")

        self.last_plane = (normal, d)
        return normal, d

    def compute_dist(self, points, normal, d, threshold=0.08):
        # Compute distances to the plane
        distances = np.abs((points @ normal) + d)
        inliers = distances < threshold
        floor_points = points[inliers]
        obstacle_points = np.delete(points, inliers, axis=0)
        self.get_logger().info(f"Classified {len(floor_points)} floor points, {len(obstacle_points)} obstacle points")

        # Optional: store or process the floor and obstacle points
        # e.g., publish, visualize, save, etc.


        return floor_points, obstacle_points

    def update_viewer(self, floor_np: np.ndarray, obstacle_np: np.ndarray):
        try:
            self.pcd.clear()
            if len(floor_np) == 0 and len(obstacle_np) == 0:
                self.get_logger().warn("No points to display")
                return

            all_points = np.vstack([floor_np, obstacle_np])
            colors = np.vstack([
                np.tile([0, 0, 1], (len(floor_np), 1)),  # blue = floor
                np.tile([1, 0, 0], (len(obstacle_np), 1))  # red = obstacle
            ])

            self.pcd.points = o3d.utility.Vector3dVector(all_points)
            self.pcd.colors = o3d.utility.Vector3dVector(colors)

            bbox = self.pcd.get_axis_aligned_bounding_box()
            center = bbox.get_center()
            extent = bbox.get_extent()
            diameter = np.linalg.norm(extent)
            if not self.vis_geometry_added:
                self.vis.add_geometry(self.pcd)
                self.vis_geometry_added = True
            else:
                bbox = self.pcd.get_axis_aligned_bounding_box()
                center = bbox.get_center()
                extent = bbox.get_extent()
                diameter = np.linalg.norm(extent)
                self.vis.update_geometry(self.pcd)

            self.vc.set_front([0, 0, 1])
            self.vc.set_up([0, -1, 0])
            self.vc.set_zoom(1.0 / 5.0)
            self.vis.poll_events()
            self.vis.update_renderer()
            # self.vis.reset_view_point(True)
        except Exception as e:
            self.get_logger().error(f"Error in update_viewer: {str(e)}")

    def project(self, pts: np.ndarray, grid_size=(512,512)):
    
        # 2. Clip to XY range
        x_min, x_max = -3, 3
        y_min, y_max = -2, 6 #np.max(pts[:, 1]) + 1
        if len(pts) == 0:
            return None, None
    
        # 3. Project to image grid
        img = np.zeros(grid_size, dtype=np.uint8)
        x_range = x_max - x_min
        y_range = y_max - y_min
    
        u = ((pts[:, 0] - x_min) / x_range * (grid_size[1] - 1)).astype(int)
        v = ((pts[:, 2] - y_min) / y_range * (grid_size[0] - 1)).astype(int)
    
        # flip vertical axis so image is upright
        v = grid_size[0] - 1 - v
        u = np.clip(u, 0, grid_size[1] - 1)
        v = np.clip(v, 0, grid_size[0] - 1)

        img[v, u] = 255
        
        # 4. Morphological filtering (remove noise)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        img_clean = cv2.erode(img, kernel, iterations=1)
        img_clean = cv2.dilate(img_clean, kernel, iterations=1)
        # 5. Optionally: map back to original 3D points
        v, u = np.where(img_clean > 0)
        x = x_min + (u / (grid_size[1] - 1)) * (x_range)
        y = y_min + ((grid_size[0] - 1 - v) / (grid_size[0] - 1)) * (y_range)
    
        return img_clean, np.column_stack((x, y))

    def rotation_to_z(self, v, pts):
        v = v / np.linalg.norm(v)
        y = np.array([0, 1, 0])

        if np.allclose(v, y):
            return pts
        if np.allclose(v, -y):
            # 180-degree rotation around any perpendicular axis (e.g., x)
            return pts

        axis = np.cross(v, y)
        axis = axis / np.linalg.norm(axis)
        angle = -np.arccos(np.dot(v, y))

        K = np.array([
            [0, -axis[2], axis[1]],
            [axis[2], 0, -axis[0]],
            [-axis[1], axis[0], 0]
        ])

        R = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * np.dot(K, K)
        pts = pts @ R
        return pts

    def publish_obstacle_coords(self, obstacle_xy):
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = "map"  # or your camera frame

        for x, y in obstacle_xy:
            pose = Pose()
            pose.position.x = float(x)
            pose.position.y = float(y)
            pose.position.z = 0.0  # optional
            pose.orientation.w = 1.0  # identity quaternion
            pose_array.poses.append(pose)

        self.obstacle_pub.publish(pose_array)
        print(f"Published {len(pose_array.poses)} poses.")


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudProcessor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

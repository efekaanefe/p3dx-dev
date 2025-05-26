import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import socket
import json
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
        self.last_plane = None  # To store (normal, d)
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name='Live PointCloud', width=800, height=600)
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
            
            rotated = self.filter_n_turn(points, self.camera_tilt_deg)
            filtered = self.mask_the_floor(rotated)
            if filtered is None:
                if self.last_plane is not None:
                    print("[Frame] Using cached plane (not enough filtered points)")
                    normal, d = self.last_plane
                    floor_xy, obstacle_xy, floor_xyz, obstacle_xyz = self.compute_dist(rotated, normal, d)
                    self.update_viewer(floor_xyz, obstacle_xyz)
                    return
                else:
                    print("[Frame] No valid filtered points and no cached plane.")
                    self.update_viewer(rotated, np.array([]).reshape(0, 3))
                    return
            cloud = o3d.geometry.PointCloud()
            cloud.points = o3d.utility.Vector3dVector(filtered)

            result = self.fit_plane(cloud)
            if result:
                normal, d = result
                floor_xy, obstacle_xy, floor_xyz, obstacle_xyz = self.compute_dist(rotated, normal, d)
                #self.send_obstacle_xy(obstacle_xy)
                print(f"[Frame] Obstacles: {obstacle_xy.shape[0]} points")
                self.update_viewer(floor_xyz, obstacle_xyz)
            else:
                print("[Frame] Plane fitting failed.")
                self.update_viewer(rotated, np.array([]).reshape(0, 3))

        except Exception as e:
            self.get_logger().error(f"Error in pointcloud callback: {str(e)}")

         # Apply inverse camera tilt (rotate upward around x-axis)
    def filter_n_turn(self, points, camera_tilt_deg):
        theta = np.radians(-camera_tilt_deg)  # reverse tilt
        R = np.array([
            [1, 0, 0],
            [0, np.cos(theta), -np.sin(theta)],
            [0, np.sin(theta),  np.cos(theta)]
        ])
        RotZ = np.array([
            [np.cos(np.pi), -np.sin(np.pi), 0],
            [np.sin(np.pi),  np.cos(np.pi), 0],
            [0, 0, 1]
        ])
        RotY = np.array([
            [np.cos(np.pi), 0,  np.sin(np.pi)],
            [0, 1, 0],
            [-np.sin(np.pi), 0, np.cos(np.pi)]
        ])

        # Combine rotations
        R_total = RotZ @ R @ RotY
        rotated_points = points @ R_total.T

        # Create Open3D point cloud
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(rotated_points)

        # Remove outliers (noise)
        filtered_cloud, _ = cloud.remove_statistical_outlier(
            nb_neighbors=points.shape[0]*0.1,
            std_ratio=2.0
        )
        rotated_points = np.asarray(filtered_cloud.points)
        return rotated_points
        
    def mask_the_floor(self,rotated_points,y_ratio=0.1,z_ratio=0.2):
        # Further Y-Z filtering to isolate floor candidates
        y_diff = (rotated_points[:, 1].max() - rotated_points[:, 1].min()) 
        thresh_y = y_diff * y_ratio + rotated_points[:, 1].min()
        mask_y = rotated_points[:, 1] < thresh_y
        if y_diff > 0.2 and rotated_points[mask_y].shape[0] > 10:
            filtered_points = rotated_points[mask_y]
        else:
            filtered_points = rotated_points
        thresh_z = (filtered_points[:, 2].min() - filtered_points[:, 2].max()) * z_ratio + filtered_points[:, 2].max()
        mask_z = filtered_points[:, 2] > thresh_z
        filtered_points = filtered_points[mask_z]

        if filtered_points.shape[0] < 10:
            print('Not enough point to fit plane')
            return
        else:
            filtered_cloud = o3d.geometry.PointCloud()
            filtered_cloud.points = o3d.utility.Vector3dVector(filtered_points)
        filtered_points = np.asarray(filtered_cloud.points)
        return filtered_points
    
    def fit_plane(self,filtered_cloud):
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
            return self.last_plane
        
        self.last_plane = (normal, d)
        return normal,d
    
    def compute_dist(self,rotated_points,normal,d,threshold = 0.08):
        # Compute distances to the plane
        distances = np.abs((rotated_points @ normal) + d)
        inliers = distances < threshold
        distances_below = obstacle_points @ normal_unit + d
        floor_points = rotated_points[inliers]
        obstacle_points = np.delete(rotated_points, inliers, axis=0)
        obstacle_points = obstacle_points[distances_below >= 0]
        self.get_logger().info(f"Classified {len(floor_points)} floor points, {len(obstacle_points)} obstacle points")

        # Optional: store or process the floor and obstacle points
        # e.g., publish, visualize, save, etc.

        # Map your [-Z, X] → their [X, Y]
        obstacle_xy = np.column_stack([-obstacle_points[:, 2], obstacle_points[:, 0]])
        floor_xy = np.column_stack([-floor_points[:, 2], floor_points[:, 0]])

        return floor_xy,obstacle_xy,floor_points,obstacle_points
    
    def update_viewer(self, floor_np, obstacle_np):
        try:
            self.pcd.clear()
            if len(floor_np) == 0 and- len(obstacle_np) == 0:
                self.get_logger().warn("No points to display")
                return
            
            all_points = np.vstack([floor_np, obstacle_np])
            colors = np.vstack([
                np.tile([0, 0, 1], (len(floor_np), 1)),     # blue = floor
                np.tile([1, 0, 0], (len(obstacle_np), 1))   # red = obstacle
            ])

            self.pcd.points = o3d.utility.Vector3dVector(all_points)
            self.pcd.colors = o3d.utility.Vector3dVector(colors)

            self.vis.update_geometry(self.pcd)
            self.vis.poll_events()
            self.vis.update_renderer()
            self.vis.reset_view_point(True)
        except Exception as e:
            self.get_logger().error(f"Error in update_viewer: {str(e)}")

    def send_obstacle_xy(self, obstacle_xy):
        coords_list = [tuple(row) for row in obstacle_xy]
        data = json.dumps(coords_list).encode('utf-8')

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect(('127.0.0.1', 5000))  # or remote IP and port
            s.sendall(data)

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

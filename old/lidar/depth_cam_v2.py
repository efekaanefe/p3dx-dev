import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import open3d as o3d
from typing import Optional


class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/depth/color/points',
            self.pointcloud_callback,
            10)
        self.camera_tilt_deg = -20.0     # adjust for your mount
        # parameters for floor sampling/classification
        self.y_percentile = 5.0          # take the closest 5% by forward distance
        self.z_percentile = 5.0          # take the lowest 5% by height
        self.z_tolerance  = 0.1         # ±5 cm around floor median
        self.floor_value = -1.2         # expected z value of floor pts
        
        # set up Open3D visualizer
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(
            window_name='L515 Obstacle View', width=800, height=600)
        self.pcd = o3d.geometry.PointCloud()
        self.vis_geometry_added = False
        self.vis.add_geometry(self.pcd)

    def pointcloud_callback(self, msg: PointCloud2):
        # 1) unpack into Nx3 numpy array
        pts = np.array(list(pc2.read_points(
            msg, field_names=('x','y','z'), skip_nans=True)))
        if pts is None or len(pts) == 0:
            self.get_logger().warn("Empty or invalid point array received.")
            return None
        pts = self.filter_points(pts)
        # 2) rotate into camera frame (y forward, x right, z up)
        pts = self.rotate_frame(self.camera_tilt_deg,pts)
        sample_pts = self.sample_the_points(pts)
        '''
        if self.needs_tilt_correction(pts):
            correction_angle = self.estimate_tilt_from_ransac(sample_pts)
            if correction_angle is not None:
                self.get_logger().warn(f"Tilt correction applied: {correction_angle:.2f}°")
                correction_angle = np.radians(correction_angle)
                R_corr = np.array([
                    [1, 0, 0],
                    [0, np.cos(correction_angle), -np.sin(correction_angle)],
                    [0, np.sin(correction_angle),  np.cos(correction_angle)]
                ])
                pts = pts @ R_corr.T
                sample_pts = self.sample_the_points(pts)
        '''
        # 3) sample closest & lowest points to estimate floor height
        ave_floor_z = self.estimate_floor_z(sample_pts)
        print(ave_floor_z)
        if ave_floor_z is None:
            self.get_logger().warn("not enough sample points for floor, default z value will be used")
            z_val = np.min(pts[:,2])
            return z_val

        # 4) classify
        dz = np.abs(pts[:,2] - ave_floor_z)
        is_floor = dz < self.z_tolerance
        floor_pts    = pts[is_floor]
        obstacle_pts = pts[~is_floor]
        self.get_logger().info(
            f"floor={len(floor_pts)}, obstacles={len(obstacle_pts)}")
        
        # 5) update Open3D view
        floor_pts_o3d = self.baseframe2o3dframe(floor_pts)
        obstacle_pts_o3d = self.baseframe2o3dframe(obstacle_pts)
        self.update_viewer(floor_pts_o3d, obstacle_pts_o3d)

    def filter_points(self, pts: np.ndarray) -> np.ndarray:
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(pts)

        # Remove outliers (noise)
        filtered_cloud, _ = cloud.remove_statistical_outlier(
            nb_neighbors=300,
            std_ratio=2.0
        )
        filtered_points = np.asarray(filtered_cloud.points)
        return filtered_points
    
    def rotate_frame(self, tilt_angle, pts: np.ndarray) -> np.ndarray:
        # undo tilt around X:
        th = np.radians(-tilt_angle + np.pi/2)
        R_x = np.array([
            [1,         0,          0],
            [0,  np.cos(th), -np.sin(th)],
            [0,  np.sin(th),  np.cos(th)]
        ])
        # flip axes so Y is forward, X is right, Z is up:
        R_z = np.array([    # 180° about Z
            [-1,  0, 0],
            [ 0, -1, 0],
            [ 0,  0, 1]
        ])
        R_y = np.array([    # 180° about Y
            [-1, 0,  0],
            [ 0, 1,  0],
            [ 0, 0, -1]
        ])
        R = R_x
        # optional: statistical outlier removal to denoise
        rotated_points = pts @ R.T
        return np.asarray(rotated_points)

    def sample_the_points(self, pts: np.ndarray) -> np.ndarray:
        y_th = np.percentile(pts[:,1], self.y_percentile)
        z_th = np.percentile(pts[:,2], self.z_percentile)
        print("y_th:", y_th, "z_th:", z_th)
        '''
        sample = pts[
            (pts[:, 1] <= y_th) &
            (pts[:, 2] <= z_th) &
            (np.abs(pts[:, 2] - self.floor_value) <= self.z_tolerance)
            ]
        '''
        sample = pts[(pts[:, 1] <= y_th)]
        sample = sample[(sample[:, 2] <= z_th)]

        return sample

    def estimate_floor_z(self, sample_pts: np.ndarray) -> Optional[float]:
        # pick the closest y-percentile (smallest y) AND lowest z-percentile:
        if len(sample_pts) < 10:
            self.get_logger().warn("Not enough sample points found to estimate floor plane.")
            return -0.5
        return float(np.average(sample_pts[:,2]))
    
    def needs_tilt_correction(self, pts: np.ndarray) -> bool:
        y_vals = pts[:, 1]
        z_vals = pts[:, 2]

        y10 = np.percentile(y_vals, 10)
        y90 = np.percentile(y_vals, 90)

        front_zs = z_vals[y_vals <= y10]
        back_zs  = z_vals[y_vals >= y90]

        if len(front_zs) < 5 or len(back_zs) < 5:
            return False  # not enough points to decide
        
        z_front = np.min(front_zs)
        z_back  = np.min(back_zs)
        delta_z = abs(z_back - z_front)
        self.get_logger().info(f"Front z: {z_front:.3f}, Back z: {z_back:.3f}, Δz: {delta_z:.3f}")

        return delta_z > 0.05
    
    def estimate_tilt_from_ransac(self, sample_pts: np.ndarray) -> Optional[float]:
        if len(sample_pts) < 30:
            return None
        # Use Open3D for RANSAC plane fitting
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(sample_pts)
        plane_model, inliers = pcd.segment_plane(
            distance_threshold=0.01,
            ransac_n=3,
            num_iterations=10
        )
        a, b, c, _ = plane_model
        normal = np.array([a, b, c])
        normal /= np.linalg.norm(normal)
        dot = normal @ np.array([0, 0, 1])  # world Z
        dot = np.clip(dot, -1.0, 1.0)
        angle_rad = np.arccos(dot)
        tilt_deg = np.degrees(angle_rad)

        if normal[1] > 0:  # flip sign if facing forward
            tilt_deg *= -1
        return tilt_deg

    def baseframe2o3dframe(pts: np.ndarray):
        np.column_stack([pts[:, 0], pts[:, 2], -pts[:, 1]])
        return pts

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

            if not self.vis_geometry_added:
                self.vis.add_geometry(self.pcd)
                self.vis_geometry_added = True
            else:
                self.vis.update_geometry(self.pcd)

            self.vis.poll_events()
            self.vis.update_renderer()
            self.vis.reset_view_point(True)
        except Exception as e:
            self.get_logger().error(f"Error in update_viewer: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

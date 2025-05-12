import numpy as np
import open3d as o3d

def load_pointcloud_from_npy(file_path, camera_tilt_deg=-15.0):
    import numpy as np
    import open3d as o3d

    # Load points: shape (N, 3)
    points = np.load(file_path)

    # Apply inverse camera tilt (rotate upward around x-axis)
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
        nb_neighbors=20,
        std_ratio=2.0
    )
    rotated_points = np.asarray(filtered_cloud.points)

    return filtered_cloud, rotated_points


def classify_floor_and_obstacles(cloud, rotated_points):
    # Fit plane with RANSAC
    thresh_y = (rotated_points[:, 1].max() - rotated_points[:, 1].min()) * 0.1 + rotated_points[:, 1].min()
    mask_y = rotated_points[:, 1] < thresh_y
    filtered_points = rotated_points[mask_y]
    thresh_z = (filtered_points[:, 2].min() - filtered_points[:, 2].max()) * 0.1 + filtered_points[:, 2].max()
    mask_z = filtered_points[:, 2] > thresh_z
    filtered_points = filtered_points[mask_z]

    # Open3D point cloud oluştur
    filtered_cloud = o3d.geometry.PointCloud()
    filtered_cloud.points = o3d.utility.Vector3dVector(filtered_points)
    o3d.visualization.draw_geometries([filtered_cloud])
    plane_model, _ = filtered_cloud.segment_plane(
        distance_threshold=0.02,
        ransac_n=3,
        num_iterations=1000
    )

    a, b, c, d = plane_model
    normal = np.array([a, b, c]) / np.linalg.norm([a, b, c])
    dot_to_y = np.dot(normal, [0, 1, 0])

    print(f"Plane normal: {normal}, dot to y: {dot_to_y:.2f}")

    if dot_to_y < 0.8:
        print("Detected plane is not horizontal enough. Aborting classification.")
        return [], list(map(tuple, rotated_points))  # Everything is obstacle
    distances = np.abs((rotated_points @ normal) + d)

    # Inliers = all points close to the plane
    threshold = 0.08  # same as above
    inliers = distances < threshold

    floor_points = rotated_points[inliers]
    obstacle_points = np.delete(rotated_points, inliers, axis=0)

    floor = list(map(tuple, floor_points))
    obstacle = list(map(tuple, obstacle_points))

    print(f"Classified: {len(floor)} floor points, {len(obstacle)} obstacle points")

    if floor or obstacle:
        # Zemin (mavi)
        floor_pcd = o3d.geometry.PointCloud()
        floor_pcd.points = o3d.utility.Vector3dVector(np.array(floor))
        floor_pcd.paint_uniform_color([0, 0, 1])  # mavi

        # Engel (kırmızı)
        obstacle_pcd = o3d.geometry.PointCloud()
        obstacle_pcd.points = o3d.utility.Vector3dVector(np.array(obstacle))
        obstacle_pcd.paint_uniform_color([1, 0, 0])  # kırmızı

        # Göster
        o3d.visualization.draw_geometries([floor_pcd, obstacle_pcd])
    else:
        print("No points to visualize.")

    return floor, obstacle

# Example usage
if __name__ == "__main__":
    npy_path = "/home/romer/pointcloud_sample.npy"  # Your saved file
    cloud, rotated_points = load_pointcloud_from_npy(npy_path)
    '''
    raw_pcd = o3d.geometry.PointCloud()
    raw_pcd.points = o3d.utility.Vector3dVector(rotated_points)
    raw_pcd.paint_uniform_color([0.5, 0.5, 0.5])  # gri

    o3d.visualization.draw_geometries([raw_pcd])
    '''
    floor, obstacle = classify_floor_and_obstacles(cloud, rotated_points)

    # (Optional) visualize
    floor_pcd = o3d.geometry.PointCloud()
    floor_pcd.points = o3d.utility.Vector3dVector(np.array(floor))

    obstacle_pcd = o3d.geometry.PointCloud()
    obstacle_pcd.points = o3d.utility.Vector3dVector(np.array(obstacle))

    floor_pcd.paint_uniform_color([0, 1, 0])
    obstacle_pcd.paint_uniform_color([1, 0, 0])

    o3d.visualization.draw_geometries([floor_pcd, obstacle_pcd])

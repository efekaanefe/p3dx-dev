import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
import socket
import json
import time

s = None
def generate_synthetic_pointcloud(n_floor=12000, object_specs=None):
    # Floor generation
    x_floor = np.random.uniform(-2.5, 2.5, n_floor)
    z_floor = np.random.uniform(0.5, 6.0, n_floor)
    y_floor = np.random.normal(0, 0.015, n_floor)
    floor_points = np.stack((x_floor, y_floor, z_floor), axis=1)

    # Default object specs if not provided
    if object_specs is None:
        object_specs = [
            {"center": (1.0, 0.3, 4.0), "size": (0.6, 0.6, 0.04), "count": 700},   # short wide object
            {"center": (1.5, 0.5, 2.0), "size": (1.8, 1.0, 0.03), "count": 1800},    # tall thin object
            {"center": (-1.0, 1.25, 2.0), "size": (0.5, 2.5, 0.05), "count": 1000},  # cube
            {"center": (-2.0, 0.6, 1.5), "size": (1.2, 1.2, 0.02), "count": 1600},   # tall pole-like
        ]

    object_points = []

    for obj in object_specs:
        cx, cy, cz = obj["center"]
        sx, sy, sz = obj["size"]
        count = obj["count"]

        x_obj = np.random.uniform(cx - sx/2, cx + sx/2, count)
        y_obj = np.random.uniform(cy - sy/2, cy + sy/2, count)
        z_obj = np.random.uniform(cz - sz/2, cz + sz/2, count)

        points = np.stack((x_obj, y_obj, z_obj), axis=1)
        object_points.append(points)

    # Combine floor and objects
    all_points = np.vstack([floor_points] + object_points)
    np.random.shuffle(all_points)
    object_points = np.vstack(object_points)
    return all_points, object_points


def save_pointcloud_as_pcd(points: np.ndarray, filename="synthetic_pointcloud.pcd"):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    o3d.io.write_point_cloud(filename, pcd)
    return pcd


def visualize_pointcloud(pcd):
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(pcd)

    ctr = vis.get_view_control()

    # Set the view to look toward positive Z
    ctr.set_front([0.0, 0.0, -1.0])       # camera faces toward +Z
    ctr.set_lookat([0.0, 0.85, 3.0])     # look at some point along Z at floor level
    ctr.set_up([0.0, 1.0, 0.0])         # y down in your case (sensor convention)

    ctr.set_zoom(0.45)  # Optional: adjust zoom level

    vis.run()
    vis.destroy_window()


def create_obstacle_grid(object_points, grid_res=0.1, x_range=(-3, 3), y_range=(0, 6)):
    # Convert Z as Y and X as X
    points_2d = object_points[:, [0, 2]]  # [x, z]

    # Quantize to grid
    x_min, x_max = x_range
    y_min, y_max = y_range

    x_bins = int((x_max - x_min) / grid_res)
    y_bins = int((y_max - y_min) / grid_res)

    grid = np.zeros((y_bins, x_bins), dtype=np.uint8)

    for x, y in points_2d:
        xi = int((x - x_min) / grid_res)
        yi = int((y - y_min) / grid_res)
        if 0 <= xi < x_bins and 0 <= yi < y_bins:
            grid[yi, xi] = 1

    return grid, x_bins, y_bins, x_min, y_min, grid_res

def compute_velocity_vector_from_voxels(voxels,  robot_pos=np.array([0.0, 0.0]),
                                        influence_radius=1.0, repulse_gain=10.0,
                                        attract_vector=np.array([0.0, 1.0]), attract_gain=1.0,
                                        repulse_power=3.0):
    """
    Compute total motion vector based on repulsion from obstacle voxels and constant attraction.
    """
    total_repulse = np.array([0.0, 0.0])

    for obs in voxels:
        diff = robot_pos - obs
        dist = np.linalg.norm(diff)

        if 1e-6 < dist < influence_radius:
            direction = diff / dist
            strength = repulse_gain * (1.0 / dist)**repulse_power
            total_repulse += strength * direction

    total_attract = attract_gain * attract_vector
    final_velocity = total_attract + total_repulse

    return final_velocity, total_repulse, total_attract


def voxel_desample_2d(points, resolution=0.1):
    """
    Perform 2D voxel desampling (on x and y axes).
    Args:
        points: (N, 2) array of [x, y] points
        resolution: size of voxel in meters
    Returns:
        desampled_points: (M, 2) array of voxel-averaged point coordinates
    """
    # Map to voxel indices
    voxel_keys = np.floor(points / resolution).astype(np.int32)
    
    # Unique voxel index per point
    voxel_hash = voxel_keys[:, 0] + 1000000 * voxel_keys[:, 2]  # combine x and y indices uniquely

    # Dict: key=voxel_hash, value=[sum_x, sum_y, count]
    voxel_dict = {}

    for i in range(len(points)):
        key = voxel_hash[i]
        if key not in voxel_dict:
            voxel_dict[key] = [points[i][0], points[i][2], 1]
        else:
            voxel_dict[key][0] += points[i][0]
            voxel_dict[key][1] += points[i][2]
            voxel_dict[key][2] += 1

    # Compute averages
    desampled_points = []
    for v in voxel_dict.values():
        avg_x = v[0] / v[2]
        avg_y = v[1] / v[2]
        desampled_points.append([avg_x, avg_y])

    return np.array(desampled_points)

def visualize_velocity_field(voxels, robot_pos=np.array([0.0, 0.0]), velocity=None):
    plt.figure(figsize=(8, 6))
    plt.axis("equal")
    plt.grid(True)

    # Plot obstacle points
    plt.scatter(voxels[:, 0], voxels[:, 1], c='red', s=10, label='Obstacle Voxels')

    # Plot robot
    plt.plot(robot_pos[0], robot_pos[1], 'bo', label='Robot (Origin)')

    # Plot final velocity vector
    if velocity is not None:
        plt.quiver(robot_pos[0], robot_pos[1], velocity[0], velocity[1],
                   angles='xy', scale_units='xy', scale=1.0, color='blue', width=0.007,
                   label='Final Velocity')

    plt.legend()
    plt.title("Robot Velocity Vector from Obstacle Repulsion + Forward Attraction")
    plt.xlabel("X")
    plt.ylabel("Z")
    plt.show()

def simulate_robot_motion(voxels, steps=100, dt=0.1, 
                          influence_radius=1.0, repulse_gain=0.2,
                          attract_vector=np.array([0.3, 1.0]), attract_gain=2.0,
                          repulse_power=2.0):
    robot_pos = np.array([0.0, 0.0])  # Start at origin
    trajectory = [robot_pos.copy()]

    for _ in range(steps):
        vel, _, _ = compute_velocity_vector_from_voxels(
            voxels,
            robot_pos=robot_pos,
            influence_radius=influence_radius,
            repulse_gain=repulse_gain,
            attract_vector=attract_vector,
            attract_gain=attract_gain,
            repulse_power=repulse_power
        )

        # Clamp velocity (optional)
        max_speed = 0.2  # meters/sec
        speed = np.linalg.norm(vel)
        if speed > max_speed:
            vel = (vel / speed) * max_speed

        # Update position
        robot_pos += vel * dt
        trajectory.append(robot_pos.copy())

    return np.array(trajectory)

def simulate_bicycle_motion(voxels, steps=100, dt=0.1, influence_radius=1.0, repulse_gain=0.2,
                          attract_vector=np.array([0.0, 1.0]), attract_gain=2.0,
                            repulse_power=2.0,
                            max_speed=0.2, max_steering=np.radians(25),  # max turn rate
                            wheelbase=0.38):
    pos = np.array([0.0, 0.0])  # initial position
    theta = np.pi /2  # initial orientation (facing +Y)

    trajectory = [pos.copy()]
    headings = [theta]

    for _ in range(steps):
        # Compute potential field velocity (desired direction)
        vel, _, _ = compute_velocity_vector_from_voxels(
            voxels,
            robot_pos=pos,
            influence_radius=influence_radius,
            repulse_gain=repulse_gain,
            attract_vector=attract_vector,
            attract_gain=attract_gain,
            repulse_power=repulse_power
        )

        if np.linalg.norm(vel) < 1e-4:
            break  # no significant motion

        # Desired heading angle from vector
        desired_theta = np.arctan2(vel[1], vel[0])

        # Compute steering angle (difference in heading)
        angle_diff = desired_theta - theta
        angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))  # wrap [-π, π]

        # Limit steering angle
        steering_angle = np.clip(angle_diff, -max_steering, max_steering)

        # Constant forward speed
        v = max_speed

        # Bicycle model update
        dx = v * np.cos(theta) * dt
        dy = v * np.sin(theta) * dt
        dtheta = (v / wheelbase) * np.tan(steering_angle) * dt

        pos += np.array([dx, dy])
        theta += dtheta

        trajectory.append(pos.copy())
        headings.append(theta)
        print(v, dtheta / dt)
        publish(v, dtheta/dt)
        time.sleep(0.1)     # 10 Hz
    return np.array(trajectory), np.array(headings), 

def plot_trajectory(voxels, trajectory):
    plt.figure(figsize=(8, 6))
    plt.axis("equal")
    plt.grid(True)

    plt.scatter(voxels[:, 0], voxels[:, 1], c='red', s=10, label='Obstacle Voxels')
    plt.plot(trajectory[:, 0], trajectory[:, 1], 'b-', label='Robot Path')
    plt.plot(trajectory[0, 0], trajectory[0, 1], 'go', label='Start')
    plt.plot(trajectory[-1, 0], trajectory[-1, 1], 'ro', label='End')

    plt.legend()
    plt.title("Robot Navigation via Repulsion + Forward Attraction")
    plt.xlabel("X")
    plt.ylabel("Z")
    plt.show()

def plot_trajectory_with_orientation(voxels, trajectory, headings, every=5):
    plt.figure(figsize=(8, 6))
    plt.axis("equal")
    plt.grid(True)

    plt.scatter(voxels[:, 0], voxels[:, 1], c='red', s=10, label='Obstacle Voxels')
    plt.plot(trajectory[:, 0], trajectory[:, 1], 'b-', label='Robot Path')

    for i in range(0, len(trajectory), every):
        x, y = trajectory[i]
        angle = headings[i]
        dx, dy = 0.3 * np.cos(angle), 0.3 * np.sin(angle)
        plt.arrow(x, y, dx, dy, head_width=0.05, color='green', alpha=0.6)

    plt.plot(trajectory[0, 0], trajectory[0, 1], 'go', label='Start')
    plt.plot(trajectory[-1, 0], trajectory[-1, 1], 'ro', label='End')

    plt.legend()
    plt.title("Bicycle Model Navigation with Repulsion + Forward Attraction")
    plt.xlabel("X")
    plt.ylabel("Z")
    plt.show()

def publish(linear_x, angular_z):
        msg = {
            "linear_x": float(linear_x),
            "angular_z": float(angular_z)
        }

        try:
            s.sendall(json.dumps(msg).encode('utf-8'))
            response = s.recv(1024)
        except Exception as e:
                print('e')

# Main execution
points,obstacles = generate_synthetic_pointcloud()
pcd = save_pointcloud_as_pcd(points)
visualize_pointcloud(pcd)

desampled_obstacles = voxel_desample_2d(obstacles, resolution=0.05)
'''
plt.scatter(obstacles[:, 0], obstacles[:, 2], s=1, label="Raw")
plt.scatter(desampled_obstacles[:, 0], desampled_obstacles[:, 1], s=10, label="Desampled")
plt.axis("equal")
plt.legend()
plt.title("2D Obstacle Point Desampling via Voxels")
plt.show()
'''
vel, repulse, attract = compute_velocity_vector_from_voxels(desampled_obstacles)

#visualize_velocity_field(desampled_obstacles, velocity=vel)

trajectory = simulate_robot_motion(desampled_obstacles, steps=500, dt=0.1)
plot_trajectory(desampled_obstacles, trajectory)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect(("192.168.40.97", 9090))

    bicycle_trajectory, headings = simulate_bicycle_motion(desampled_obstacles,steps=200,repulse_gain=0.1,
                              attract_vector=np.array([0.2, 1.0]), attract_gain=2.0,
                                repulse_power=4.0,
                                max_speed=0.2, max_steering=np.radians(50),  # max turn rate
                                wheelbase=0.4)
    plot_trajectory_with_orientation(desampled_obstacles, bicycle_trajectory, headings)

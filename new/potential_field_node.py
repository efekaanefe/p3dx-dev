import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, PoseArray
from std_msgs.msg import ByteMultiArray
import numpy as np
import socket
import json
from .p3dxRobot import p3dxRobot


class PotentialFieldNode(Node):
    def __init__(self):
        super().__init__('potential_field_node')

        self.obstacle_subscription = self.create_subscription(PoseArray, 'obstacle_points', self.obstacle_callback, 10)
        # Initial state
        self.heading = 0.0

        self.get_logger().info(f"Potential field controller initialized.")
        self.timer = self.create_timer(0.5, self.control_loop)
        self.iteration = 0

        self.obstacles = []

        self.robot = p3dxRobot(is_simulation=True)


    def publish(self, linear_x, angular_z):
        self.robot.set_velocity(linear_x,angular_z)

    def obstacle_callback(self, msg: PoseArray):
        new_obstacles = []
        for pose in msg.poses:
            x = pose.position.x
            y = pose.position.y
            new_obstacles.append([x, y])
        self.obstacles = new_obstacles
        #print(f"Recieved {len(msg.poses)} poses.")

    def compute_velocity_vector_from_voxels(self, voxels,
                                            influence_radius=1.0, repulse_gain=10.0,
                                            repulse_power=3.0):
        """
        Compute total motion vector based on repulsion from obstacle voxel
        """
        total_repulse = np.array([0.0, 0.0])

        for obs in voxels:
            diff =  -obs
            dist = np.linalg.norm(diff)

            if 1e-6 < dist < influence_radius:
                direction = diff / dist
                direction = np.array(direction)
                strength = repulse_gain * (1.0 / dist) ** repulse_power
                total_repulse += strength * direction

        return total_repulse

    def simulate_bicycle_motion(self, voxels, steps=100, dt=0.1, influence_radius=2.0, repulse_gain=0.01,
                                repulse_power=3.0,
                                max_speed=0.6, max_steering=np.radians(25),  # max turn rate
                                wheelbase=0.37):
        theta = 0  # initial orientation (facing +Y)

        # Compute potential field velocity (desired direction)
        vel = self.compute_velocity_vector_from_voxels(
            voxels,
            influence_radius=influence_radius,
            repulse_gain=repulse_gain,
            repulse_power=repulse_power
        )



        if np.linalg.norm(vel) < 1e-4:
            return None  # no significant motion

        # Desired heading angle from vector
        desired_theta = np.arctan2(-vel[1], -vel[0])

        # Compute steering angle (difference in heading)
        angle_diff = desired_theta - theta
        angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))  # wrap [-π, π]

        # Limit steering angle
        steering_angle = np.clip(angle_diff, -max_steering, max_steering)
        print(steering_angle)
        # Constant forward speed
        lineer_vel = np.dot([0, 1], vel)
        v = np.clip(lineer_vel, -max_speed, max_speed)
        # Bicycle model update
        dx = v * np.cos(theta) * dt
        dy = v * np.sin(theta) * dt
        dtheta = (v / wheelbase) * np.tan(steering_angle) * dt
        print(dtheta)
        angular_speed = -dtheta / dt
        theta += dtheta
        return v, angular_speed

    def voxel_desample_2d(self, points, resolution=0.05):
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
        voxel_hash = voxel_keys[:, 0] + 1000000 * voxel_keys[:, 1]  # combine x and y indices uniquely

        # Dict: key=voxel_hash, value=[sum_x, sum_y, count]
        voxel_dict = {}

        for i in range(len(points)):
            key = voxel_hash[i]
            if key not in voxel_dict:
                voxel_dict[key] = [points[i][0], points[i][1], 1]
            else:
                voxel_dict[key][0] += points[i][0]
                voxel_dict[key][1] += points[i][1]
                voxel_dict[key][2] += 1

        # Compute averages
        desampled_points = []
        for v in voxel_dict.values():
            avg_x = v[0] / v[2]
            avg_y = v[1] / v[2]
            desampled_points.append([avg_x, avg_y])

        return np.array(desampled_points)

    def control_loop(self):
        # Call your bicycle simulation each loop
        obstacles = np.array(self.obstacles)  # obstacle list should be Nx2 np array
        if obstacles.size==0:
            self.publish(0.0, 0.0)
            return None
        voxels = self.voxel_desample_2d(obstacles)
        # Simulate 1-step bicycle motion
        result = self.simulate_bicycle_motion(
            voxels,
            influence_radius= 1.0,
            steps=1,  # simulate only 1 step forward
            dt=0.1,
        )
        if result is not None:
            v, angular_z = result
            self.publish(v, angular_z)
        else:
            self.get_logger().warn("No significant motion vector computed.")
            self.publish(0.0, 0.0)


# Main function
def main():
    rclpy.init()
    node = PotentialFieldNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

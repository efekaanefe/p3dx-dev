import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, PoseArray
from std_msgs.msg import ByteMultiArray
import numpy as np
import socket
import json

class PotentialFieldNode(Node):
    def __init__(self):
        super().__init__('potential_field_node')

        self.publisher = self.create_publisher(Twist, 'obstacle_vel', 10)
        self.obstacle_subscription = self.create_subscription(PoseArray, 'obstacle_points', self.obstacle_callback, 10)

        # Initial state
        self.position = [0.0, 0.0]
        self.robot_x_dir = [1.0, 0.0]  # Assume robot starts facing x direction
        self.heading = 0.0
        self.target_dir = [1,1]

        self.get_logger().info(f"Potential field controller initialized.")
        self.timer = self.create_timer(0.032, self.control_loop)
        self.iteration = 0

        self.obstacles = []

    def publish(self, linear_x, angular_z):
        msg = {
            "linear_x": float(linear_x),
            "angular_z": float(angular_z)
        }

        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect(("192.168.1.42", 9090))  # ← replace with actual IP of the Pioneer robot server
                s.sendall(json.dumps(msg).encode('utf-8'))
                response = s.recv(1024)
                self.get_logger().info(f"TCP Response: {response.decode('utf-8')}")
        except Exception as e:
            self.get_logger().error(f"Failed to send velocity command via TCP: {e}")

    def obstacle_callback(self, msg: PoseArray):
        new_obstacles = []
        for pose in msg.poses:
            x = pose.position.x
            y = pose.position.y
            new_obstacles.append([x, y])
        self.obstacles = new_obstacles

    def compute_velocity_vector_from_voxels(self,voxels,  robot_pos=np.array([0.0, 0.0]),
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

    def simulate_bicycle_motion(self,voxels, steps=100, dt=0.1, influence_radius=1.0, repulse_gain=0.2,
                            attract_vector=np.array([0.0, 1.0]), attract_gain=2.0,
                                repulse_power=2.0,
                                max_speed=0.2, max_steering=np.radians(25),  # max turn rate
                                wheelbase=0.5):
        pos = np.array([0.0, 0.0])  # initial position
        theta = np.pi /2  # initial orientation (facing +Y)


            # Compute potential field velocity (desired direction)
        vel, _, _ = self.compute_velocity_vector_from_voxels(
                voxels,
                robot_pos=pos,
                influence_radius=influence_radius,
                repulse_gain=repulse_gain,
                attract_vector=attract_vector,
                attract_gain=attract_gain,
                repulse_power=repulse_power
            )

        if np.linalg.norm(vel) < 1e-4:
            return None  # no significant motion

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

        angular_speed = dtheta/dt

        pos += np.array([dx, dy])
        theta += dtheta

        return v, angular_speed

    def control_loop(self):
        # Call your bicycle simulation each loop
        voxels = np.array(self.obstacles)  # obstacle list should be Nx2 np array
        target_dir = self.target_dir

        # Simulate 1-step bicycle motion
        v, angular_z = self.simulate_bicycle_motion(
            voxels,
            steps=1,  # simulate only 1 step forward
            dt=0.1,
            attract_vector=target_dir,
            attract_gain=2.0
        )

        self.publish(v, angular_z)
   
# Main function
def main():
    rclpy.init()
    node = PotentialFieldNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
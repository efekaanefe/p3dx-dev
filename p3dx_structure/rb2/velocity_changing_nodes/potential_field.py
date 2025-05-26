from utils import *
import math
import random
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, PoseArray

# Define constants
MAX_SPEED = 6.28  # Max rotational speed (rad/s)
TIMESTEP = 32 # Controller time step in milliseconds

# Potential field parameters
Kv = 5 # Attraction distance-based gain to target
Kvp = 20 # Attractive constant gain
Ko = 0.1
Ka = 5
target_THRESHOLD = 0.1  # Distance to target to consider it reached
INFLUENCE_RADIUS = 0.1  # Radius of influence for obstacles
K_field, K_key = 1, 1



class PotentialFieldNode(Node):
    def __init__(self):
        super().__init__('potential_field_node')

        self.publisher = self.create_publisher(Twist, 'obstacle_vel', 10)
        self.status_subscription = self.create_subscription(Pose, 'robot_status', self.robot_status_callback, 10)
        self.obstacle_subscription = self.create_subscription(PoseArray, 'obstacle_positions', self.obstacle_callback, 10)

        # Initial state
        self.position = [0.0, 0.0]
        self.robot_x_dir = [1.0, 0.0]  # Assume robot starts facing x direction
        self.heading = 0.0

        # Fixed target and obstacle positions
        self.target_position = [1.0, 1.0]
        self.obstacles = [
            [1.0, 0.0],
            [0.0, 1.0],
        ]

        self.get_logger().info(f"Potential field controller initialized.")
        self.timer = self.create_timer(0.032, self.control_loop)
        self.iteration = 0

    def publish(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.publisher.publish(msg)

    def obstacle_callback(self, msg: PoseArray):
        new_obstacles = []
        for pose in msg.poses:
            x = pose.position.x
            y = pose.position.y
            new_obstacles.append([x, y])
        self.obstacles = new_obstacles

    def robot_status_callback(self, msg: Pose):
        # Update position from Pose message
        self.position = [msg.position.x, msg.position.y]

        # Extract orientation quaternion
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        # Compute yaw
        yaw = yaw_from_quaternion(qx, qy, qz, qw)

        # Convert yaw to 2D direction vector
        self.robot_x_dir = yaw_to_direction_vector(yaw)

        # Optional: update heading (if needed elsewhere)
        self.heading = yaw

    def attractive_force(self):
        """Calculate attractive force towards target."""
        relative_pos_vector = combine_vectors(self.target_position, self.position, '-')
        distance = math.sqrt(relative_pos_vector[0]**2 + relative_pos_vector[1]**2)
        Kv_inst = Kv*distance
        v_target = scalar_multiply(Kv_inst + Kvp, combine_vectors(self.target_position, self.position, '-')) 
             
        return [v_target[0], v_target[1]]
    
    def repulsive_forces(self):
        """Calculate attractive force towards target."""
        rep_forces = []
        for obstacle in self.obstacles: 
            relative_pos_vector = combine_vectors(self.position, obstacle, '-')
            distance = math.sqrt(relative_pos_vector[0]**2 + relative_pos_vector[1]**2)
            Ko_inst = Ko/distance
            v_obstacle = scalar_multiply(Ko_inst, unit_vector(relative_pos_vector)) 
            rep_forces.append(v_obstacle)
             
        return rep_forces


    def reached(self):
        # Check if target is reached
        distance_to_target = math.sqrt((self.position[0] - self.target_position[0])**2 + 
                                        (self.position[1] - self.target_position[1])**2)
            
        if distance_to_target < target_THRESHOLD:
            print("target reached!")
            
            # Move target
            print('target and obstacle are moved!')
            self.target_position = [random.uniform(0, 2), random.uniform(0, 2)]
            self.obstacles = [
                [random.uniform(0, 2), random.uniform(0, 2)],
                [random.uniform(0, 2), random.uniform(0, 2)],
            ]
            #self.set_wheel_velocities(0, 0)  # Stop the robot




    def control_loop(self):

        # Calculate forces in global frame
        attr_force = self.attractive_force()
        rep_forces = self.repulsive_forces()

        # Combine forces
        force_sum = []
        force_sum_rep = []

        force_sum.append(attr_force)
        for item in rep_forces: force_sum_rep.append(item)
        force_sum += force_sum_rep

        rep_force = [sum(v[0] for v in force_sum_rep), sum(v[1] for v in force_sum_rep)]
        total_force = [sum(v[0] for v in force_sum), sum(v[1] for v in force_sum)]

        # Calculate heading difference btw robot and desired velocity
        self.heading_diff = angle_between_unit_vectors(unit_vector(total_force), self.robot_x_dir)

        # Calculate linear and angular velocities FINAL
        self.field_linear_x = math.sqrt(total_force[0]**2+total_force[1]**2) / 60
        self.field_angular_z = -1 * Ka * self.heading_diff

        self.publish(self.field_linear_x, self.field_angular_z)


        # Debug output
        if self.iteration % 50 == 0:
            print(10*'-')
            print(f'Field linear vel: {self.field_linear_x:.2f}, Field angular vel: {self.field_angular_z:.2f}')

        self.reached()
        self.iteration += 1


    def destroy_node(self):
        self.get_logger().info("Shutting down potential field node.")
        super().destroy_node()
        
# Main function
def main():
    rclpy.init()
    node = PotentialFieldNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
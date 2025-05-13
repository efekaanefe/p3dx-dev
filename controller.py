"""
Potential Field Navigation for E-puck in Webots with Dynamically Added Obstacles
This controller implements a potential field navigation approach for the e-puck robot in Webots.
The robot is attracted to a target point while being repelled by predefined obstacle points.
"""

from controller import Robot, Motor, Supervisor, Node
import math
import random

# TODO 
# 1) radius of influence'ı koda dahil et!


def matrix_multiply(A, B):
    """Multiply two matrices A and B."""
    # Check dimensions
    if len(A[0]) != len(B):
        raise ValueError("Number of columns in A must equal number of rows in B")
    
    result = []
    for i in range(len(A)):  # for each row in A
        row = []
        for j in range(len(B[0])):  # for each column in B
            val = sum(A[i][k] * B[k][j] for k in range(len(B)))
            row.append(val)
        result.append(row)
    
    return result

def scalar_multiply(scalar, vector):
    """Multiply a vector by a scalar."""
    return [scalar * x for x in vector]

def combine_vectors(v1, v2, operation='+'):
    """Add or subtract two vectors element-wise."""
    if len(v1) != len(v2):
        raise ValueError("Vectors must be of same length")
    if operation == '+':
        return [a + b for a, b in zip(v1, v2)]
    elif operation == '-':
        return [a - b for a, b in zip(v1, v2)]
    else:
        raise ValueError("Unsupported operation. Use '+' or '-'.")

def unit_vector(v):
    """Return the unit vector (normalized vector) of the input vector."""
    magnitude = math.sqrt(sum(x**2 for x in v))
    if magnitude == 0:
        raise ValueError("Zero vector has no direction; cannot compute unit vector.")
    return [x / magnitude for x in v]

def angle_between_unit_vectors(u1, u2):
    """Compute the angle in radians between two unit vectors."""
    if len(u1) != len(u2):
        raise ValueError("Vectors must be of same length")
    angle = math.atan2(u2[1], u2[0]) - math.atan2(u1[1], u1[0])
    
    if angle > math.pi : angle -= 2*math.pi
    elif  angle < -1*math.pi : angle += 2*math.pi

    return angle

# Define constants
MAX_SPEED = 6.28  # Max rotational speed (rad/s)
TIMESTEP = 32  # Controller time step in milliseconds

# Potential field parameters
Kv = 5 # Attraction distance-bsaed gain to target
Kvp = 20 # Attractive constant gain
Ko = 0.1
Ka = 0.5
target_THRESHOLD = 0.1  # Distance to target to consider it reached
INFLUENCE_RADIUS = 0.1  # Radius of influence for obstacles

class PotentialFieldEpuck(Supervisor):  # Using Supervisor instead of Robot
    def __init__(self):
        super().__init__()
        
        # Initialize motors
        self.left_motor = self.getDevice("left wheel motor")
        self.right_motor = self.getDevice("right wheel motor")
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        
        # Robot properties
        self.wheel_radius = 0.0205  # in meters
        self.axle_length = 0.052  # distance between wheels in meters
        
        # Position tracking using odometry
        self.position = [0.0, 0.0]  # Start position [x, y]
        self.robot_x_dir = [0.0, 0.0]
        self.heading = 0.0  # Start heading (radians)
        
        # Define target position
        self.target = [1, 1]  # X, Y coordinates
        
        # Define obstacle positions (X, Y coordinates)
        self.obstacles = [
            [0.5, 0.2],
            [1, 0.3],
        ]
  
    def get_real_position(self):
        """Get the real position of the robot from Webots (using Supervisor)."""
        # Get a reference to the robot node
        robot_node = self.getSelf()
        # Get the position from the translation field
        self.position[0], self.position[1] = robot_node.getPosition()[0], robot_node.getPosition()[1]
        
        # Calculate robot heading
        rotation_matrix = robot_node.getOrientation()
        self.robot_x_dir = unit_vector([rotation_matrix[0], rotation_matrix[3]])
        self.heading = angle_between_unit_vectors([1, 0], self.robot_x_dir)
        if self.heading < 0 : self.heading += 2*math.pi


    def attractive_force(self):
        """Calculate attractive force towards target."""
        # Use real position from supervisor if available
        relative_pos_vector = combine_vectors(self.target, self.position, '-')
        distance = math.sqrt(relative_pos_vector[0]**2 + relative_pos_vector[1]**2)
        Kv_inst = Kv*distance
        v_target = scalar_multiply(Kv_inst + Kvp, combine_vectors(self.target, self.position, '-')) 
             
        return [v_target[0], v_target[1]]
    
    def repulsive_forces(self):
        """Calculate attractive force towards target."""
        rep_forces = []
        # Use real position from supervisor if available
        for obstacle in self.obstacles: 
            relative_pos_vector = combine_vectors(self.position, obstacle, '-')
            distance = math.sqrt(relative_pos_vector[0]**2 + relative_pos_vector[1]**2)
            Ko_inst = Ko/distance
            v_obstacle = scalar_multiply(Ko_inst, unit_vector(relative_pos_vector)) 
            rep_forces.append(v_obstacle)
             
        return rep_forces
    
    
    def set_wheel_velocities(self, linear_velocity, angular_velocity):
        """Convert linear and angular velocities to wheel speeds."""
        # Set wheel velocities
        wl = (linear_velocity-(self.axle_length/2)*angular_velocity)/self.wheel_radius
        wr = (linear_velocity+(self.axle_length/2)*angular_velocity)/self.wheel_radius
        vels = [wl,wr]

        if max(vels) > MAX_SPEED - 0.1:
            pos = vels.index(max(vels))
            if pos == 1 : 
                wl = MAX_SPEED*wl/wr
                wr = MAX_SPEED
                
            elif pos == 0: 
                wr = MAX_SPEED*wr/wl
                wl = MAX_SPEED
                
        self.left_motor.setVelocity(wl)
        self.right_motor.setVelocity(wr)

    def reached(self):
                # Check if target is reached
        distance_to_target = math.sqrt((self.position[0] - self.target[0])**2 + 
                                        (self.position[1] - self.target[1])**2)
            
        if distance_to_target < target_THRESHOLD:
            print("target reached!")
            self.set_wheel_velocities(0, 0)  # Stop the robot
    
    def run(self):
        """Main control loop."""
        print(f"Starting navigation to target: ({self.target[0]}, {self.target[1]})")
        #print(f"Avoiding {len(self.obstacles)} obstacles")
        
        iteration = 0
        
        while self.step(TIMESTEP) != -1:
            # Update position and orientation
            self.get_real_position()
            
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

            # Calculate linear and angular velocities 
            linear_vel = math.sqrt(total_force[0]**2+total_force[1]**2)
            linear_vel = linear_vel/60
            angular_vel = -1 * Ka * self.heading_diff
            
            # Set wheel speeds
            self.set_wheel_velocities(linear_vel, angular_vel)
            
            # Debug output
            if iteration % 50 == 0:
                print(10*'-')
                #print(f"Real Position: ({self.position[0]:.2f}, {self.position[1]:.2f})")
                #print(f"Robot Heading: {self.heading:.2f} rad")
                #print(f"Heading Difference: {self.heading_diff:.2f} rad")
                print(f"Attractive force: ({attr_force[0]:.2f}, {attr_force[1]:.2f})")
                print(f"Repulsive force: ({rep_force[0]:.2f}, {rep_force[1]:.2f})")
                print(f"Total force: ({total_force[0]:.2f}, {total_force[1]:.2f})")
                
            
            self.reached()
            iteration += 1
        
# Main function
def main():
    # Create and run the robot
    robot = PotentialFieldEpuck()
    robot.run()

if __name__ == "__main__":
    main()
"""
Potential Field Navigation for E-puck in Webots with Dynamically Added Obstacles
This controller implements a potential field navigation approach for the e-puck robot in Webots.
The robot is attracted to a goal point while being repelled by predefined obstacle points.
"""

from controller import Robot, Motor, Supervisor, Node
import math
import random

# Kodun kinematik drive o field'a göre sürme kısmı ile ilgilenilecek.


# Define constants
MAX_SPEED = 6.28  # Max rotational speed (rad/s)
TIMESTEP = 32  # Controller time step in milliseconds

# Potential field parameters
K_ATTR = 10 # Attraction gain to goal
K_REP = 0.1  # Repulsion gain from obstacles
GOAL_THRESHOLD = 0.1  # Distance to goal to consider it reached
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
        self.heading = 0.0  # Start heading (radians)
        
        # Define goal position
        self.goal = [0.5, 0.5]  # X, Y coordinates
        
        # Define obstacle positions (X, Y coordinates)
        self.obstacles = [
            [0.3, 0.3],
        ]
  
        
    def get_real_position(self):
        """Get the real position of the robot from Webots (using Supervisor)."""
        # Get a reference to the robot node
        robot_node = self.getSelf()
        # Get the position from the translation field
        self.position = robot_node.getPosition()
        return [self.position[0], self.position[1]]  # Return x, y coordinates
    
    def attractive_force(self):
        """Calculate attractive force towards goal."""
        # Use real position from supervisor if available
        real_pos = self.position
        
        dx = self.goal[0] - real_pos[0]
        dy = self.goal[1] - real_pos[1]
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance < GOAL_THRESHOLD:
            return [0, 0]
        
        # Attractive force is proportional to distance
        fx = K_ATTR * dx
        fy = K_ATTR * dy
        
        return [fx, fy]
    
    def repulsive_force(self):
        """Calculate repulsive force from predefined obstacles."""
        # Use real position from supervisor if available
        real_pos = self.position
        
        fx, fy = 0, 0
        
        for obstacle in self.obstacles:
            # Calculate distance to obstacle
            dx = real_pos[0] - obstacle[0]
            dy = real_pos[1] - obstacle[1]
            distance = math.sqrt(dx*dx + dy*dy)
            
            # Apply repulsive force only if obstacle is within influence radius
            if distance < INFLUENCE_RADIUS:
                # Calculate repulsive force magnitude
                # Force increases as robot gets closer to obstacle
                magnitude = K_REP * (1.0/distance - 1.0/INFLUENCE_RADIUS) * (1.0/(distance*distance))
                
                # Direction is away from obstacle
                if distance > 0:  # Avoid division by zero
                    fx += magnitude * dx / distance
                    fy += magnitude * dy / distance
        
        return [fx, fy]
    

    def set_wheel_velocities(self, velocity_right, velocity_left):
        """Convert linear and angular velocities to wheel speeds."""
        # Set wheel velocities
        self.left_motor.setVelocity(velocity_left)
        self.right_motor.setVelocity(velocity_right)
    
    def run(self):
        """Main control loop."""
        print(f"Starting navigation to goal: ({self.goal[0]}, {self.goal[1]})")
        print(f"Avoiding {len(self.obstacles)} obstacles")
        
        iteration = 0
        
        while self.step(TIMESTEP) != -1:
            # Update position based on odometry
            
            # Calculate forces in global frame
            attr_force = self.attractive_force()
            rep_force = self.repulsive_force()
            
            # Combine forces
            total_force_x = attr_force[0] + rep_force[0]
            total_force_y = attr_force[1] + rep_force[1]
            force_magnitude = math.sqrt(total_force_x**2 + total_force_y**2)
            force_norm_x, force_norm_y = (total_force_x/force_magnitude, total_force_y/force_magnitude)
            force_heading = math.atan2(force_norm_x, force_norm_y)

            heading_diff = self.heading-force_heading
            print('Heading diff:',heading_diff)

            # Turn until heading is OK
            while abs(heading_diff) > 0.05:         
                # Set wheel velocities
                self.set_wheel_velocities(0,0.1)

            self.set_wheel_velocities(0.1,0.1)
            
            # Debug output
            if iteration % 50 == 0:
                real_pos = self.position
                print(f"Real Position: ({real_pos[0]:.2f}, {real_pos[1]:.2f})")
                print(f"Heading: {self.heading:.2f} rad")
                print(f"Attractive force: ({attr_force[0]:.2f}, {attr_force[1]:.2f})")
                print(f"Repulsive force: ({rep_force[0]:.2f}, {rep_force[1]:.2f})")
                print(f"Total force: ({total_force_x:.2f}, {total_force_y:.2f})")
            
            # Check if goal is reached
            real_pos = self.position
            distance_to_goal = math.sqrt((real_pos[0] - self.goal[0])**2 + 
                                        (real_pos[1] - self.goal[1])**2)
            
            if distance_to_goal < GOAL_THRESHOLD:
                print("Goal reached!")
                self.set_wheel_velocities(0)  # Stop the robot
                
        
# Main function
def main():
    # Create and run the robot
    robot = PotentialFieldEpuck()
    robot.run()

if __name__ == "__main__":
    main()
from utils import matrix_multiply, scalar_multiply, combine_vectors, unit_vector, angle_between_unit_vectors
from controller import Robot, Motor, Supervisor, Node, Keyboard
import math
import random

# Define constants
MAX_SPEED = 6.28  # Max rotational speed (rad/s)
TIMESTEP = 32 # Controller time step in milliseconds

# Potential field parameters
Kv = 5 # Attraction distance-bsaed gain to target
Kvp = 20 # Attractive constant gain
Ko = 0.1
Ka = 5
target_THRESHOLD = 0.1  # Distance to target to consider it reached
INFLUENCE_RADIUS = 0.1  # Radius of influence for obstacles
K_field, K_key = 1, 1

class PotentialFieldEpuck(Supervisor):  # Using Supervisor instead of Robot
    def __init__(self):
        super().__init__()
        self.left_motor = self.getDevice("left wheel motor")
        self.right_motor = self.getDevice("right wheel motor")
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

        self.keyboard = Keyboard()
        self.keyboard.enable(TIMESTEP)

        self.key_linear_x_increment = 0.1
        self.key_angular_z_increment = 0.1

        self.total_linear_x = 0.0
        self.total_angular_z = 0.0
        self.key_linear_x = 0.0
        self.key_angular_z = 0.0
        self.field_linear_x = 0.0
        self.field_angular_z = 0.0
        
        self.wheel_radius = 0.0205  # in meters
        self.axle_length = 0.052  # distance between wheels in meters
        
        self.position = [0.0, 0.0]  # Start position [x, y]
        self.robot_x_dir = [0.0, 0.0]
        self.heading = 0.0  # Start heading (radians)
        
        # Define target and obtacles position
        #self.target = [1, 1]  # X, Y coordinates
        #self.obstacles = [
        #    [0.5, 0.2],
        #   [1, 0.3],
        #]
        self.obstacles = []

        try:
            self.getFromDef('BALL').remove()
            self.getFromDef('CUBE1').remove()
            self.getFromDef('CUBE2').remove()    
        except:None

        root_node = self.getRoot()
        children_field = root_node.getField('children')
        self.target_position = [1, 1, 0]
        children_field.importMFNodeFromString(-1, 'DEF BALL Solid { translation 1 1 0 children [ Shape { geometry Sphere { radius 0.05 } } ] }')
        self.target_node = self.getFromDef('BALL')

        self.obtacle_position = [1, 0, 0]
        self.obstacles.append(self.obtacle_position)
        children_field.importMFNodeFromString(-1, 'DEF CUBE1 Solid { translation 1 0 0 children [ Shape { geometry Box { size 0.05 0.05 0.05 } } ] }')
        self.obstacle_node1 = self.getFromDef('CUBE1')

        self.obtacle_position = [0, 1, 0]
        self.obstacles.append(self.obtacle_position)
        children_field.importMFNodeFromString(-1, 'DEF CUBE2 Solid { translation 0 1 0 children [ Shape { geometry Box { size 0.05 0.05 0.05 } } ] }')
        self.obstacle_node2 = self.getFromDef('CUBE2')

    def get_user_input(self):
        key = self.keyboard.getKey()
        while key != -1:
            print(f"Key pressed: {key}")
            if key == Keyboard.UP:
                self.key_linear_x += self.key_linear_x_increment
            elif key == Keyboard.DOWN:
                self.key_linear_x -= self.key_linear_x_increment
            elif key == Keyboard.RIGHT:
                self.key_angular_z -= self.key_angular_z_increment
            elif key == Keyboard.LEFT:
                self.key_angular_z += self.key_angular_z_increment
            elif key == ord(' '):  # Space
                self.key_linear_x = 0.0
                self.key_angular_z = 0.0
            else:
                print("Invalid key pressed. Exiting...")
                return False
            key = self.keyboard.getKey()

        return True


    def move(self, node, position:list):
        translation_field = node.getField('translation')
        translation_field.setSFVec3f(position)
    
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
           
        self.target_position = [self.target_node.getPosition()[0], self.target_node.getPosition()[1]]
        #('Target position: %f %f\n' %(self.target_position[0], self.target_position[1]))

        self.obtacle_position1 = [self.obstacle_node1.getPosition()[0], self.obstacle_node1.getPosition()[1]]
        self.obstacles[0] = self.obtacle_position1

        self.obtacle_position2 = [self.obstacle_node2.getPosition()[0], self.obstacle_node2.getPosition()[1]]
        self.obstacles[1] = self.obtacle_position2

    def attractive_force(self):
        """Calculate attractive force towards target."""
        # Use real position from supervisor if available
        relative_pos_vector = combine_vectors(self.target_position, self.position, '-')
        distance = math.sqrt(relative_pos_vector[0]**2 + relative_pos_vector[1]**2)
        Kv_inst = Kv*distance
        v_target = scalar_multiply(Kv_inst + Kvp, combine_vectors(self.target_position, self.position, '-')) 
             
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
        distance_to_target = math.sqrt((self.position[0] - self.target_position[0])**2 + 
                                        (self.position[1] - self.target_position[1])**2)
            
        if distance_to_target < target_THRESHOLD:
            print("target reached!")
            
            # Move target
            print('target and obstacle are moved!')
            self.move(self.target_node, [random.uniform(0,2), random.uniform(0,2), 0])
            self.move(self.obstacle_node1, [random.uniform(0,2), random.uniform(0,2), 0])
            self.move(self.obstacle_node2, [random.uniform(0,2), random.uniform(0,2), 0])
            
            #self.set_wheel_velocities(0, 0)  # Stop the robot
    
    def run(self):
        """Main control loop."""
        print(f"Starting navigation to target: ({self.target_position[0]}, {self.target_position[1]})")
        #print(f"Avoiding {len(self.obstacles)} obstacles")
        
        iteration = 0
        
        while self.step(TIMESTEP) != -1:
            # Update position and orientation
            self.get_real_position()
            self.get_user_input()
            
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
            self.field_linear_x = math.sqrt(total_force[0]**2+total_force[1]**2) / 60
            self.field_angular_z = -1 * Ka * self.heading_diff
            
            self.total_linear_x = K_field * self.field_linear_x + K_key * self.key_linear_x
            self.total_angular_z = K_field * self.field_angular_z + K_key * self.key_angular_z
            
            # Set wheel speeds
            self.set_wheel_velocities(self.total_linear_x, self.total_angular_z)
            
            # Debug output
            if iteration % 50 == 0:
                print(10*'-')
                #print(f"Real Position: ({self.position[0]:.2f}, {self.position[1]:.2f})")
                #print(f"Robot Heading: {self.heading:.2f} rad")
                #print(f"Heading Difference: {self.heading_diff:.2f} rad")
                #print(f"Attractive force: ({attr_force[0]:.2f}, {attr_force[1]:.2f})")
                #print(f"Repulsive force: ({rep_force[0]:.2f}, {rep_force[1]:.2f})")
                #print(f"Total force: ({total_force[0]:.2f}, {total_force[1]:.2f})")
                print(f'Key linear vel: {self.key_linear_x:.2f}, Key angular vel: {self.key_angular_z:.2f}')
                print(f'Field linear vel: {self.field_linear_x:.2f}, Field angular vel: {self.field_angular_z:.2f}')
                print(f'Total linear vel: {self.total_linear_x:.2f}, Total angular vel: {self.total_angular_z:.2f}')
                print(f'Obtstacle position: {self.obstacles[-1][0]:.2f}, {self.obstacles[-1][1]:.2f}')
            
            self.reached()
            iteration += 1
        
# Main function
def main():
    # Create and run the robot
    robot = PotentialFieldEpuck()
    robot.run()

if __name__ == "__main__":
    main()
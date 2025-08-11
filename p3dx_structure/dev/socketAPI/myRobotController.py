import socket
import json
import time

class SimpleRobotController:
    def __init__(self, host='192.168.68.65', port=9090, is_simulation=False):
        """
        Simple robot controller for sending velocity commands
        
        Args:
            host: Robot IP address
            port: Robot port
            is_simulation: If True, just prints commands instead of sending
        """
        self.is_simulation = is_simulation
        self.linear_x = 0.0
        self.angular_z = 0.0
        
        if not self.is_simulation:
            print(f"Connecting to robot at {host}:{port}...")
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((host, port))
            print("Connected successfully!")
        else:
            print("Running in simulation mode")
    
    def set_velocity(self, linear_x, angular_z):
        """
        Set robot velocity
        
        Args:
            linear_x: Forward/backward speed (m/s)
            angular_z: Rotation speed (rad/s)
        """
        self.linear_x = linear_x
        self.angular_z = angular_z
        self._send_command()
    
    def _send_command(self):
        """Send current velocity command to robot"""
        cmd = {"linear_x": self.linear_x, "angular_z": self.angular_z}
        
        if self.is_simulation:
            print(f"Simulated: linear_x={cmd['linear_x']:.2f}, angular_z={cmd['angular_z']:.2f}")
        else:
            try:
                self.sock.sendall(json.dumps(cmd).encode('utf-8'))
                feedback = self.sock.recv(1024).decode()
                print(f"Robot feedback: {feedback}")
            except Exception as e:
                print(f"Failed to send command: {e}")
    
    def stop(self):
        """Stop the robot"""
        self.set_velocity(0.0, 0.0)
        print("Robot stopped")
    
    def move_forward(self, speed=0.5):
        """Move forward at given speed"""
        self.set_velocity(speed, 0.0)
    
    def move_backward(self, speed=0.5):
        """Move backward at given speed"""
        self.set_velocity(-speed, 0.0)
    
    def turn_left(self, speed=0.5):
        """Turn left at given angular speed"""
        self.set_velocity(0.0, speed)
    
    def turn_right(self, speed=0.5):
        """Turn right at given angular speed"""
        self.set_velocity(0.0, -speed)
    
    def close(self):
        """Close connection to robot"""
        self.stop()
        if not self.is_simulation:
            self.sock.close()
            print("Connection closed")


# Example usage
if __name__ == "__main__":
    # For real robot
    # robot = SimpleRobotController(host='192.168.68.65', port=9090)

    # For simulation (testing)
    robot = SimpleRobotController(is_simulation=False)

    # Send commands
    robot.set_velocity(0.5, 0.0)  # Move forward at 0.5 m/s
    robot.turn_left(0.3)          # Turn left at 0.3 rad/s
    robot.stop()                  # Stop the robot

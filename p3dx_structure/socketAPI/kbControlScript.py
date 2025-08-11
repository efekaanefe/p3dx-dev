import socket
import json
import time
import threading
import sys
import os
from myRobotController import SimpleRobotController

# For Windows
try:
    import msvcrt
except ImportError:
    msvcrt = None

# For Unix/Linux/Mac
try:
    import termios
    import tty
except ImportError:
    termios = None
    tty = None

class TerminalKeyboardController:
    def __init__(self, robot_host='192.168.68.65', robot_port=9090, is_simulation=True):
        self.robot = SimpleRobotController(robot_host, robot_port, is_simulation)
        self.running = True
        self.linear_speed = 0.8   # Default linear speed
        self.angular_speed = 0.4  # Default angular speed
        self.speed_increment = 0.05
        
        # Current velocities
        self.current_linear = 0.0
        self.current_angular = 0.0
        
        # Setup terminal for key reading
        if msvcrt:  # Windows
            self.get_char = self._get_char_windows
        elif termios:  # Unix/Linux/Mac
            self.get_char = self._get_char_unix
            self.old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        else:
            raise Exception("Unsupported platform for keyboard input")
    
    def _get_char_windows(self):
        """Get single character input on Windows"""
        return msvcrt.getch().decode('utf-8')
    
    def _get_char_unix(self):
        """Get single character input on Unix/Linux/Mac"""
        return sys.stdin.read(1)
    
    def print_status(self):
        """Print current status"""
        os.system('clear' if os.name == 'posix' else 'cls')
        print("=" * 60)
        print("          ROBOT KEYBOARD CONTROLLER")
        print("=" * 60)
        print()
        print("CONTROLS:")
        print("  W/S     - Forward/Backward")
        print("  A/D     - Turn Left/Right")
        print("  +/-     - Increase/Decrease Speed")
        print("  SPACE   - Stop")
        print("  Q       - Quit")
        print()
        print(f"CURRENT SETTINGS:")
        print(f"  Linear Speed:  {self.linear_speed:.2f} m/s")
        print(f"  Angular Speed: {self.angular_speed:.2f} rad/s")
        print()
        print(f"CURRENT MOTION:")
        print(f"  Linear:  {self.current_linear:+.2f} m/s")
        print(f"  Angular: {self.current_angular:+.2f} rad/s")
        print()
        if self.current_linear != 0 or self.current_angular != 0:
            status = []
            if self.current_linear > 0:
                status.append("FORWARD")
            elif self.current_linear < 0:
                status.append("BACKWARD")
            if self.current_angular > 0:
                status.append("TURNING LEFT")
            elif self.current_angular < 0:
                status.append("TURNING RIGHT")
            print(f"STATUS: {' + '.join(status)}")
        else:
            print("STATUS: STOPPED")
        print()
        print("Press any key to control robot...")
    
    def update_velocities(self, linear, angular):
        """Update robot velocities and display"""
        self.current_linear = linear
        self.current_angular = angular
        self.robot.set_velocity(linear, angular)
        self.print_status()
    
    def run(self):
        """Main control loop"""
        print("Starting keyboard controller...")
        time.sleep(1)
        
        self.print_status()
        
        try:
            while self.running:
                char = self.get_char().lower()
                
                if char == 'q':
                    break
                elif char == 'w':
                    self.update_velocities(self.linear_speed, 0)
                elif char == 's':
                    self.update_velocities(-self.linear_speed, 0)
                elif char == 'a':
                    self.update_velocities(0, self.angular_speed)
                elif char == 'd':
                    self.update_velocities(0, -self.angular_speed)
                elif char == ' ':
                    self.update_velocities(0, 0)
                elif char == '+' or char == '=':
                    self.linear_speed = min(self.linear_speed + self.speed_increment, 2.0)
                    self.angular_speed = min(self.angular_speed + self.speed_increment, 2.0)
                    self.print_status()
                elif char == '-' or char == '_':
                    self.linear_speed = max(self.linear_speed - self.speed_increment, 0.1)
                    self.angular_speed = max(self.angular_speed - self.speed_increment, 0.1)
                    self.print_status()
                
        except KeyboardInterrupt:
            pass
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up and restore terminal"""
        print("\nStopping robot...")
        self.robot.stop()
        self.robot.close()
        
        # Restore terminal settings on Unix
        if termios and hasattr(self, 'old_settings'):
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        
        print("Controller stopped. Goodbye!")

def main():
    print("Robot Keyboard Controller")
    print("=" * 25)
    
    # Configuration
    use_simulation = input("Use simulation mode? (y/n): ").lower().startswith('y')
    
    if not use_simulation:
        host = input("Robot IP (press Enter for 192.168.68.65): ").strip()
        if not host:
            host = '192.168.68.65'
        port = input("Robot port (press Enter for 9090): ").strip()
        if not port:
            port = 9090
        else:
            port = int(port)
    else:
        host = '192.168.68.65'
        port = 9090
    
    # Start controller
    controller = TerminalKeyboardController(host, port, use_simulation)
    controller.run()

if __name__ == "__main__":
    main()
#!/usr/bin/env python3
import rospy
import sys
import termios
import tty 
import select
from geometry_msgs.msg import Twist

class Computer:
    def __init__(self, simulation=True):
        # Initialize ROS node
        rospy.init_node('computer_node', anonymous=True)

        # Create publisher to /RosAria/cmd_vel
        if simulation:
            self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
            #self.pub = rospy.Publisher('/pioneer/cmd_vel', Twist, queue_size=10)
        else:
            self.pub = rospy.Publisher('/pioneer/cmd_vel', Twist, queue_size=10)
            #self.pub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=10)

        self.robot_linear_x  = 0.0
        self.robot_angular_z = 0.0

        self.robot_linear_x_increment = 0.1
        self.robot_angular_z_increment = 0.1

        # Velocity limits
        self.max_linear  = 2.0
        self.max_angular = 2.0

        # Wait for publisher to connect to subscribers
        rospy.sleep(1)

    def move(self):
        self.apply_velocity_limits()
        twist = Twist()
        twist.linear.x = self.robot_linear_x    
        twist.angular.z = self.robot_angular_z
        print(f"Linear: {self.robot_linear_x:.2f}, Angular: {self.robot_angular_z:.2f}")
        self.pub.publish(twist)

    def get_key(self, timeout=0.1):  # 100 ms
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            rlist, _, _ = select.select([sys.stdin], [], [], timeout)
            if rlist:
                return sys.stdin.read(1)
            else:
                return None
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def apply_velocity_limits(self):
        # Apply velocity limits
        if self.robot_linear_x > self.max_linear:
            self.robot_linear_x = self.max_linear
            print("Max linear velocity reached")
        elif self.robot_linear_x < -self.max_linear:
            print("Min linear velocity reached")
            self.robot_linear_x = -self.max_linear

        if self.robot_angular_z  > self.max_angular:
            print("Max angular velocity reached")
            self.robot_angular_z  = self.max_angular
        elif self.robot_angular_z  < -self.max_angular:
            print("Min angular velocity reached")
            self.robot_angular_z  = -self.max_angular

        return True

    def get_user_input(self):    
        key = self.get_key()
        if key is None:
            return True  # No input, continue

        if key == '\x1b':  # Escape sequence
            seq = sys.stdin.read(2)
            if seq == '[A':  # Up
                self.robot_linear_x += self.robot_linear_x_increment
            elif seq == '[B':  # Down
                self.robot_linear_x -= self.robot_linear_x_increment
            elif seq == '[C':  # Right
                self.robot_angular_z -= self.robot_angular_z_increment
            elif seq == '[D':  # Left
                self.robot_angular_z += self.robot_angular_z_increment

        elif key == ' ':  # Space to stop
            self.robot_linear_x = 0.0
            self.robot_angular_z = 0.0

        else:
            print("invalid key pressed, exiting..")
            return False
    
        return True

def main():
    simulation = False    
    turtle = Computer(simulation=simulation)
    rate = rospy.Rate(10)  # 10 Hz loop

    try:
        while not rospy.is_shutdown():
            if not turtle.get_user_input():
                break
            turtle.move()
            rate.sleep()

    except KeyboardInterrupt:
        print("Shutting down")
        turtle.robot_linear_x = 0.0
        turtle.robot_angular_z = 0.0
        turtle.move()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print("ROS Interrupt Exception")
        exit(0)

#!/usr/bin/env python3

import sys
import termios
import tty
import select
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller')
        self.publisher_ = self.create_publisher(Twist, 'keyboard_vel', 10)
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.linear_increment = 0.1
        self.angular_increment = 0.1
        self.max_linear = 2.0
        self.max_angular = 2.0
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Use arrow keys to move. Space to stop. Any other key to quit.")

    def get_key(self, timeout=0.1):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            rlist, _, _ = select.select([sys.stdin], [], [], timeout)
            if rlist:
                return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return None

    def apply_limits(self):
        self.linear_x = max(-self.max_linear, min(self.linear_x, self.max_linear))
        self.angular_z = max(-self.max_angular, min(self.angular_z, self.max_angular))

    def timer_callback(self):
        key = self.get_key()
        if key is None:
            return

        if key == '\x1b':
            seq = sys.stdin.read(2)
            if seq == '[A':  # Up
                self.linear_x += self.linear_increment
            elif seq == '[B':  # Down
                self.linear_x -= self.linear_increment
            elif seq == '[C':  # Right
                self.angular_z -= self.angular_increment
            elif seq == '[D':  # Left
                self.angular_z += self.angular_increment
        elif key == ' ':
            self.linear_x = 0.0
            self.angular_z = 0.0
        else:
            self.get_logger().info("Invalid key, exiting.")
            rclpy.shutdown()
            return

        self.apply_limits()

        msg = Twist()
        msg.linear.x = self.linear_x
        msg.angular.z = self.angular_z
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published Twist: linear_x={self.linear_x}, angular_z={self.angular_z}")


def main():
    rclpy.init()
    node = KeyboardController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        msg = Twist()
        node.publisher_.publish(msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

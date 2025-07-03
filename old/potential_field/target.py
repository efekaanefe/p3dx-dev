#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, Pose2D
import threading


class TargetController(Node):
    def __init__(self):
        super().__init__('target_controller')
        self.publisher_ = self.create_publisher(Twist, 'target_vel', 10)

        # Subscriptions
        self.create_subscription(Pose2D, 'robot_pose', self.status_callback, 10)
        self.create_subscription(Point, 'target_pose', self.target_callback, 10)

        # Timer for velocity updates
        self.timer = self.create_timer(1.0, self.timer_callback)

        # Robot state
        self.curr_loc = [0.0, 0.0]
        self.curr_angle = 0.0
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.target_coor = None
        self.goal_reached = True

        # Limits
        self.max_linear = 0.2
        self.max_angular = 0.1

        # Start input thread
        self.input_thread = threading.Thread(target=self.input_loop, daemon=True)
        self.input_thread.start()

    def status_callback(self, msg: Pose2D):
        self.curr_loc = [msg.x, msg.y]
        self.curr_angle = msg.theta

    def target_callback(self, msg: Point):
        self.target_coor = [msg.x, msg.y]
        self.goal_reached = False
        self.get_logger().info(f"New target received from topic: ({msg.x:.2f}, {msg.y:.2f})")

    def input_loop(self):
        while rclpy.ok():
            try:
                user_input = input("Enter target as x,y (or 'q' to quit): ").strip()
                if user_input.lower() in ['q', 'quit', 'exit']:
                    print("Exiting input loop.")
                    break
                if not user_input:
                    continue
                x_str, y_str = user_input.split(",")
                x = float(x_str)
                y = float(y_str)
                self.target_coor = [x, y]
                self.goal_reached = False
                self.get_logger().info(f"New target set from input: ({x}, {y})")
            except Exception as e:
                self.get_logger().warn(f"Invalid input: {e}")

    def apply_limits(self):
        self.linear_x = max(-self.max_linear, min(self.linear_x, self.max_linear))
        self.angular_z = max(-self.max_angular, min(self.angular_z, self.max_angular))

    def compute_speed(self):
        if self.target_coor is None or self.goal_reached:
            self.linear_x = 0.0
            self.angular_z = 0.0
            return

        dx = self.target_coor[0] - self.curr_loc[0]
        dy = self.target_coor[1] - self.curr_loc[1]
        distance = math.hypot(dx, dy)

        goal_tolerance = 0.2
        angle_tolerance = 0.05

        if distance < goal_tolerance:
            print("aaa")
            self.linear_x = 0.0
            self.angular_z = 0.0
            self.goal_reached = True
            self.get_logger().info("Goal reached!")
            return

        target_angle = math.atan2(dy, dx)
        angle_diff = (target_angle - self.curr_angle + math.pi) % (2 * math.pi) - math.pi

        print(angle_diff)

        if abs(angle_diff) > angle_tolerance:
            self.linear_x = 0.0
            self.angular_z = self.max_angular
        else:
            self.linear_x = self.max_linear
            self.angular_z = 0.0

    def timer_callback(self):
        self.compute_speed()
        self.apply_limits()

        msg = Twist()
        msg.linear.x = self.linear_x
        msg.angular.z = self.angular_z
        self.publisher_.publish(msg)

        # Optional debug log
        #self.get_logger().info(f"Published: lin={self.linear_x:.2f}, ang={self.angular_z:.2f}")


def main():
    rclpy.init()
    node = TargetController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down controller...")
        stop_msg = Twist()
        node.publisher_.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

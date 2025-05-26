#!/usr/bin/env python3
"""
Özellikler:
- Socket: Rosaria ile iletişim kurduğumuz nokta. Hız bilgisi yollayabiliyor,
          status bilgisi alabiliyoruz
- ROS2: kalan her parça ile iletişim kurduğumuz nokta. Yani "lidar (potential field)",
          "camera (aruco)" ve "keyboard". Üçü için de ayrı topic'e subscribe oluyoruz

- is_simulation: Simulation olarak seçim yapabiliyoruz, eğer simulation seçersek
                 rb1_ros1'in ve socket'in çalıştırılmasına gerek kalmıyor.
- self.curr_mode_func: buna bir fonksiyon veriyoruz, bu fonksiyon subscribe olduğumuz
                topiclerden gelen hız verisini belli bir stratejiye göre birleştiren
                fonksiyonlardan biri olmalı. Yani potential field'ın son noktası burada

Notlar:
# linear velocity: m/s
# angular velocity: rad/s
"""



import socket
import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

KEYBOARD = 0
ARUCO = 1
OBSTACLE = 2

# TODO: fine-tune
MAX_LINEAR_SPEED = 6.28
MAX_ANGULAR_SPEED = 1.0

K_field, K_key = 1, 1
KEY_CHANGE = 0.1 # keyboard'da basılan bir tuşun keyboard'a ait hızı ne kadar etkileyeceği

class RobotController(Node):
    def __init__(self,is_simulation=False, host='192.168.1.100', port=9090):
        super().__init__('robot_controller')
        self.is_simulation = is_simulation

        # subscribe
        self.subscriber_keyboard = self.create_subscription(Twist,'keyboard_vel',self.keyboard_vel_callback,10)
        self.subscriber_aruco = self.create_subscription(Twist,'aruco_vel',self.aruco_vel_callback,10)
        self.subscriber_obstacle = self.create_subscription(Twist, 'obs_avoid_vel', self.obstacle_vel_callback, 10)
        self.status_publisher = self.create_publisher(Twist, 'robot_status', 10)

        self.linear_x = 0.0
        self.angular_z = 0.0

        self.source_velocities = {
            KEYBOARD: (0.0, 0.0),
            ARUCO: (0.0, 0.0),
            OBSTACLE: (0.0, 0.0)
        }

        # set the current mode here, can ignore certain sources or try different strategies
        self.curr_mode_func = self.mode_potential_field

        # socket stuff
        if not is_simulation:
            self.host = host
            self.port = port
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            try:
                self.sock.connect((self.host, self.port))
                self.get_logger().info(f"Connected to robot at {self.host}:{self.port}")
            except Exception as e:
                self.get_logger().error(f"Connection failed: {e}")
        else:
            self.curr_loc = [0.0,0.0]
            self.create_timer(1.0, self.update_location_artificially)

        self.create_timer(2.0, self.request_status)  # call every 2 seconds

    def update_location_artificially(self):
        self.curr_loc[0] = self.curr_loc[0] + self.linear_x * math.cos(self.angular_z)
        self.curr_loc[1] = self.curr_loc[1] + self.linear_x * math.sin(self.angular_z)

    def send_curr_vel(self):
        cmd = {
            "linear_x": self.linear_x,
            "angular_z": self.angular_z,
        }
        if not self.is_simulation:
            try:
                self.sock.sendall(json.dumps(cmd).encode('utf-8'))
                self.get_logger().info(f"Sent command: {cmd}")
                feedback = self.sock.recv(1024).decode()
                self.get_logger().info(f"Feedback: {feedback}")
            except Exception as e:
                self.get_logger().error(f"Failed to send command: {e}")
        else:
            print(f"""Current velocity is: {cmd["linear_x"]}, {cmd["angular_z"]} """)

    def destroy_node(self):
        self.sock.close()
        self.get_logger().info("Shutting down core ros2 node.")
        super().destroy_node()

    def set_velocity(self, linear_x, angular_z):
        """
        clamps the velocity before setting
        """
        self.linear_x = max(min(linear_x, MAX_LINEAR_SPEED), -MAX_LINEAR_SPEED)
        self.angular_z = max(min(angular_z, MAX_ANGULAR_SPEED), -MAX_ANGULAR_SPEED)

    def mode_base(self):
        total_linear = sum(v[0] for v in self.source_velocities.values())
        total_angular = sum(v[1] for v in self.source_velocities.values())
        self.set_velocity(total_linear, total_angular)

    def mode_potential_field(self):
        # TODO: şu an için arucoyu direkt ekliyoruz, onun için ek bir potential field burada yapılabilir
        field_linear = sum(v[0] for k, v in self.source_velocities.items() if k != KEYBOARD)
        field_angular = sum(v[1] for k, v in self.source_velocities.items() if k != KEYBOARD)
        total_linear = K_field * field_linear + K_key * self.source_velocities[KEYBOARD][0]
        total_angular = K_field * field_angular + K_key * self.source_velocities[KEYBOARD][1]

        self.set_velocity(total_linear, total_angular)

    def base_callback(self, linear_x, angular_z, source):
        self.source_velocities[source] = (linear_x, angular_z)
        self.curr_mode_func()
        self.send_curr_vel()

    def keyboard_vel_callback(self, msg):
        self.get_logger().info("Received keyboard velocity")
        self.base_callback(msg.linear.x, msg.angular.z, KEYBOARD)

    def aruco_vel_callback(self, msg):
        self.get_logger().info("Received aruco velocity")
        self.base_callback(msg.linear.x, msg.angular.z, ARUCO)

    def obstacle_vel_callback(self, msg):
        self.get_logger().info("Received potential field velocity")
        self.base_callback(msg.linear.x, msg.angular.z, OBSTACLE)

    def request_status(self):
        """
        Publishes to the topic periodically
        """
        if not self.is_simulation:
            try:
                status_request = {"get_status": True}
                self.sock.sendall(json.dumps(status_request).encode('utf-8'))
                response = self.sock.recv(1024).decode()
                data = json.loads(response)

                # Only publish if response is valid
                if data.get("status") == "ok":
                    twist_msg = Twist()
                    twist_msg.linear.x = data.get("linear_x", 0.0)
                    twist_msg.angular.z = data.get("angular_z", 0.0)
                    self.status_publisher.publish(twist_msg)
                    self.get_logger().info(
                        f"Published robot status: linear_x={twist_msg.linear.x}, angular_z={twist_msg.angular.z}")
                else:
                    self.get_logger().warn("Invalid status response received.")
            except Exception as e:
                self.get_logger().error(f"Failed to request or publish status: {e}")
        else:
            twist_msg = Twist()
            twist_msg.linear.x = self.curr_loc[0]
            twist_msg.angular.z = self.curr_loc[1]
            self.status_publisher.publish(twist_msg)
            self.get_logger().info(
                f"Published robot status: linear_x={twist_msg.linear.x}, angular_z={twist_msg.angular.z}")


def main():
    is_simulation = True if input("Do you want this to be a simulation? (y/n) ") == "y" else False
    rclpy.init()
    node = RobotController(is_simulation=is_simulation)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

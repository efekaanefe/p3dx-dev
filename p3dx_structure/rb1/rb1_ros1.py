#!/usr/bin/env python3

"""
1. Bu script, bilgisayar tarafından gönderilen twist mesajlarını alarak rosaria topic'ine gönderir ve robotun hız kontrolünü sağlar.
2. Direkt olarak robottan çıkan USB kablosuna bağlı olan bilgisayara (veya Raspberry'ye) takılmalıdır.
3. Çalıştırmadan önce
rosrun rosaria RosAria
komudunu yeni bir terminalde çalıştırarak robot bağlantısını sağlayın.
4. Eğer uzaktan çalıştıracaksanız SSH ile bağlantı yapın.
"""

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


# for socket
import socket
import threading
import json
from tf_transformations import euler_from_quaternion


# small helper
def get_yaw(orientation):
    q = [orientation.x, orientation.y, orientation.z, orientation.w]
    _, _, yaw = euler_from_quaternion(q)
    return yaw


class Pioneer:
    def __init__(self, simulation=True):
        # Initialize ROS node
        rospy.init_node('pioneer_node', anonymous=True)

        # Create publisher based on mode
        if simulation:
            self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        else:
            self.pub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=10)
            rospy.Subscriber('/RosAria/pose', Odometry, self.odom_callback)
            self.latest_odom = None

        # Subscribe to input Twist messages
        rospy.Subscriber('/pioneer/cmd_vel', Twist, self.twist_callback)

        self.robot_linear_x  = 0.0
        self.robot_angular_z = 0.0

        # Velocity limits
        self.max_linear  = 2.0
        self.max_angular = 2.0

        # Effect of obstacles to the robot's velocity
        self.obstacles_linear_x = 0.0
        self.obstacles_angular_z = 0.0

        # Wait for publisher to connect to subscribers
        rospy.sleep(1)

        # Initialize the latest twist message
        self.latest_twist = Twist()

        # Start periodic timer at 10 Hz
        rospy.Timer(rospy.Duration(0.1), self.process_twist)
        rospy.loginfo("Pioneer node initialized and timer started.")

        rospy.on_shutdown(self.stop_robot)

        # for socket
        self.host = '0.0.0.0'  # Listen on all interfaces
        self.port = 9090  # TODO: choose any open port

        # Start the socket server in a new thread
        self.socket_thread = threading.Thread(target=self.start_socket_server)
        self.socket_thread.daemon = True
        self.socket_thread.start()



    def start_socket_server(self):
        """
        receives the velocity
        sends the status (position etc.) when requested
        :return:
        """
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((self.host, self.port))
            s.listen(1)
            rospy.loginfo(f"Socket server listening on {self.host}:{self.port}")
            while not rospy.is_shutdown():
                conn, addr = s.accept()
                rospy.loginfo(f"Client connected from {addr}")
                with conn:
                    while not rospy.is_shutdown():
                        data = conn.recv(1024)
                        if not data:
                            break
                        try:
                            msg = json.loads(data.decode('utf-8'))

                            if msg.get("get_status"): # can get the whole status if requested
                                if self.latest_odom:
                                    vel = self.latest_odom.twist.twist
                                    pose = self.latest_odom.pose.pose

                                    status_info = {
                                        "status": "ok",
                                        "pose": {
                                            "x": pose.position.x,
                                            "y": pose.position.y,
                                            "theta": get_yaw(pose.orientation)
                                        },
                                        "velocity": {
                                            "linear_x": vel.linear.x,
                                            "angular_z": vel.angular.z
                                        }
                                    }
                                else:
                                    status_info = {"status": "no_odom_data"}

                                conn.sendall(json.dumps(status_info).encode('utf-8'))

                            else:
                                # Handle incoming velocity command
                                twist = Twist()
                                twist.linear.x = msg.get('linear_x', 0.0)
                                twist.angular.z = msg.get('angular_z', 0.0)
                                self.latest_twist = twist

                                # Send ack
                                feedback = {"status": "command_received", "received": msg}
                                conn.sendall(json.dumps(feedback).encode('utf-8'))

                        except Exception as e:
                            rospy.logerr(f"Error handling socket message: {e}")
                            conn.sendall(b'{"status": "error"}')

    def odom_callback(self, msg):
        self.latest_odom = msg

    def twist_callback(self, msg):
        self.latest_twist = msg

    def apply_velocity_limits(self):
        rospy.logdebug("Applying velocity limits before sending twist")
        if self.robot_linear_x > self.max_linear:
            self.robot_linear_x = self.max_linear
            rospy.logdebug("Max linear velocity reached")
        elif self.robot_linear_x < -self.max_linear:
            rospy.logdebug("Min linear velocity reached")
            self.robot_linear_x = -self.max_linear

        if self.robot_angular_z  > self.max_angular:
            rospy.logdebug("Max angular velocity reached")
            self.robot_angular_z  = self.max_angular
        elif self.robot_angular_z  < -self.max_angular:
            rospy.logdebug("Min angular velocity reached")
            self.robot_angular_z  = -self.max_angular        

    def send_twist(self):
        # publish the latest twist message to the robot or simulation
        twist = Twist()

        self.robot_linear_x  = self.latest_twist.linear.x  + self.obstacles_linear_x
        self.robot_angular_z = self.latest_twist.angular.z + self.obstacles_angular_z   
        print("new twist is calculated")

        self.apply_velocity_limits()
        
        twist.linear.x  = self.robot_linear_x
        twist.angular.z = self.robot_angular_z 

        # Publish to output topic
        self.pub.publish(twist)


    def get_pose_from_rosaria(self):
        # Get pose from RosAria
        # This is a placeholder for actual implementation
        # You would typically use a service or topic to get this data
        pass

    def process_twist(self, event):
        rospy.loginfo(" Received Twist message:")
        rospy.loginfo(" Linear:  x=%.2f, y=%.2f, z=%.2f", self.latest_twist.linear.x, self.latest_twist.linear.y, self.latest_twist.linear.z)
        rospy.loginfo(" Angular: x=%.2f, y=%.2f, z=%.2f", self.latest_twist.angular.x, self.latest_twist.angular.y, self.latest_twist.angular.z)

        self.send_twist()
    
    def stop_robot(self):
        rospy.loginfo("Stopping robot...")
        stop_msg = Twist()  # Zero velocities
        for i in range(10):
            self.pub.publish(stop_msg)
            rospy.sleep(0.05)  # 50ms delay between messages


    
def main():
    simulation = True    
    pioneer = Pioneer(simulation=simulation)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

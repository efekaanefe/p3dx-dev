#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist


class Pioneer:
    def __init__(self, simulation=True):
        # Initialize ROS node
        rospy.init_node('pioneer_node', anonymous=True)

        # Create publisher based on mode
        if simulation:
            self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        else:
            self.pub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=10)

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

    def process_lidar(self):
        # Process LIDAR data here
        self.obstacles_linear_x  = 0.0
        self.obstacles_angular_z = 0.0
        print("LIDAR data processed")

    def get_pose_from_rosaria(self):
        # Get pose from RosAria
        # This is a placeholder for actual implementation
        # You would typically use a service or topic to get this data
        pass

    def process_twist(self, event):        
        rospy.loginfo(" Received Twist message:")
        rospy.loginfo(" Linear:  x=%.2f, y=%.2f, z=%.2f", self.latest_twist.linear.x, self.latest_twist.linear.y, self.latest_twist.linear.z)
        rospy.loginfo(" Angular: x=%.2f, y=%.2f, z=%.2f", self.latest_twist.angular.x, self.latest_twist.angular.y, self.latest_twist.angular.z)

        self.process_lidar()
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

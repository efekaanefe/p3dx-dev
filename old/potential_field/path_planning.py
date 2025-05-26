#!/usr/bin/env python
import math
import numpy as np
from scipy.optimize import minimize_scalar

import rospy
import tf.transformations
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

# ─── Helper: minimal angle difference ──────────────────────────────────────────────
def angle_diff(a, b):
    """returns a - b, wrapped to [-π, π]"""
    d = a - b
    return math.atan2(math.sin(d), math.cos(d))

# ─── 1) Trajectory planning ───────────────────────────────────────────────────────
def plan_two_quadratics(x_goal, y_goal, theta_goal):
    def solve_with_joint(xj):
        A = np.array([
            [0,         x_goal**2,   x_goal, 1],
            [0,       2*x_goal,       1,     0],
            [xj**2,   -xj**2,       -xj,    -1],
            [2*xj,    -2*xj,        -1,      0]
        ])
        b = np.array([
            y_goal,
            math.tan(theta_goal),
            0,
            0
        ])
        a1, a2, b2, c2 = np.linalg.solve(A, b)
        return (a1, 0, 0), (a2, b2, c2)

    def cost(xj):
        try:
            (a1, *_), (a2, *_) = solve_with_joint(xj)
            return abs(a1 - a2)
        except np.linalg.LinAlgError:
            return 1e6

    res = minimize_scalar(cost, bounds=(0.1, x_goal-0.1), method='bounded')
    x_joint = res.x
    coeffs1, coeffs2 = solve_with_joint(x_joint)
    return coeffs1, coeffs2, x_joint

# ─── 2) Build your on-demand controller ────────────────────────────────────────────
def make_path_follower(x_goal, y_goal, theta_goal,
                       pos_tol=0.2, yaw_tol=0.1,
                       v_nominal=0.2, L=0.3):
    coeffs1, coeffs2, x_joint = plan_two_quadratics(x_goal, y_goal, theta_goal)
    switched = False

    a1, b1, _ = coeffs1
    yj = a1 * x_joint**2
    heading_j = math.atan2(2*a1*x_joint + b1, 1)

    def get_control(x, y, yaw):
        nonlocal switched
        # check transition
        if (not switched and
            math.hypot(x - x_joint, y - yj) < pos_tol and
            abs(angle_diff(yaw, heading_j)) < yaw_tol):
            switched = True

        # pick correct poly
        a, b, _ = coeffs1 if not switched else coeffs2

        # curvature → ω
        y_p  = 2*a*x + b
        y_pp = 2*a
        denom = (1 + y_p**2)**1.5
        if abs(y_pp) < 1e-6:
            omega = 0.0
        else:
            R = denom / abs(y_pp)
            omega = v_nominal / R

        return v_nominal, omega

    return get_control

# ─── ROS Node ─────────────────────────────────────────────────────────────────────
class TrajectoryFollowerNode:
    def __init__(self):
        rospy.init_node("trajectory_follower")

        # parameters or hardcode your goal here:
        x_goal    = rospy.get_param("~x_goal",    6.0)
        y_goal    = rospy.get_param("~y_goal",    8.0)
        theta_deg = rospy.get_param("~theta_deg", 45.0)
        theta_goal = math.radians(theta_deg)

        self.get_control = make_path_follower(
            x_goal, y_goal, theta_goal,
            pos_tol=rospy.get_param("~pos_tol", 0.2),
            yaw_tol=rospy.get_param("~yaw_tol", 0.1),
            v_nominal=rospy.get_param("~v_nominal", 0.2),
            L=rospy.get_param("~wheelbase", 0.3)
        )

        self.pose = {"x":0.0, "y":0.0, "yaw":0.0}
        rospy.Subscriber("/RosAria/pose", Odometry, self.pose_callback)
        self.cmd_pub = rospy.Publisher("/RosAria/cmd_vel", Twist, queue_size=1)

        self.rate = rospy.Rate(10)
        self.run()

    def pose_callback(self, msg):
        self.pose["x"] = msg.pose.pose.position.x
        self.pose["y"] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _,_,yaw = tf.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w])
        self.pose["yaw"] = yaw

    def run(self):
        while not rospy.is_shutdown():
            v, omega = self.get_control(
                self.pose["x"], self.pose["y"], self.pose["yaw"]
            )
            twist = Twist()
            twist.linear.x  = v
            twist.angular.z = omega
            self.cmd_pub.publish(twist)
            self.rate.sleep()

if __name__ == "__main__":
    try:
        TrajectoryFollowerNode()
    except rospy.ROSInterruptException:
        pass

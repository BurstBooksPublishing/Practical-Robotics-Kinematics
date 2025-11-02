#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

# robot params (meters)
WHEEL_RADIUS = 0.08   # wheel radius
TRACK_WIDTH  = 0.46   # distance between wheels

# publishers to motor controllers (topics are examples)
pub_left = None
pub_right = None

def on_cmd(msg):
    v = msg.linear.x      # forward velocity (m/s)
    omega = msg.angular.z # yaw rate (rad/s)
    # wheel angular speeds (rad/s), from eq. (1)
    w_r = (v + 0.5*TRACK_WIDTH*omega)/WHEEL_RADIUS
    w_l = (v - 0.5*TRACK_WIDTH*omega)/WHEEL_RADIUS
    pub_right.publish(w_r)  # send to right motor controller
    pub_left.publish(w_l)   # send to left motor controller

if __name__ == '__main__':
    rospy.init_node('cmdvel_to_wheels')
    pub_left  = rospy.Publisher('/left_wheel_velocity',  Float64, queue_size=1)
    pub_right = rospy.Publisher('/right_wheel_velocity', Float64, queue_size=1)
    rospy.Subscriber('/cmd_vel', Twist, on_cmd)  # from navigation stack
    rospy.spin()
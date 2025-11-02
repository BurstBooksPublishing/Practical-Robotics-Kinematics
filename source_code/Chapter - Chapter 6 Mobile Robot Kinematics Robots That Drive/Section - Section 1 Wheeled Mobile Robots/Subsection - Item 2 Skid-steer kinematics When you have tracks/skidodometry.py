#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray  # example encoder array [vL, vR]
from geometry_msgs.msg import TransformStamped
import tf2_ros

# parameters (tune via calibration)
b_eff = rospy.get_param('~b_eff', 0.8)        # effective track separation (m)
slip_gain = rospy.get_param('~slip_gain', 0.95)  # scale forward speed
frame_id = rospy.get_param('~frame_id', 'odom')

# vehicle state
x = 0.0; y = 0.0; theta = 0.0
last_time = rospy.Time.now()

pub = rospy.Publisher('odom', Odometry, queue_size=10)
tfb = tf2_ros.TransformBroadcaster()

def enc_cb(msg):
    global x,y,theta,last_time
    now = rospy.Time.now()
    dt = (now - last_time).to_sec()
    if dt <= 0.0: return
    # encoder message supplies linear track speeds in m/s
    vL, vR = msg.data[0], msg.data[1]
    # apply slip model and compute twist
    v = slip_gain * 0.5 * (vR + vL)
    omega = (vR - vL) / b_eff
    # integrate
    x += v * np.cos(theta) * dt
    y += v * np.sin(theta) * dt
    theta += omega * dt
    last_time = now
    # publish odom
    odom = Odometry()
    odom.header.stamp = now
    odom.header.frame_id = frame_id
    odom.child_frame_id = 'base_link'
    odom.pose.pose.position.x = x
    odom.pose.pose.position.y = y
    # yaw -> quaternion (simple conversion)
    qz = np.sin(theta/2.0); qw = np.cos(theta/2.0)
    odom.pose.pose.orientation.z = qz; odom.pose.pose.orientation.w = qw
    odom.twist.twist.linear.x = v
    odom.twist.twist.angular.z = omega
    pub.publish(odom)
    # broadcast tf
    t = TransformStamped()
    t.header = odom.header
    t.child_frame_id = odom.child_frame_id
    t.transform.translation.x = x; t.transform.translation.y = y
    t.transform.rotation = odom.pose.pose.orientation
    tfb.sendTransform(t)

rospy.init_node('skid_odometry_node')
rospy.Subscriber('track_speeds', Float32MultiArray, enc_cb)  # encoders -> [vL, vR]
rospy.spin()
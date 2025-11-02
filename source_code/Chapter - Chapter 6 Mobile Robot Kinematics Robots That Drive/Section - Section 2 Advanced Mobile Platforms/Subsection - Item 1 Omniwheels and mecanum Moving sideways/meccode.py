import numpy as np
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

r = 0.06        # wheel radius (m)
L = 0.30        # half length (m)
W = 0.20        # half width (m)
sumLW = L + W

# compute wheel angular velocities (rad/s) from body twist
def twist_to_wheels(vx, vy, wz):
    # order: FL, FR, RL, RR
    wFL = (vx - vy - sumLW*wz) / r
    wFR = (vx + vy + sumLW*wz) / r
    wRL = (vx + vy - sumLW*wz) / r
    wRR = (vx - vy + sumLW*wz) / r
    return np.array([wFL, wFR, wRL, wRR])

# compute body velocities from wheel encoders (rad/s)
def wheels_to_twist(ws):
    # ws: numpy array [wFL, wFR, wRL, wRR]
    vx = (r/4.0) * np.sum(ws)
    vy = (r/4.0) * (-ws[0] + ws[1] + ws[2] - ws[3])
    wz = (r/(4.0*sumLW)) * (-ws[0] + ws[1] - ws[2] + ws[3])
    return vx, vy, wz

def cmd_vel_cb(msg):
    # convert incoming /cmd_vel to wheel commands
    ws = twist_to_wheels(msg.linear.x, msg.linear.y, msg.angular.z)
    # clamp to safe max angular speed (rad/s)
    ws = np.clip(ws, -300.0, 300.0)
    # publish as array to low-level controller
    out = Float64MultiArray(data=ws.tolist())
    wheel_pub.publish(out)

rospy.init_node('mecanum_kinematics_node')
wheel_pub = rospy.Publisher('/wheel_controller/command', Float64MultiArray, queue_size=1)
rospy.Subscriber('/cmd_vel', Twist, cmd_vel_cb)
rospy.spin()
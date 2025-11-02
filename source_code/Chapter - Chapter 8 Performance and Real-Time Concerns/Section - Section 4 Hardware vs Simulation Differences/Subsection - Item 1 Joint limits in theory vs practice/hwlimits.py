#!/usr/bin/env python3
import rospy
from urdf_parser_py.urdf import URDF  # parse URDF on parameter server
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

rospy.init_node('hw_limit_enforcer')
robot = URDF.from_parameter_server()               # read /robot_description
# compute effective limits with a margin factor
margin_rad = 0.05                                  # engineering margin (rad)
limits = {}
for j in robot.joints:
    if j.limit and j.type != 'fixed':
        low = j.limit.lower + margin_rad          # conservative lower bound
        high = j.limit.upper - margin_rad         # conservative upper bound
        limits[j.name] = (low, high)

pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=1)

def saturate_point(joint_names, point):
    # saturate positions elementwise to effective limits
    pos = list(point.positions)
    for i, name in enumerate(joint_names):
        low, high = limits.get(name, (-float('inf'), float('inf')))
        # simple clamping; could apply soft ramp instead
        if pos[i] < low: pos[i] = low
        if pos[i] > high: pos[i] = high
    point.positions = pos
    return point

# example usage: accept a planned trajectory, clamp, and publish
def publish_safe(traj_msg):
    for p in traj_msg.points:
        p = saturate_point(traj_msg.joint_names, p)  # enforce hw bounds
    pub.publish(traj_msg)

# controller loop would call publish_safe with MoveIt or planner output
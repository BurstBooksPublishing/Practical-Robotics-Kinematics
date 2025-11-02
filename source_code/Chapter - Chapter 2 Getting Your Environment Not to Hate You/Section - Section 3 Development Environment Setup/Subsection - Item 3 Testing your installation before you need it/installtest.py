#!/usr/bin/env python3
# Quick install check: pybullet vs analytic FK for a 2-DOF planar arm.
import sys, math, numpy as np
import pybullet as p

URDF_PATH = "two_link.urdf"   # URDF must define joint names 'joint1','joint2' and link 'ee_link'.
EE_LINK_NAME = "ee_link"
L1, L2 = 0.5, 0.3             # link lengths used in analytic model (meters)
Q = [0.0, 0.0]                # test joint configuration (radians)
ALPHA = 0.01                  # meters per radian for pose error weighting
TOL = 1e-6                    # acceptable pose error (choose per robot precision)

print("pybullet version:", getattr(p, "__version__", "unknown"))

p.connect(p.DIRECT)
robot = p.loadURDF(URDF_PATH, useFixedBase=True)

# map link name to index
ee_index = None
for i in range(p.getNumJoints(robot)):
    info = p.getJointInfo(robot, i)
    link_name = info[12].decode()  # jointInfo[12] is link name
    if link_name == EE_LINK_NAME:
        ee_index = i
        break
if ee_index is None:
    print("ERROR: end-effector link not found in URDF:", EE_LINK_NAME)
    sys.exit(2)

# reset to known joint positions
for i in range(p.getNumJoints(robot)):
    p.resetJointState(robot, i, targetValue=Q[i] if i < len(Q) else 0.0)

# get FK from pybullet
ls = p.getLinkState(robot, ee_index, computeForwardKinematics=True)
pos_pb = np.array(ls[4])         # world position of link COM / frame
orn_pb = ls[5]                   # quaternion (x,y,z,w)
R_pb = np.array(p.getMatrixFromQuaternion(orn_pb)).reshape(3,3)
T_pb = np.eye(4)
T_pb[:3,:3] = R_pb
T_pb[:3,3] = pos_pb

# analytic FK for planar 2-DOF at Q = [q1,q2] (assume rotations about z, base at origin)
q1, q2 = Q
x = L1*math.cos(q1) + L2*math.cos(q1+q2)
y = L1*math.sin(q1) + L2*math.sin(q1+q2)
z = 0.0
R_analytic = np.array([
    [math.cos(q1+q2), -math.sin(q1+q2), 0.0],
    [math.sin(q1+q2),  math.cos(q1+q2), 0.0],
    [0.0,              0.0,             1.0],
])
T_expected = np.eye(4)
T_expected[:3,:3] = R_analytic
T_expected[:3,3] = np.array([x,y,z])

# compute error per equations (1)-(2)
t_err = T_expected[:3,3] - T_pb[:3,3]
R_err = T_expected[:3,:3].T @ T_pb[:3,:3]
trace = np.trace(R_err)
# numerical clamp for acos
trace = max(-1.0, min(3.0, trace))
theta = math.acos((trace - 1.0) / 2.0)
pose_error = math.sqrt(np.dot(t_err,t_err) + (ALPHA*theta)**2)

print("pose_error (m):", pose_error)
if pose_error > TOL:
    print("FAILED: pose mismatch exceeds tolerance")
    sys.exit(1)
print("OK: installation smoke-test passed")
p.disconnect()
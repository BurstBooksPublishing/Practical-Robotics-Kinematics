import pybullet as p
import pybullet_data
import numpy as np

p.connect(p.DIRECT)                       # headless client for batch use
p.setAdditionalSearchPath(pybullet_data.getDataPath())
robot = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

# map joint names to indices and collect revolute joints
joints = {}
rev_joints = []
for i in range(p.getNumJoints(robot)):
    info = p.getJointInfo(robot, i)      # returns tuple; name at index 1
    name = info[1].decode()               # decode bytes to str
    jtype = info[2]
    joints[name] = i
    if jtype == p.JOINT_REVOLUTE or jtype == p.JOINT_PRISMATIC:
        rev_joints.append(i)

# set a realistic joint configuration (radians)
q = [0.0, -0.4, 0.0, -2.0, 0.0, 1.6, 0.8]   # 7-DOF Panda arm
for idx, angle in zip(rev_joints, q):
    p.resetJointState(robot, idx, angle)    # deterministic FK without dynamics

# query end-effector link pose (link name 'panda_hand' or index known from URDF)
ee_link = joints["panda_hand"]
# computeForwardKinematics=1 ensures internal recomputation even if dynamics used
ls = p.getLinkState(robot, ee_link, computeForwardKinematics=1)
pos_world = np.array(ls[4])                 # linkWorldPosition
orn_world = np.array(ls[5])                 # linkWorldOrientation as quaternion

# convert quaternion to rotation matrix
R = np.array(p.getMatrixFromQuaternion(orn_world)).reshape(3, 3)
T = np.eye(4)
T[:3, :3] = R
T[:3, 3] = pos_world
print("EE world transform:\n", T)
p.disconnect()
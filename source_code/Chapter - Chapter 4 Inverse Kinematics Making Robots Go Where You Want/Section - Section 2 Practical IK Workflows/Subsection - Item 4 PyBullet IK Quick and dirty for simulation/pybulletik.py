import pybullet as p
import pybullet_data
import numpy as np

p.connect(p.GUI)                       # start physics client
p.setAdditionalSearchPath(pybullet_data.getDataPath())
robot = p.loadURDF("kuka_iiwa/model.urdf", useFixedBase=True)  # load robot

ee_link = 6                            # end-effector link index (example)
num_joints = p.getNumJoints(robot)

# read joint info and initial rest pose for seeding
joint_indices = []
rest_pose = []
for i in range(num_joints):
    info = p.getJointInfo(robot, i)
    if info[2] == p.JOINT_REVOLUTE:   # select revolute joints only
        joint_indices.append(i)
        rest_pose.append(p.getJointState(robot, i)[0])

# target pose in world frame
target_pos = [0.5, 0.0, 0.6]
target_ori = p.getQuaternionFromEuler([0, 1.57, 0])

# solver parameters: lower/upper limits and damping arrays (example)
lower_limits = [-2.9671]*len(joint_indices)
upper_limits = [2.9671]*len(joint_indices)
joint_ranges = [5.9342]*len(joint_indices)
damping = [0.1]*len(joint_indices)

for _ in range(200):
    # solve IK (seed via restPoses helps regularize)
    ik_solution = p.calculateInverseKinematics(
        bodyUniqueId=robot,
        endEffectorLinkIndex=ee_link,
        targetPosition=target_pos,
        targetOrientation=target_ori,
        lowerLimits=lower_limits,
        upperLimits=upper_limits,
        jointRanges=joint_ranges,
        restPoses=rest_pose,
        jointDamping=damping,
        maxNumIterations=50,            # increase for tighter convergence
        residualThreshold=1e-4
    )
    # apply solution to joint controllers (limiting to relevant joints)
    target_positions = [ik_solution[i] for i in range(len(joint_indices))]
    p.setJointMotorControlArray(
        bodyIndex=robot,
        jointIndices=joint_indices,
        controlMode=p.POSITION_CONTROL,
        targetPositions=target_positions,
        positionGains=[0.2]*len(joint_indices)
    )

    p.stepSimulation()                # advance sim
    # compute actual EE pose and monitor residual
    link_state = p.getLinkState(robot, ee_link)
    actual_pos = np.array(link_state[4])   # world link position
    err = np.linalg.norm(actual_pos - np.array(target_pos))
    if err < 1e-3:
        break
p.disconnect()
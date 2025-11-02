import pybullet as p
import pybullet_data
import numpy as np

p.connect(p.GUI)                      # or p.DIRECT for headless testing
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # model search path
plane = p.loadURDF("plane.urdf")      # environment
robot = p.loadURDF("ur_description/ur5.urdf", useFixedBase=True)  # example URDF

end_effector_link = 7                # URDF link index (use getNumJoints to confirm)
joint_indices = [i for i in range(p.getNumJoints(robot)) if p.getJointInfo(robot, i)[2] != p.JOINT_FIXED]

# desired Cartesian target (world frame)
target_pos = np.array([0.6, 0.0, 0.4])
gain = 0.8
dt = 0.01

for step in range(200):
    # read current joint positions
    joint_states = [p.getJointState(robot, i)[0] for i in joint_indices]
    # get current end-effector pose
    link_state = p.getLinkState(robot, end_effector_link, computeForwardKinematics=True)
    cur_pos = np.array(link_state[4])   # world position of link frame

    err = target_pos - cur_pos
    if np.linalg.norm(err) < 1e-3:
        break

    # compute Jacobian (linear part), zeros for local velocities used here
    zero_vec = [0.0]*len(joint_indices)
    jac_t, jac_r = p.calculateJacobian(robot, end_effector_link, [0,0,0], joint_states, zero_vec, zero_vec)
    J = np.array(jac_t)                 # shape (3, n)

    # damped pseudoinverse to avoid large steps near singularities
    lam = 1e-3
    J_pinv = np.linalg.inv(J.dot(J.T) + lam*np.eye(3)).dot(J).T

    dq = J_pinv.dot(gain * err)         # resolved-rate delta q
    # enforce position and velocity limits in practice
    new_q = np.array(joint_states) + dq * dt

    # apply position control to reach new joint targets
    p.setJointMotorControlArray(robot, joint_indices, p.POSITION_CONTROL, targetPositions=new_q.tolist())
    p.stepSimulation()
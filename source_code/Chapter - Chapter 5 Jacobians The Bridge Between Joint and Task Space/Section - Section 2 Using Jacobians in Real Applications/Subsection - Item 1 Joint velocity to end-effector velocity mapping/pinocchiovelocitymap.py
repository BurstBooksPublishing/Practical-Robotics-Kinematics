import pinocchio as pin
import numpy as np

# assume: model, data, ee_frame_id already created and valid
q = np.zeros(model.nq)                 # current joint positions
q_dot = np.zeros(model.nv)             # measured joint velocities

# forward kinematics and update frames (Pinocchio APIs)
pin.forwardKinematics(model, data, q)               # update joint placements
pin.updateFramePlacements(model, data)              # update frame placements

# getFrameJacobian returns 6 x n matrix (world-aligned by choice)
J = pin.getFrameJacobian(model, data, ee_frame_id,
                         pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)

# map joint rates to end-effector twist (6D vector)
twist_ee = J.dot(q_dot)             # [vx, vy, vz, wx, wy, wz]

# desired twist and damped least squares inverse
v_des = np.array([0.1,0,0, 0,0,0])  # example: move along x at 0.1 m/s
lambda2 = 1e-4                       # tuning parameter for DLS

# Damped least-squares inverse: q_dot = J^T (J J^T + lambda2 I)^-1 v_des
JJt = J.dot(J.T)
q_dot_cmd = J.T.dot(np.linalg.solve(JJt + lambda2*np.eye(6), v_des))

# enforce joint velocity limits by uniform scaling if necessary
qdot_limits = 1.0*np.ones(model.nv)    # example limits (rad/s)
scale = min(1.0, np.min(np.abs(qdot_limits / (np.abs(q_dot_cmd) + 1e-12))))
q_dot_cmd = scale * q_dot_cmd

# q_dot_cmd now safe to send to low-level velocity controller
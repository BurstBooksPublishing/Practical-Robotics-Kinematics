import numpy as np
import pinocchio as pin

# model, data, frame_id provided; q, xdot are current state and task velocity
pin.forwardKinematics(model, data, q)                  # update kinematics
pin.updateFramePlacements(model, data)
J = pin.getFrameJacobian(model, data, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)  # 6xN
U, S, Vt = np.linalg.svd(J, full_matrices=False)       # compute SVD
sigma_min = S.min()
# damping schedule parameters
sigma_h, sigma_l, lambda0 = 1e-2, 1e-3, 1e-2
# compute lambda per schedule
if sigma_min > sigma_h:
    lam = 0.0
elif sigma_min < sigma_l:
    lam = lambda0
else:
    lam = lambda0 * (sigma_h - sigma_min) / (sigma_h - sigma_l)
# build damped pseudo-inverse directly with SVD components
S_inv = np.array([s / (s**2 + lam**2) for s in S])     # damped reciprocals
J_pinv = Vt.T @ np.diag(S_inv) @ U.T                   # N x 6
qdot_task = J_pinv @ xdot                              # primary task velocity
# null-space bias towards nominal joint pose
k_null = 0.5                                           # gain
q_nom = np.zeros_like(q)                               # mid-range example
qdot_null = k_null * (q_nom - q)
I = np.eye(model.nq)
qdot = qdot_task + (I - J_pinv @ J) @ qdot_null       # combined command
# send qdot to low-level controller (convert to torque/velocity as needed)
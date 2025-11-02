import numpy as np
import pinocchio as pin

# model, data, frame_id are preloaded; q is current joint vector
pin.forwardKinematics(model, data, q)             # compute transforms
pin.updateFramePlacement(model, data, frame_id)   # update frame placement
J = pin.getFrameJacobian(model, data, frame_id, pin.ReferenceFrame.LOCAL)  # 6xN

# compute SVD (small matrices; cheap for 6xN)
U, S, Vt = np.linalg.svd(J, full_matrices=False)   # S: singular values array

sigma_min = S[-1]
sigma_max = S[0]
manipulability = float(np.prod(S))                # eq. (1)
condition_number = sigma_max / (sigma_min + 1e-12)

# thresholds (tune to robot and task units)
sigma_thresh = 1e-3
cond_thresh = 1e5
near_singular = (sigma_min < sigma_thresh) or (condition_number > cond_thresh)

# compute damped least squares inverse (eq. (2))
def damped_pinv(J, lam):
    JJt = J @ J.T
    inv = np.linalg.inv(JJt + (lam**2) * np.eye(JJt.shape[0]))
    return J.T @ inv  # returns N x 6 pseudoinverse

lam0 = 1e-3
lam = lam0 if not near_singular else max(lam0, lam0*(sigma_thresh/sigma_min))
J_pinv = damped_pinv(J, lam)

# apply to desired task velocity v_task (6x1) -> joint velocity qdot (Nx1)
qdot = J_pinv @ v_task  # safe, regularized solution
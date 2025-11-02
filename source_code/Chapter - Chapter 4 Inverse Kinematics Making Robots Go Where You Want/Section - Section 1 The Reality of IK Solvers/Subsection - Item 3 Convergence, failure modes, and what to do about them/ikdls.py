import numpy as np
import pinocchio as pin

# model, data, frame_id created elsewhere; q0 initial joint vector
def resolved_rate_ik(model, data, frame_id, q0, x_des, dt=0.01, max_it=100):
    q = q0.copy()
    for it in range(max_it):
        pin.forwardKinematics(model, data, q)               # update FK
        pin.updateFramePlacements(model, data)
        X = data.oMf[frame_id].translation                 # current EE position
        e = x_des - X                                      # position error
        if np.linalg.norm(e) < 1e-4:                       # convergence test
            return q, True
        J = pin.getFrameJacobian(model, data, frame_id,
                                 pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        # SVD and adaptive damping
        U, S, Vt = np.linalg.svd(J, full_matrices=False)
        sigma_min = S[-1]
        sigma_thresh = 1e-3
        lam = 0.0 if sigma_min > sigma_thresh else 1e-2*(sigma_thresh - sigma_min)
        # DLS pseudoinverse (stable form)
        JJt = J.dot(J.T)
        inv = np.linalg.inv(JJt + (lam**2)*np.eye(JJt.shape[0]))
        J_pinv = J.T.dot(inv)
        dq = J_pinv.dot(e) * dt                             # small incremental step
        # joint limit check (brief); clamp or project as needed
        q_next = q + dq
        if violates_limits(q_next):                         # user-defined check
            return q, False
        q = q_next
    # random-restart fallback
    for seed in sample_random_seeds(n=5):
        q_seed = seed
        q_try, ok = resolved_rate_ik(model, data, frame_id, q_seed, x_des, dt, int(max_it/2))
        if ok: return q_try, True
    return q, False
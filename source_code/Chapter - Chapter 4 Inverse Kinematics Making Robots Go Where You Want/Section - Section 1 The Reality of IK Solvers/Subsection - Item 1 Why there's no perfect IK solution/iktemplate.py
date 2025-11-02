import numpy as np
from scipy.optimize import least_squares
# fk(q) -> 6D pose error vector; jac(q) -> 6xN Jacobian (use Pinocchio/Drake API)
def residual(q, q_des_pose):
    # compute pose residual (position + orientation error vector)
    err = fk(q) - q_des_pose  # small-angle/pose metric assumed
    # soft penalty for joint limits (simple quadratic penalty)
    limit_pen = np.where(q < q_min, q_min - q, 0.0) + np.where(q > q_max, q - q_max, 0.0)
    return np.concatenate([err, 0.1 * limit_pen])  # scale penalties

def jacobian(q, q_des_pose):
    J_task = jac(q)  # 6xN
    # penalty Jacobian (diagonal sign approx)
    J_pen = np.zeros((len(q), len(q)))
    for i in range(len(q)):
        if q[i] < q_min[i]:
            J_pen[len(err)+i, i] = -1.0
        elif q[i] > q_max[i]:
            J_pen[len(err)+i, i] = 1.0
    # assemble full Jacobian for least_squares (stack rows)
    return np.vstack([J_task, 0.1 * J_pen])

# initial guess and bounds
q0 = q_prev.copy()  # warm start from previous solution
res = least_squares(residual, q0, jac=jacobian, args=(q_des_pose,),
                    bounds=(q_min, q_max), xtol=1e-6, ftol=1e-6, max_nfev=200)
q_sol = res.x  # may still need collision check and fallback
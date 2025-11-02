import numpy as np

def adaptive_damped_pinv(J, sigma_thresh=1e-2, lambda_max=1e-1):
    # compute SVD
    U, s, Vt = np.linalg.svd(J, full_matrices=False)  # s: singular values
    sigma_min = s[-1]
    # adaptive damping schedule
    if sigma_min >= sigma_thresh:
        lam = 0.0
    else:
        # quadratic ramp: lam in (0, lambda_max]
        alpha = (1.0 - sigma_min/sigma_thresh)**2
        lam = alpha * lambda_max
    # build damped inverse singular values
    s_damped = s / (s**2 + lam**2)
    J_pinv = (Vt.T * s_damped) @ U.T
    return J_pinv, sigma_min, lam

# Example usage:
# J = robot.get_jacobian(frame='ee')  # e.g., computed by Pinocchio or Drake
# v_task = np.array([vx, vy, vz, wx, wy, wz])
# J_pinv, sigma_min, lam = adaptive_damped_pinv(J)
# dq = J_pinv.dot(v_task)  # joint velocity command
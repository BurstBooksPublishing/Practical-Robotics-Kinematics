import numpy as np
import pinocchio as pin

def seed_generator(last_q, q_nom, alpha=0.2):
    # Weighted blend: keeps continuity but biases to nominal pose
    return (1.0 - alpha) * last_q + alpha * q_nom

def ik_solve_with_seed(model, data, frame_id, x_des, seed,
                       joint_limits, max_iters=50, tol=1e-4):
    q = seed.copy()
    lambda_damp = 1e-2
    for i in range(max_iters):
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacement(model, data, frame_id)
        x_curr = data.oMf[frame_id].translation  # example position task
        err = x_des - x_curr
        if np.linalg.norm(err) < tol:
            return q, True, 'converged'
        J = pin.computeFrameJacobian(model, data, q, frame_id)[:, :len(q)]
        # manipulability check (small value -> near-singularity)
        manipulability = np.sqrt(np.linalg.det(J.dot(J.T)) + 1e-12)
        # damped least-squares step
        A = J.T.dot(J) + lambda_damp * np.eye(len(q))
        b = J.T.dot(err)
        try:
            delta = np.linalg.solve(A, b)
        except np.linalg.LinAlgError:
            return q, False, 'singular_matrix'
        q_trial = q + delta
        # enforce simple joint limits projection
        q_trial = np.minimum(np.maximum(q_trial, joint_limits[:,0]), joint_limits[:,1])
        # accept if reduces error, otherwise increase damping
        pin.forwardKinematics(model, data, q_trial)
        pin.updateFramePlacement(model, data, frame_id)
        err_trial = x_des - data.oMf[frame_id].translation
        if np.linalg.norm(err_trial) < np.linalg.norm(err):
            q = q_trial
            lambda_damp *= 0.8
        else:
            lambda_damp *= 2.0
        # reject if manipulability too low
        if manipulability < 1e-6:
            return q, False, 'near_singularity'
    return q, False, 'timeout'
# Usage: build model, data, frame_id; provide x_des, last_q, q_nom, joint_limits
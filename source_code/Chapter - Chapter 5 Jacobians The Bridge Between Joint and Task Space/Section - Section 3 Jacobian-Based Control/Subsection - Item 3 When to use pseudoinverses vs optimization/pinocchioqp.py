import numpy as np
import pinocchio as pin
import cvxpy as cp

# model, data setup done elsewhere; q is current joint vector
# frameId is end-effector frame; pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED used
J = pin.getFrameJacobian(model, data, frameId,
                         pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)  # (6,n)

# desired task velocity (6D twist)
v = np.hstack([linear_vel, angular_vel])  # shape (6,)

# Damped pseudoinverse solution (fast)
lambda_d = 0.05
JJT = J @ J.T
Jplus = J.T @ np.linalg.inv(JJT + (lambda_d**2) * np.eye(JJT.shape[0]))
qdot_pinv = Jplus @ v  # minimal-norm damped least-squares

# QP: minimize ||J qdot - v||^2 + alpha ||qdot - qdot_pref||^2
n = J.shape[1]
qdot = cp.Variable(n)
alpha = 1e-2
qdot_pref = np.zeros(n)  # e.g., bias toward home posture

objective = cp.Minimize(0.5 * cp.sum_squares(J @ qdot - v)
                        + 0.5 * alpha * cp.sum_squares(qdot - qdot_pref))
# inequality example: simple linear limit A qdot <= b (e.g., keep end-effector z-velocity <= zmax)
Aineq = np.zeros((1,n)); bineq = np.array([0.5])  # placeholder
constraints = [Aineq @ qdot <= bineq,
               qdot >= qdot_min, qdot <= qdot_max]  # velocity bounds (numpy arrays)

prob = cp.Problem(objective, constraints)
prob.solve(solver=cp.OSQP, warm_start=True)  # OSQP good for real-time QPs

qdot_qp = qdot.value  # use in low-level controller
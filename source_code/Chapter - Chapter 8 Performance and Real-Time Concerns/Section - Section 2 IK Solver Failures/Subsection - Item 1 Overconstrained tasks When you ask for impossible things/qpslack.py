import numpy as np
import osqp
from scipy import sparse

# J: task Jacobian (m x n), e: task error (m,), G: inequality Jacobian (p x n)
# h: inequality rhs (p,), q0: current joint vector (n,), qmin/qmax: bounds
rho = 1e4           # slack penalty
m, n = J.shape[0], J.shape[1]
p = G.shape[0]

# Build QP: minimize 0.5*[dq;s]^T P [dq;s] + q^T [dq;s]
P = sparse.block_diag([sparse.csc_matrix(2*(J.T @ J)), rho * sparse.eye(p)])
qvec = np.hstack((2*(J.T @ e), np.zeros(p)))

# Constraints: G*dq - s <= h  =>  [G, -I] [dq;s] <= h
# Joint limits: qmin <= q0 + dq <= qmax  =>  dq in [qmin-q0, qmax-q0]
A = sparse.vstack([sparse.hstack([G, -sparse.eye(p)]),
                   sparse.hstack([sparse.eye(n), sparse.csc_matrix((n,p))]),
                   sparse.hstack([-sparse.eye(n), sparse.csc_matrix((n,p))])]).tocsc()
l = np.hstack((-np.inf*np.ones(p), qmin - q0, -(qmax - q0)))  # lower bounds
u = np.hstack((h, qmax - q0, -(qmin - q0)))                    # upper bounds

# OSQP setup and solve (warm-startable in control loop)
prob = osqp.OSQP()
prob.setup(P=P, q=qvec, A=A, l=l, u=u, verbose=False)
res = prob.solve()

dq = res.x[:n]
s = res.x[n:]
slack_norm = np.linalg.norm(np.clip(s, 0, None), 2)

# Decision: if slack_norm above tolerance, treat as infeasible
if slack_norm > 1e-3:
    # handle overconstraint: relax task, replan, or notify higher-level module
    pass
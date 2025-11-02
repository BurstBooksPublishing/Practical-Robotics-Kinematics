import numpy as np

def damped_pinv(J, lam):
    # left-damped pseudoinverse for m<=n
    JJt = J @ J.T
    inv = np.linalg.inv(JJt + (lam**2) * np.eye(JJt.shape[0]))
    return J.T @ inv

# inside control loop:
# J = ... # get Jacobian from Pinocchio/MoveIt/Drake, shape (m,n)
# v_task = ... # desired end-effector spatial velocity, shape (m,)
# q = ... # current joint positions, shape (n,)
# q_nom = ... # nominal posture, shape (n,)

lam = 0.01               # damping factor, tune per robot
Kp = 5.0                 # secondary gain
Jp = damped_pinv(J, lam) # J^#
N = np.eye(J.shape[1]) - Jp @ J  # null-space projector

q_dot0 = -Kp * (q - q_nom)       # posture regulation
q_dot = Jp @ v_task + N @ q_dot0 # full joint velocity command

# saturate and send q_dot to low-level controller
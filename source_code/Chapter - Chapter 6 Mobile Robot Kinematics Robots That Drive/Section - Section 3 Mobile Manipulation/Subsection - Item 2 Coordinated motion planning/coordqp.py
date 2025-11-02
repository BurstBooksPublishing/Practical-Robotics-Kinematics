import numpy as np
import pinocchio as pin
import osqp
from scipy import sparse

# load model (URDF must include base as virtual joint)
model = pin.buildModelFromUrdf("robot_with_base.urdf")
data = model.createData()

# current combined configuration q (size n)
q = np.array(...)      # current base+arm positions
v_des = np.array(...)  # desired end-effector twist (6D) in world frame

# compute Jacobian for end-effector frame (external API)
frame_id = model.getFrameId("ee_link")
pin.forwardKinematics(model, data, q)
pin.updateFramePlacements(model, data)
J = pin.computeFrameJacobian(model, data, q, frame_id, pin.LOCAL_WORLD_ALIGNED)

# build QP matrices for: min 1/2 ||J qdot - v_des||^2 + (lambda/2)||qdot||^2
lambda_reg = 1e-2
H = J.T.dot(J) + lambda_reg * np.eye(model.nq)    # Hessian
g = -J.T.dot(v_des)                                # linear term

# nonholonomic equality: no lateral base velocity in body frame
# assume q indices 0,1,2 -> x,y,theta; lateral velocity constraint expressed in world:
S = np.zeros((1, model.nq))
S[0,1] = 1.0  # enforce dot y = 0 (transform if body-frame needed)
Aeq = S
beq = np.zeros((1,))

# velocity bounds as inequalities: -qdot_max <= qdot <= qdot_max
qdot_max = 1.0
Aineq = sparse.vstack([sparse.eye(model.nq), -sparse.eye(model.nq)])
bineq = np.hstack([qdot_max*np.ones(model.nq), qdot_max*np.ones(model.nq)])

# stack constraints for OSQP: OSQP solves min 0.5 x'Hx + g'x, s.t. l <= Ax <= u
P = sparse.csc_matrix(H)
q_vec = g
A_osqp = sparse.vstack([sparse.csc_matrix(Aeq), Aineq]).tocsc()
l_osqp = np.hstack([beq, -np.inf*np.ones(2*model.nq)])  # equality then inequalities
u_osqp = np.hstack([beq, bineq])

# solve
prob = osqp.OSQP()
prob.setup(P, q_vec, A_osqp, l_osqp, u_osqp, verbose=False)
res = prob.solve()
qdot = res.x  # commanded generalized velocities
# send qdot to low-level controllers for base and arm
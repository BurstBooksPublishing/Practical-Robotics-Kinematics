import numpy as np
import pinocchio as pin  # fast FK/Jacobian
from pydrake.all import MathematicalProgram, Solve  # Drake optimizer

# --- load robot model (replace with path to your URDF) ---
model = pin.buildModelFromUrdf("path/to/franka.urdf")  # build model
data = model.createData()  # runtime data container

# --- current state and task ---
q = np.zeros(model.nq)  # current joint positions
pin.forwardKinematics(model, data, q)  # compute kinematic quantities
pin.updateFramePlacements(model, data)  # ensure frame placements are updated
frame_id = model.getFrameId("panda_link8")  # end-effector frame name
ee_pose = data.oMf[frame_id]  # SE3 pose of frame in world
J = pin.getFrameJacobian(model, data, frame_id, pin.ReferenceFrame.LOCAL)  # 6xN Jacobian

# desire: small Cartesian delta in end-effector frame
delta_x = np.array([0.01, 0.0, 0.0, 0.0, 0.0, 0.0])  # translation + rotation axis

# --- formulate QP: minimize ||J dq - delta_x||^2 + eps||dq||^2 subject to joint limits ---
prog = MathematicalProgram()
dq = prog.NewContinuousVariables(model.nq, "dq")  # decision: delta joint positions

W = 1.0 * np.eye(6)  # task weight (cartesian)
lam = 1e-3  # damping
# Quadratic cost: (J dq - delta_x)'W(J dq - delta_x) + lam*dq'dq
Q_cost = J.T.dot(W).dot(J) + lam * np.eye(model.nq)
b_cost = -J.T.dot(W).dot(delta_x)
prog.AddQuadraticCost(Q_cost, b_cost, dq)

# joint limits as bounding box for q + dq
q_min = model.lowerPositionLimit
q_max = model.upperPositionLimit
prog.AddBoundingBoxConstraint(q_min - q, q_max - q, dq)

result = Solve(prog)  # solve QP (Drake picks available solver)
dq_sol = result.GetSolution(dq)  # apply this delta to commanded joints
# --- send q + dq_sol to low-level controller (with safety checks) ---
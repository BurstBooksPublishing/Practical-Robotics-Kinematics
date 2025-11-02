import pinocchio as pin
import numpy as np
from scipy.optimize import least_squares

# model/data setup (URDF path and frame name must match your robot)
model = pin.buildModelFromUrdf("robot.urdf")         # load model (C++ API available)
data = model.createData()
frame_id = model.getFrameId("ee_link")

# desired pose (SE3): rotation R_d (3x3), translation t_d (3,)
X_des = pin.SE3(np.eye(3), np.array([0.5, 0.0, 0.4]))

# helper: compute 6D se3 error and frame Jacobian in LOCAL WORLD ALIGNED frame
def pose_residual(q):
    pin.forwardKinematics(model, data, q)            # computes joint placements
    pin.updateFramePlacements(model, data)           # computes frame placements
    X = data.oMf[frame_id]                           # current end-effector SE3 (use \lstinline|data.oMf|)
    e = pin.log6(X_des.inverse() * X)                # 6D twist error se(3) -> R^6
    pin.computeFrameJacobian(model, data, q, frame_id, pin.ReferenceFrame.LOCAL) 
    J = pin.getFrameJacobian(model, data, frame_id, pin.ReferenceFrame.LOCAL)
    # return stacked residual: pose error and small regularization (for stability)
    reg = 1e-3 * (q - q_ref)
    return np.hstack([e, reg])

# initial guess and bounds
q0 = pin.neutral(model)                              # safe default joint config
q_ref = q0.copy()
lb = np.array([lim.lower for lim in model.lowerPositionLimit])  # use model joint limits
ub = np.array([lim.upper for lim in model.upperPositionLimit])

# call SciPy least-squares with bounds; optimizer will call pose_residual many times
res = least_squares(pose_residual, q0, bounds=(lb, ub), xtol=1e-6, ftol=1e-6, max_nfev=200)
q_sol = res.x
# q_sol can be sent to the low-level joint controller after safety checks
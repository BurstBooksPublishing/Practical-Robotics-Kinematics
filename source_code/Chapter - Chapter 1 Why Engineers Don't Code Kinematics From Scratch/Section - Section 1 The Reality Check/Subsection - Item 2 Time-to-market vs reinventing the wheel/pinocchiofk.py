import pinocchio as pin      # use package pinocchio (C++ core, Python bindings)
from pinocchio.utils import zero
import numpy as np

# Load robot model from URDF (common workflow) -- fast, tested parser.
model = pin.buildModelFromUrdf("urdf/industrial_arm.urdf")  # URDF path
data = model.createData()

# Example joint vector (6-DOF), use numpy array; #seed from sensors or initial guess.
q = np.array([0.0, -0.5, 0.8, -1.2, 0.4, 0.0])

# Forward kinematics and frame placement update (single API call sequence).
pin.forwardKinematics(model, data, q)         # compute transforms
pin.updateFramePlacements(model, data)        # update frame placements

# Get end-effector frame by name; use frame index lookup once in init.
ee_frame = model.getFrameId("ee_link")        # fast lookup
ee_pose = data.oMf[ee_frame]                  # SE3 object: .translation, .rotation

# Spatial Jacobian at end-effector in local frame.
J = pin.getFrameJacobian(model, data, q, ee_frame, pin.LOCAL)  # 6xN

# Inline checks: reachability and numerical condition number for singularity detection.
cond = np.linalg.cond(J[:3, :])               # translational part cond-number
# Use results for planning or control loops.
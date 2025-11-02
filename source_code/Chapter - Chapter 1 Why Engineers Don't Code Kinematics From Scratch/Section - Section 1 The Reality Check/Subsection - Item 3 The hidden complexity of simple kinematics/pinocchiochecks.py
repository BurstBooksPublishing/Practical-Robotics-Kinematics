import pinocchio as pin
import numpy as np

# Load model from URDF (path)/build model/data
model = pin.buildModelFromUrdf("robot.urdf")          # URDF must match runtime robot
data = model.createData()

q = np.zeros(model.nq)                               # current joint position
# Compute forward kinematics and frames
pin.forwardKinematics(model, data, q)                # joint -> placements
pin.updateFramePlacements(model, data)               # compute all frame placements
ee_frame = model.getFrameId("ee_link")               # end-effector frame id
T_0ee = data.oMf[ee_frame]                           # SE(3) placement

# Jacobian and singular-value check
J = pin.computeFrameJacobian(model, data, q, ee_frame, pin.LOCAL)  # 6 x n
sv = np.linalg.svd(J, compute_uv=False)                         # singular values
sigma_min = sv[-1] if sv.size else 0.0

# Joint limit check (model.upperPositionLimit exists)
within_limits = np.all(q >= model.lowerPositionLimit) and \
                np.all(q <= model.upperPositionLimit)

# Defensive decision
if sigma_min < 1e-4 or not within_limits:
    raise RuntimeError("Unsafe kinematic configuration: singular or limit violation")
# Else proceed to compute trajectory or IK seed updates
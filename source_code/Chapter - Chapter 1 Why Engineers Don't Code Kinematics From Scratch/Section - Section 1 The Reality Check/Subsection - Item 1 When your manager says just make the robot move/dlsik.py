import numpy as np
import pinocchio as pin

urdf = "/path/to/robot.urdf"            # URDF file for model
model = pin.buildModelFromUrdf(urdf)
data = model.createData()
frame_name = "ee_link"                  # end-effector frame name
fid = model.getFrameId(frame_name)

q = pin.neutral(model)                  # initial config (numpy array)
target_pose = pin.SE3.Identity()        # set desired pose (example)

for it in range(100):
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    current = data.oMf[fid]             # current end-effector pose
    # 6D error: translation + rotation (log map)
    dx_pos = target_pose.translation - current.translation
    dx_rot = (target_pose.rotation * current.rotation.transpose()).log()
    dx = np.hstack([dx_pos, dx_rot])    # spatial error (6,)
    if np.linalg.norm(dx) < 1e-4:
        break
    pin.computeJointJacobians(model, data, q)
    J = pin.getFrameJacobian(model, data, fid, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    # Damped least-squares step
    lambda2 = 1e-3
    JJt = J.dot(J.T) + lambda2 * np.eye(6)
    dq = J.T.dot(np.linalg.solve(JJt, dx))   # solve (6x6)
    # apply small step and enforce joint limits (example bounds)
    alpha = 0.5
    q = q + alpha * dq
    q = np.minimum(np.maximum(q, model.lowerPositionLimit), model.upperPositionLimit)
# q now is a feasible joint solution candidate; validate in collision checker
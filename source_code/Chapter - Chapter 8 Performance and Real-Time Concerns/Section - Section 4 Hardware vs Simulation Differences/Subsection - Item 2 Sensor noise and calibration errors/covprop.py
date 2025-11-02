import pinocchio as pin
import numpy as np
# model/data load (assume URDF loaded into model)
model = pin.buildModelFromUrdf("robot.urdf")           # robot URDF
data = model.createData()
q = np.array([...])                                   # current joint angles
pin.forwardKinematics(model, data, q)                 # compute poses
pin.updateFramePlacements(model, data)
ee_frame = model.getFrameId("ee_link")
J6 = pin.computeFrameJacobian(model, data, q, ee_frame, pin.LOCAL)  # 6xN
# encoder noise: assume independent variances per joint (rad^2)
sigma_q = 1e-4 * np.ones(model.nq)                    # example std dev
Cov_q = np.diag(sigma_q**2)
Cov_x = J6 @ Cov_q @ J6.T                             # 6x6 spatial covariance
pos_cov = Cov_x[:3, :3]                               # position covariance
# simple Mahalanobis distance for a candidate point p_target
p_ee = data.oMf[ee_frame].translation
p_target = np.array([0.8, 0.0, 0.5])
d = p_target - p_ee
mah2 = d.T @ np.linalg.inv(pos_cov + 1e-12*np.eye(3)) @ d  # numeric regularization
# if mah2 greater than chi2 threshold, treat as uncertain
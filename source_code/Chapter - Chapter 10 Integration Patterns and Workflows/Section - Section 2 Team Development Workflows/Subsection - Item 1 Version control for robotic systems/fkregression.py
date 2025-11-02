# tests/test_fk_regression.py
import numpy as np
import pinocchio as pin
import pytest
# load model and baseline (simple for clarity)
model = pin.buildModelFromUrdf("robot.urdf")  # load URDF
data = model.createData()
q_samples = np.load("tests/baselines/q_samples.npy")  # joint poses
baseline_poses = np.load("tests/baselines/ee_poses.npy")  # [N, 7] quaternions+pos
eps_pos = 1e-3  # 1 mm tolerance
eps_ang = 0.01  # ~0.57 degrees
def pose_distance(p_old, p_new):
    dp = np.linalg.norm(p_old[:3]-p_new[:3])
    # simple angle diff from quaternions
    q_old, q_new = p_old[3:], p_new[3:]
    cos_theta = abs(np.dot(q_old, q_new))
    ang = 2*np.arccos(np.clip(cos_theta, -1.0, 1.0))
    return dp, ang
@pytest.mark.parametrize("i", range(len(q_samples)))
def test_fk_matches_baseline(i):
    q = q_samples[i]
    pin.forwardKinematics(model, data, q)  # compute FK
    H = pin.updateFramePlacement(model, data, model.getFrameId("ee_link"))  # end-effector pose
    p_new = np.hstack((H.translation, pin.Quaternion(H.rotation).coeffs()))  # pos+quat
    p_old = baseline_poses[i]
    dp, ang = pose_distance(p_old, p_new)
    assert dp <= eps_pos and ang <= eps_ang
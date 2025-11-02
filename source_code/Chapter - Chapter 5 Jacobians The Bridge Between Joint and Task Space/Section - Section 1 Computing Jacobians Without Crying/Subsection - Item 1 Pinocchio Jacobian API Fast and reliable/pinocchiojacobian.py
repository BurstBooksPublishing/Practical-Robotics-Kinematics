import pinocchio as pin
import numpy as np
from numpy.linalg import inv

# Load once (startup) -- URDF path and mesh directories as needed.
model = pin.buildModelFromUrdf("path/to/robot.urdf")
data = model.createData()

# Precompute frame id for end-effector.
ee_frame = model.getFrameId("ee_link")

def damped_resolved_rate(q, v_des, lambda2=1e-3, rate_limit=1.0):
    # q: joint positions (n,), v_des: desired spatial twist (6,)
    pin.forwardKinematics(model, data, q)                 # update caches
    pin.updateFramePlacements(model, data)
    J = pin.getFrameJacobian(model, data, ee_frame,
                             pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)  # 6 x n

    # Damped Levenberg-Marquardt pseudoinverse (Equation (1)).
    JJt = J @ J.T
    n = J.shape[1]
    inv_term = inv(JJt + lambda2 * np.eye(6))
    qdot = J.T @ (inv_term @ v_des)   # n-vector

    # Rate limiting (simple clipping).
    qdot = np.clip(qdot, -rate_limit, rate_limit)
    return qdot

# Example call inside control loop:
# q_current = read_joint_positions()  # from robot API
# v_command = np.hstack((omega_des, v_linear_des))  # 6-vector
# qdot_cmd = damped_resolved_rate(q_current, v_command)
# send_joint_velocities(qdot_cmd)
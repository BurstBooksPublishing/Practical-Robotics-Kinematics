import numpy as np
import pinocchio as pin
from scipy.spatial.transform import Rotation as R

# model, data, frame_id, and joint limits are initialized elsewhere
# q: current joint angles (n,), q_limits: (n,2), qdot_max: (n,)

Kp = np.diag([1.0,1.0,1.0, 1.0,1.0,1.0])    # task-space P gain
lambda_damp = 0.05                            # damping scalar
dt = 0.01                                     # control timestep

def compute_task_error(current_pose, desired_pose):
    # current_pose and desired_pose are 4x4 transforms
    p_cur = current_pose[:3,3]; p_des = desired_pose[:3,3]
    # orientation error: rotation that takes current -> desired
    R_cur = R.from_matrix(current_pose[:3,:3])
    R_des = R.from_matrix(desired_pose[:3,:3])
    rotvec_err = (R_des * R_cur.inv()).as_rotvec()  # 3-vector
    trans_err = p_des - p_cur
    return np.hstack((trans_err, rotvec_err))       # 6-vector

def damped_pinv(J, lam):
    # computes J# = J^T (J J^T + lam^2 I)^{-1} stably
    JJt = J @ J.T
    n = J.shape[0]
    reg = (lam**2) * np.eye(n)
    inv = np.linalg.solve(JJt + reg, np.eye(n))
    return J.T @ inv

while True:
    # read sensors / state estimator
    q = read_joint_positions()                   # (n,)
    current_pose = forward_kinematics(q)         # 4x4
    desired_pose, v_des = read_task_command()    # 4x4, 6-vector twist feedforward

    # compute Jacobian in world-aligned spatial convention
    pin.computeJointJacobians(model, data, q)
    pin.updateFramePlacements(model, data)
    J = pin.getFrameJacobian(model, data, frame_id,
                             pin.LOCAL_WORLD_ALIGNED)  # shape (6,n)

    e_twist = compute_task_error(current_pose, desired_pose)
    x_err = v_des + Kp @ e_twist                   # 6-vector velocity-correction

    Jd = damped_pinv(J, lambda_damp)               # (n,6)
    qdot_primary = Jd @ x_err                       # (n,)

    # secondary task: joint limit avoidance (simple gradient)
    q_mid = 0.5*(q_limits[:,0] + q_limits[:,1])
    grad_q = -0.1 * (q - q_mid)                     # drives toward center
    N = np.eye(len(q)) - Jd @ J
    qdot = qdot_primary + N @ grad_q

    # clamp velocities and integrate for position controller
    qdot = np.clip(qdot, -qdot_max, qdot_max)
    q_next = q + qdot * dt
    q_next = np.minimum(np.maximum(q_next, q_limits[:,0]), q_limits[:,1])

    send_position_command(q_next)                   # or convert to lower-level commands
    sleep(dt)                                       # maintain loop timing
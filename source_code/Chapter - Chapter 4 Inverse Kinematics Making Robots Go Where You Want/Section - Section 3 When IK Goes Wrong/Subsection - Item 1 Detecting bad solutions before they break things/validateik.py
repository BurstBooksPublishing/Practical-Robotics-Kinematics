def validate_ik(q_sol, q_seed, model, data, frame_id,
                desired_p, desired_R,
                pb_client, robot_id, obstacle_ids,
                eps_fd=1e-6,
                tol_p=1e-3, tol_R=0.05,
                sigma_min_thresh=1e-3, cond_thresh=1e3,
                max_joint_jump=0.5):
    import numpy as np
    from scipy.spatial.transform import Rotation as R
    import pinocchio as pin

    # 1) Forward kinematics to get end-effector pose
    pin.forwardKinematics(model, data, q_sol)
    pin.updateFramePlacements(model, data)
    M = data.oMf[frame_id]                    # SE3 object
    p = M.translation                         # position vector
    R_cur = M.rotation                        # rotation matrix

    # 2) Pose error (position + rotation vector magnitude)
    pos_err = np.linalg.norm(desired_p - p)
    rotvec_err = R.from_matrix(desired_R.T @ R_cur).as_rotvec()
    rot_err = np.linalg.norm(rotvec_err)
    if pos_err > tol_p or rot_err > tol_R:
        return False, f"pose_error pos={pos_err:.4g}m rot={rot_err:.4g}rad"

    # 3) Joint limits (uses model.lowerPositionLimit / upperPositionLimit)
    if np.any(q_sol < model.lowerPositionLimit - 1e-12) or \
       np.any(q_sol > model.upperPositionLimit + 1e-12):
        return False, "joint_limit_violation"

    # 4) Joint jump safety
    if np.max(np.abs(q_sol - q_seed)) > max_joint_jump:
        return False, f"excessive_joint_jump {np.max(np.abs(q_sol - q_seed)):.3f}rad"

    # 5) Numerical health: finite-difference position Jacobian
    n = q_sol.size
    Jp = np.zeros((3, n))
    pin.forwardKinematics(model, data, q_sol)
    pin.updateFramePlacements(model, data)
    p0 = data.oMf[frame_id].translation
    for i in range(n):
        q_eps = q_sol.copy()
        q_eps[i] += eps_fd
        pin.forwardKinematics(model, data, q_eps)
        pin.updateFramePlacements(model, data)
        pe = data.oMf[frame_id].translation
        Jp[:, i] = (pe - p0) / eps_fd
    # SVD
    U, S, Vt = np.linalg.svd(Jp, full_matrices=False)
    sigma_min = S[-1] if S.size > 0 else 0.0
    cond = (S[0] / S[-1]) if S.size > 0 and S[-1] > 0 else np.inf
    if sigma_min < sigma_min_thresh or cond > cond_thresh:
        return False, f"near_singularity sigma_min={sigma_min:.4g} cond={cond:.4g}"

    # 6) Collision check via PyBullet: ensure no contact with obstacles
    #    pb_client.getClosestPoints(robot_id, obstacle, distance=0) > 0 indicates collision.
    for obs in obstacle_ids:
        pts = pb_client.getClosestPoints(bodyA=robot_id, bodyB=obs, distance=0.0)
        if len(pts) > 0:
            return False, "collision_with_environment"

    return True, "ok"   # accepted
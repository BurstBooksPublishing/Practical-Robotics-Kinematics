# Numeric: use Pinocchio for FK/Jacobian and SciPy for LM solve
def residual(q, model, data, target_pose, frame_id):
    pin.forwardKinematics(model, data, q)               # update kinematic state
    pin.updateFramePlacement(model, data, frame_id)     # compute frame placement
    pos = data.oMf[frame_id].translation                # translation (3,)
    rot = data.oMf[frame_id].rotation.toRotationMatrix() # full orientation if needed
    # stacked position+orientation error as 6D residual
    err = np.concatenate([pos - target_pose[:3],
                          rotation_error(rot, target_pose[3:])])
    return err

# call SciPy least_squares (Levenberg-Marquardt)
res = scipy.optimize.least_squares(residual, q0, args=(model,data,target_pose,frame_id),
                                   method='lm', max_nfev=100)
q_numeric = res.x   # numeric solution, subject to convergence

# Analytic: call a generated IKFast Python module for UR5
# ikfast_ur5.solve returns a list of solution vectors (fast, exact)
solutions = ikfast_ur5.solve(target_pose)  # generated function; returns all branches
q_analytic = select_feasible_solution(solutions, joint_limits)  # filter and select
import numpy as np
import pinocchio as pin

# load model (URDF path must be reproducible in CI)
model = pin.buildModelFromUrdf("robot.urdf")  # CI: use fixed artifact path
data = model.createData()
frame_name = "ee_link"
frame_id = model.getFrameId(frame_name)

def fk(q):
    pin.forwardKinematics(model, data, q)       # compute kinematics
    pin.updateFramePlacements(model, data)      # populate data.oMf
    return data.oMf[frame_id].translation, data.oMf[frame_id].rotation

def analytic_jacobian(q):
    pin.computeJointJacobians(model, data, q)
    # WORLD_ALIGNED Jacobian of the frame in world coordinates
    return pin.getFrameJacobian(model, data, frame_id, pin.ReferenceFrame.WORLD)

# sample configuration
q = np.zeros(model.nq)
epsilon = 1e-8
J_num = np.zeros((6, model.nv))

# central finite differences
for i in range(model.nq):
    dq = np.zeros_like(q); dq[i] = epsilon
    t_plus, R_plus = fk(q + dq)
    t_minus, R_minus = fk(q - dq)
    # position part
    J_num[0:3, i] = (t_plus - t_minus) / (2 * epsilon)
    # orientation via rotation matrix difference mapped to axis-angle approx.
    # small-angle approximation via R_diff ~ I + [w]_x => w ~ vee(R_diff - I)
    Rdiff = R_plus @ R_minus.T
    w = 0.5 * np.array([Rdiff[2,1]-Rdiff[1,2],
                        Rdiff[0,2]-Rdiff[2,0],
                        Rdiff[1,0]-Rdiff[0,1]]) / (2*epsilon)
    J_num[3:6, i] = w

J_analytic = analytic_jacobian(q)
err = np.linalg.norm(J_analytic - J_num)    # Frobenius norm

assert err < 1e-6, f"Jacobian mismatch: {err}"   # fail fast for reviewers
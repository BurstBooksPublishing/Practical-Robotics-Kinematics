from mujoco_py import load_model_from_path, MjSim
import numpy as np

model = load_model_from_path("ur5_mjcf.xml")   # load your model file
sim = MjSim(model)

# helper: read end-effector position from a named site
def ee_pos_from_q(q, site_name="ee_site"):
    sim.data.qpos[:] = q               # set generalized coordinates
    sim.forward()                      # update kinematics (important)
    sid = sim.model.site_name2id(site_name)
    return sim.data.site_xpos[sid].copy()

# central-difference Jacobian
def jacobian_numeric(q, eps=1e-6, site_name="ee_site"):
    n = len(q)
    J = np.zeros((3, n))
    for i in range(n):
        dq = np.zeros_like(q)
        dq[i] = eps
        xp = ee_pos_from_q(q + dq, site_name)
        xm = ee_pos_from_q(q - dq, site_name)
        J[:, i] = (xp - xm) / (2.0 * eps)  # central diff per (1)
    return J

# example usage
q0 = sim.data.qpos.copy()               # current pose
J_est = jacobian_numeric(q0)            # numerical Jacobian
print("Estimated Jacobian shape:", J_est.shape)
import pinocchio as pin
import numpy as np

model = pin.buildModelFromUrdf("URDF_PATH")   # load robot URDF
data = model.createData()

q = np.zeros(model.nq)         # current position
dq = np.zeros(model.nv)        # current velocity
ddq_des = np.zeros(model.nv)   # desired accel from trajectory tracking

# 1) Mass matrix M(q)
pin.crba(model, data, q)       # populates data.M
M = data.M.copy()

# 2) Non-linear effects b = C(q,dq)dq + g(q)
b = pin.nonLinearEffects(model, data, q, dq)

# 3) Inverse dynamics: torques needed for ddq_des
tau_ff = pin.rnea(model, data, q, dq, ddq_des)  # includes b internally

# 4) Apply saturation and combine with feedback
tau_max = 80.0  # N*m per joint example
tau_cmd = np.clip(tau_ff + Kp*(q_des-q) + Kd*(dq_des-dq),
                  -tau_max, tau_max)
# send tau_cmd to an effort controller (ros_control or hardware API)
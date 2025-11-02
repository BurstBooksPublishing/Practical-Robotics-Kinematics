import numpy as np
import pinocchio as pin

def skew(v):                      # helper: 3-vector -> skew matrix
    return np.array([[0, -v[2], v[1]],
                     [v[2], 0, -v[0]],
                     [-v[1], v[0], 0]])

def wrench_transform_to_world(T, w_local):
    # T: pin.SE3 transforming local frame -> world frame (oMf)
    R = T.rotation
    p = T.translation
    Ad = np.zeros((6,6))
    Ad[0:3,0:3] = R
    Ad[3:6,3:6] = R
    Ad[3:6,0:3] = skew(p) @ R
    # wrench_world = (Ad^{-1})^T * w_local
    return np.linalg.inv(Ad).T @ w_local

# load model
model = pin.buildModelFromUrdf("robot.urdf")    # use correct path
data = model.createData()
q = np.zeros(model.nq)                         # example configuration

# compute frame Jacobian for end-effector frame expressed in WORLD
ee_frame = model.getFrameId("ee_link")         # end-effector frame name
pin.forwardKinematics(model, data, q)
pin.updateFramePlacements(model, data)
J = pin.getFrameJacobian(model, data, ee_frame, pin.ReferenceFrame.WORLD)  # 6 x n

# measured wrench from sensor, expressed in sensor frame
w_sensor = np.array([10.0, 0.0, 0.0,   # force N
                     0.0, 0.0, 0.5])   # moment N*m

# transform measured wrench to world frame (frame placement of ee_frame is data.oMf[ee_frame])
T_ee_in_world = data.oMf[ee_frame]             # pin.SE3
w_world = wrench_transform_to_world(T_ee_in_world, w_sensor)

# joint torque mapping
tau = J.T @ w_world                             # n-vector of joint torques
# now tau can be used for monitoring or torque-level control
import pinocchio as pin
import numpy as np

# Load full robot URDF with a floating root joint (free-flyer).
model = pin.buildModelFromUrdf("robot_with_floating_base.urdf")
data = model.createData()

# q_full: generalized coordinates (free-flyer + manipulator)
q_full = pin.neutral(model)            # neutral initializes base + joints
# set base pose if available from odometry (x,y,z,qw,qx,qy,qz)
q_full[:7] = np.array([x,y,z,qw,qx,qy,qz])  # base pose quaternion param.
q_full[7:] = q_manip_angles                 # manipulator joint angles

# Forward kinematics and update placements
pin.forwardKinematics(model, data, q_full)
pin.updateFramePlacements(model, data)

# Get frame ID for the end-effector
ee_frame = model.getFrameId("ee_link")
# Combined spatial Jacobian (6 x nv)
J_comb = pin.getFrameJacobian(model, data, ee_frame,
                              pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)

# Desired end-effector spatial velocity (vx,vy,vz,wx,wy,wz) in world
v_des = np.array([vx, vy, vz, wx, wy, wz])

# Damped least squares for generalized velocity (nv,)
lam = 1e-3
A = J_comb.dot(J_comb.T) + lam * np.eye(6)
qdot = J_comb.T.dot(np.linalg.solve(A, v_des))  # shape (nv,)

# qdot[0:6] are base generalized velocities (free-flyer: 6D)
# qdot[6:] are manipulator joint velocities
# Map base generalized velocity to wheel commands using base kinematics.
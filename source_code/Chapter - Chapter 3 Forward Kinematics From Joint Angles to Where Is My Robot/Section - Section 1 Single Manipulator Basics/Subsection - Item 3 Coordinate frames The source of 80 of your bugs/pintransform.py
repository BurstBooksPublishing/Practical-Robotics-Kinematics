import pinocchio as pin
import numpy as np

# model, data created elsewhere (URDF loaded via pin.buildModelFromUrdf)
# model = pin.buildModelFromUrdf("robot.urdf"); data = model.createData()

q = np.zeros(model.nq)              # current joint configuration
ee_frame_id = model.getFrameId("ee_frame")  # target frame name

pin.forwardKinematics(model, data, q)       # compute q-dependent quantities
pin.updateFramePlacements(model, data)      # populate data.oMf (frame placements)

ee_pose = data.oMf[ee_frame_id]             # SE3: placement of frame in model.root
T_world_ee = ee_pose.homogeneous()         # 4x4 homogeneous matrix

p_ee = np.array([0.0, 0.0, 0.05, 1.0])     # a point 5cm along ee z-axis (homogeneous)
p_world = T_world_ee @ p_ee                # correct transform: world <- ee

# sanity checks
R = T_world_ee[:3,:3]
detR = np.linalg.det(R)                    # should be ~1.0 for proper rotation
assert abs(detR - 1.0) < 1e-6              # catch non-orthonormal rotations early
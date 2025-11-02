import numpy as np
import pinocchio as pin

# model/data loaded from URDF (Pinocchio API)
# model.lowerPositionLimit and model.upperPositionLimit are numpy arrays
q_cmd = np.array([0.5, 1.2, -0.8, 2.4, 3.6, 0.0])  # commanded joints
q_min = model.lowerPositionLimit  # # lower limits from model
q_max = model.upperPositionLimit  # # upper limits from model

# clip command to hardware limits
q_safe = np.minimum(np.maximum(q_cmd, q_min), q_max)  # elementwise clamp

# forward kinematics on safe configuration
pin.forwardKinematics(model, data, q_safe)
pin.updateFramePlacements(model, data)
ee_pose = data.oMf[ee_frame_id]  # # 4x4 SE(3) pose
# ee_pose contains the actual pose the hardware can reach
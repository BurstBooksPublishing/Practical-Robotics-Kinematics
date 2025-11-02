import pinocchio as pin
import numpy as np
import tf2_ros, geometry_msgs.msg, rospy

# Load model (URDF path must be project-specific).
model = pin.buildModelFromUrdf('urdf/ur10.urdf')
data = model.createData()

# get measured joint configuration from your driver (example variable)
q_meas = np.zeros(model.nq)                # # replace with real encoder read

# compute FK to link named 'wrist_3_link' (end-effector flange)
pin.forwardKinematics(model, data, q_meas)
pin.updateFramePlacements(model, data)
frame_id = model.getFrameId('wrist_3_link')
oMf = data.oMf[frame_id]                   # SE3 of EE in world/base

# construct tool offset as SE3 (example 10cm along X)
tool = pin.SE3(np.eye(3), np.array([0.10, 0.0, 0.0]))

# correct EE tool pose in world
pose_tool = oMf * tool                     # pose_tool = T_world_ee * T_ee_tool

# get runtime tf2 transform for 'ee_link' in frame 'world'
rospy.init_node('fk_tf_check', anonymous=True)
tfbuf = tf2_ros.Buffer()
tf2_ros.TransformListener(tfbuf)           # listens to /tf
try:
    tf_msg = tfbuf.lookup_transform('world', 'ee_link', rospy.Time(0), rospy.Duration(1.0))
    # convert to numpy (rotation + translation)
    t = np.array([tf_msg.transform.translation.x,
                  tf_msg.transform.translation.y,
                  tf_msg.transform.translation.z])
    # compare positions (leave rotation comparison for brevity)
    err = np.linalg.norm(pose_tool.translation - t)
    print(f'Position error (m): {err:.4f}')  # small -> consistent frames; large -> mismatch
except Exception as e:
    print('TF lookup failed:', e)
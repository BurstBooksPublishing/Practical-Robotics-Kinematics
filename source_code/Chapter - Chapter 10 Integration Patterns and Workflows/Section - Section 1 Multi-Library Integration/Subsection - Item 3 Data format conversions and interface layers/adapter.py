import numpy as np
import pinocchio as pin
from geometry_msgs.msg import TransformStamped, Pose  # ROS msgs
from tf_transformations import quaternion_from_matrix  # stable quat conv

def se3_to_transform_stamped(se3, parent_frame, child_frame, stamp):
    # se3: pin.SE3 object
    T = se3.homogeneous  # 4x4 numpy array
    q = quaternion_from_matrix(T)  # (x,y,z,w) by tf_transformations
    t = T[:3,3].tolist()
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = parent_frame  # use canonical parent
    msg.child_frame_id = child_frame
    msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = t
    msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w = q
    return msg

def pose_msg_to_pinocchio_SE3(pose_msg):
    # pose_msg: geometry_msgs/Pose
    q = np.array([pose_msg.orientation.x, pose_msg.orientation.y,
                  pose_msg.orientation.z, pose_msg.orientation.w])
    t = np.array([pose_msg.position.x, pose_msg.position.y, pose_msg.position.z])
    R = pin.utils.rotation_from_quaternion(q)  # returns 3x3 rotation
    return pin.SE3(R, t)  # canonical SE3 for internal use
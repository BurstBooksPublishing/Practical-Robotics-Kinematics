import rospy, tf2_ros
import numpy as np
from tf.transformations import quaternion_matrix

# tf buffer setup (assumes a running tf2 broadcaster)
tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

def mat_from_transform(t):
    q = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
    T = quaternion_matrix(q)  # 4x4 homogeneous matrix
    T[0:3,3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
    return T, np.linalg.norm(q)

def check_transform_pair(frame_a, frame_b, tol=1e-6):
    # lookup A->B and B->A
    tab = tf_buffer.lookup_transform(frame_a, frame_b, rospy.Time(0))
    tba = tf_buffer.lookup_transform(frame_b, frame_a, rospy.Time(0))
    TAB, nq_ab = mat_from_transform(tab)
    TBA, nq_ba = mat_from_transform(tba)
    # quaternion norms
    assert nq_ab > 0.999, "Quaternion norm low for A->B"
    assert nq_ba > 0.999, "Quaternion norm low for B->A"
    # round-trip closure
    I_approx = TAB.dot(TBA)
    err = np.linalg.norm(I_approx - np.eye(4))
    assert err < tol, "Round-trip error too large: {:.3e}".format(err)
    # orthogonality of rotation
    R = TAB[0:3,0:3]
    U, S, Vt = np.linalg.svd(R)
    R_proj = U.dot(Vt)
    ortho_err = np.linalg.norm(R - R_proj)
    assert ortho_err < tol, "Rotation not orthonormal"
    return True

# usage example (catch exceptions in production)
# check_transform_pair('base_link', 'wrist_link')
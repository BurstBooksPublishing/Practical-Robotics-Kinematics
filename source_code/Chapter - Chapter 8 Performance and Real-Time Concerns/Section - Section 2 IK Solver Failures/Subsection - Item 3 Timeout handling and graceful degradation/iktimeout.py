import rospy
import numpy as np
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from geometry_msgs.msg import PoseStamped
from concurrent.futures import ThreadPoolExecutor, TimeoutError

# helper: compute DLS delta using numeric Jacobian from Pinocchio or finite diff
def damped_least_squares(J, e, lam=0.1):
    # J: (m x n), e: (m,), lam: damping scalar
    JJt = J.dot(J.T)
    reg = (lam**2) * np.eye(JJt.shape[0])
    inv = np.linalg.solve(JJt + reg, np.eye(JJt.shape[0]))  # stable solve
    dq = J.T.dot(inv.dot(e))
    return dq  # joint-space increment

def try_moveit_ik(pose_stamped, timeout_s=0.2):
    # build service request (fill fields as needed)
    req = GetPositionIKRequest()
    req.ik_request.pose_stamped = pose_stamped
    req.ik_request.ik_link_name = "ee_link"
    req.timeout = rospy.Duration(timeout_s)
    # call service in thread to avoid blocking ROS spin
    with ThreadPoolExecutor(max_workers=1) as ex:
        fut = ex.submit(rospy.ServiceProxy('/compute_ik', GetPositionIK), req)
        try:
            resp = fut.result(timeout=timeout_s + 0.1)  # small buffer
            return resp  # check resp.error_code afterward
        except TimeoutError:
            return None  # service blocked or long-running
        except rospy.ServiceException:
            return None  # service error

# usage in a control loop
pose = PoseStamped()  # filled from higher-level motion planner
resp = try_moveit_ik(pose, timeout_s=0.1)  # very short timeout for control loop
if resp is not None and resp.error_code.val == resp.error_code.SUCCESS:
    q_sol = np.array(resp.solution.joint_state.position)  # use IK result
else:
    # fallback: compute DLS from current joint state q0 and Jacobian J
    q0 = get_current_joint_state()  # implement with rospy/RobotState
    J = compute_jacobian_numeric(q0)  # use Pinocchio or finite differences
    e = pose_to_task_error(pose, forward_kinematics(q0))
    dq = damped_least_squares(J, e, lam=0.2)
    q_sol = q0 + np.clip(dq, -0.1, 0.1)  # apply bounded motion step
    # validate q_sol for joint limits and collisions; otherwise hold position
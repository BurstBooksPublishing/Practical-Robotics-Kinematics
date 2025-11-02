# high-level imports and ROS init (abbrev). # uses moveit_commander API.
from moveit_commander import MoveGroupCommander, RobotCommander
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

rospy.init_node('moveit_ik_example')
robot = RobotCommander()
group = MoveGroupCommander("manipulator")  # e.g., UR5 group name

# 1) Try the easy button: set pose and go (planner + IK integration).
target = PoseStamped()  # fill header.frame_id, position, orientation
target.header.frame_id = "base_link"
target.pose.position.x = 0.6; target.pose.position.y = 0.0; target.pose.position.z = 0.4
target.pose.orientation.w = 1.0

group.set_pose_target(target)               # high-level target
group.set_start_state_to_current_state()    # good seed
group.set_planning_time(2.0)                # seconds
ok = group.go(wait=True)                    # returns True on success

if not ok:
    # 2) Fallback: direct IK service call with explicit seed and timeout.
    ik_srv = rospy.ServiceProxy('/compute_ik', GetPositionIK)
    req = GetPositionIKRequest()
    req.ik_request.group_name = "manipulator"
    req.ik_request.pose_stamped = target
    # seed the solver with current joint positions
    js = JointState()
    js.name = robot.get_current_state().joint_state.name
    js.position = robot.get_current_state().joint_state.position
    req.ik_request.robot_state.joint_state = js
    req.ik_request.timeout.secs = 1
    req.ik_request.attempts = 3
    resp = ik_srv(req)                       # synchronous call
    if resp.error_code.val == resp.error_code.SUCCESS:
        joint_solution = resp.solution.joint_state
        # apply and execute solution (with collision checking)
        group.set_joint_value_target(joint_solution)
        group.go(wait=True)
    else:
        rospy.logwarn("IK failed; try alternative seeds or optimizer")
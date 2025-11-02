import sys, rospy, numpy as np
import moveit_commander  # MoveIt Python API
import pinocchio as pin   # Pinocchio Python binding

# ROS + MoveIt initialization
rospy.init_node('pinocchio_moveit_integration')            # ROS node
moveit_commander.roscpp_initialize(sys.argv)
group = moveit_commander.MoveGroupCommander("manipulator") # change group name as needed

# Load URDF string from ROS param and build Pinocchio model (write to temp file if needed)
urdf = rospy.get_param('/robot_description')
with open('/tmp/robot.urdf', 'w') as f: f.write(urdf)
model = pin.buildModelFromUrdf('/tmp/robot.urdf')          # build model from file
data  = pin.createData(model)                              # Pinocchio data structure

# Get current joint values from MoveIt (assumes joint ordering matches Pinocchio active joints)
q_moveit = np.array(group.get_current_joint_values())     # list -> numpy
q = q_moveit.copy()                                       # assume compatible ordering; verify in practice

# Forward kinematics and frame Jacobian
eef_frame = model.getFrameId("ee_link")                   # replace with your end-effector frame
pin.forwardKinematics(model, data, q)                     # compute kinematics
pin.updateFramePlacements(model, data)                    # refresh frame placements
oMf = data.oMf[eef_frame]                                 # SE3 object: pose of frame
p_cur = oMf.translation                                   # 3-vector
R_cur = oMf.rotation                                      # 3x3 rotation matrix

# Desired pose (example): translate 5 cm along z in world frame
p_des = p_cur + np.array([0.0, 0.0, 0.05])
R_des = R_cur                                             # keep same orientation in this example

# Compute pose error per eq (1)
skew = R_cur.T.dot(R_des) - R_des.T.dot(R_cur)
rot_err = 0.5 * np.array([skew[2,1], skew[0,2], skew[1,0]])
delta_x = np.hstack((p_des - p_cur, rot_err))

# Jacobian (world aligned) and DLS solve per eq (2)
pin.computeJointJacobians(model, data, q)
J = pin.getFrameJacobian(model, data, eef_frame, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
J6 = J  # 6 x n matrix
lambda_damp = 1e-2
n = J6.shape[1]
dq = np.linalg.solve(J6.T.dot(J6) + (lambda_damp**2)*np.eye(n), J6.T.dot(delta_x))

# Warm-start MoveIt with the seed
seed = (q_moveit + dq).tolist()
group.set_joint_value_target(seed)                        # seed for planning
plan = group.plan()                                       # request a plan
# group.execute(plan, wait=True)                          # execute when ready (careful on hardware)
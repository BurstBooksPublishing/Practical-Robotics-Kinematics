#!/usr/bin/env python3
import rospy
import moveit_commander                         # MoveIt high-level API
from sensor_msgs.msg import JointState
from kdl_parser_py.urdf import treeFromParam    # parses /robot_description
import PyKDL as kdl

rospy.init_node('moveit_kdl_example')

# MoveIt setup (high-level planning)
moveit_commander.roscpp_initialize([])
arm = moveit_commander.MoveGroupCommander("manipulator")  # group name in SRDF

# Parse URDF into KDL tree (for deterministic kinematics)
ok, kdl_tree = treeFromParam('/robot_description')       # parse from param server
chain = kdl_tree.getChain("base_link", "ee_link")        # base->end effector

# Prepare KDL solvers
fk_solver = kdl.ChainFkSolverPos_recursive(chain)
jac_solver = kdl.ChainJntToJacSolver(chain)

# Example joint vector from current MoveIt state
current = arm.get_current_state()                        # moveit_msgs/RobotState
joints = current.joint_state.position                   # tuple of positions
n = chain.getNrOfJoints()
q = kdl.JntArray(n)
for i in range(n): q[i] = joints[i]                      # fill KDL joint array

# Compute FK
end_frame = kdl.Frame()
fk_solver.JntToCart(q, end_frame)                        # FK solver call

# Compute Jacobian
J = kdl.Jacobian(n)
jac_solver.JntToJac(q, J)                                # fill J with 6xn entries

# Use MoveIt for a planned Cartesian pose (convenience)
target = arm.get_current_pose().pose
target.position.z += 0.05                                # small Cartesian offset
arm.set_pose_target(target)
plan = arm.plan()                                        # blocking planning call
# send plan with arm.execute(plan) in production
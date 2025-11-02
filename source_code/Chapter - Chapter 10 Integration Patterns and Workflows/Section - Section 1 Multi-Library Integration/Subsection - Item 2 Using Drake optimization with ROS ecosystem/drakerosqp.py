#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
import numpy as np
import pinocchio as pin
from pydrake.all import MathematicalProgram, Solve  # Drake API

# --- Configuration (replace with your robot specifics) ---
URDF_PATH = "/path/to/robot.urdf"
EE_FRAME = "ee_link"            # end-effector frame name
JOINT_NAMES = ["joint1","joint2","joint3","joint4","joint5","joint6","joint7"]
n = len(JOINT_NAMES)
dt = 0.01                       # control interval
alpha = 1e-3                    # regularization weight
W = np.eye(6)                   # spatial velocity weight (6x6)
qdot_min = -1.0*np.ones(n)
qdot_max =  1.0*np.ones(n)
q_min = -np.pi*np.ones(n)
q_max =  np.pi*np.ones(n)

# --- Load model (Pinocchio) ---
model = pin.buildModelFromUrdf(URDF_PATH)
data = model.createData()
frame_id = model.getFrameId(EE_FRAME)

# --- State holders ---
current_q = np.zeros(n)
current_q_received = False
desired_twist = np.zeros(6)

def joint_state_cb(msg):
    global current_q, current_q_received
    # map incoming JointState to current_q using JOINT_NAMES
    name_to_idx = {name: i for i, name in enumerate(msg.name)}
    for i, jn in enumerate(JOINT_NAMES):
        if jn in name_to_idx:
            current_q[i] = msg.position[name_to_idx[jn]]
    current_q_received = True

def twist_cb(msg):
    # simple subscriber: store desired spatial twist in EE frame
    v = np.concatenate([ [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z],
                         [msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z] ])
    global desired_twist
    desired_twist = v

def compute_qdot_via_drake(q, v_des):
    # compute geometric Jacobian at frame origin (6 x n)
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacement(model, data, frame_id)
    # reference_frame=pin.LOCAL_WORLD_ALIGNED gives spatial jacobian in world
    J = pin.getFrameJacobian(model, data, frame_id, pin.LOCAL_WORLD_ALIGNED)  # 6xn
    # Build Drake program
    prog = MathematicalProgram()
    qdot = prog.NewContinuousVariables(n, "qdot")
    # Quadratic cost: ||J qdot - v_des||_W^2 + alpha ||qdot||^2
    # Expand: (J^T W J + alpha I) x^2 - 2 v^T W J x  (constant dropped)
    H = J.T.dot(W).dot(J) + alpha * np.eye(n)
    g = - (v_des.T.dot(W).dot(J)).reshape((n,))  # linear term
    prog.AddQuadraticCost(H, g, qdot)
    # Box constraints on qdot
    prog.AddBoundingBoxConstraint(qdot_min, qdot_max, qdot)
    # Optional: enforce integrated joint limits q + dt*qdot in [q_min, q_max]
    prog.AddBoundingBoxConstraint(q_min - q*1.0, q_max - q*1.0, qdot)  # linearized with dt=1; scale by dt below
    # Solve
    result = Solve(prog)
    if not result.is_success():
        rospy.logwarn("Drake solver failed; returning zeros")
        return np.zeros(n)
    sol = result.GetSolution(qdot) * dt  # scale if dt used in constraints differently
    return sol

def main():
    rospy.init_node("drake_qp_controller")
    rospy.Subscriber("/joint_states", JointState, joint_state_cb)
    rospy.Subscriber("/cmd_twist", TwistStamped, twist_cb)
    pub = rospy.Publisher("/cmd_joint_states", JointState, queue_size=1)
    rate = rospy.Rate(1.0/dt)
    while not rospy.is_shutdown():
        if not current_q_received:
            rate.sleep(); continue
        qdot = compute_qdot_via_drake(current_q, desired_twist)
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name = JOINT_NAMES
        js.velocity = list(qdot)   # downstream controller consumes velocities
        pub.publish(js)
        rate.sleep()

if __name__ == "__main__":
    main()
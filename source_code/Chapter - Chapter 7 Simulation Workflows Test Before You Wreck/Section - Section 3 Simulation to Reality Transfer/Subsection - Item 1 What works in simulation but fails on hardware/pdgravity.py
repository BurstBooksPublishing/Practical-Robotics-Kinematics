import rospy
import numpy as np
import pinocchio as pin
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

# --- parameters (tune these conservatively on hardware) ---
Kp = np.array([80.0, 80.0, 50.0, 40.0, 20.0, 10.0])   # proportional gains
Kd = 2.0 * np.sqrt(Kp)                                 # derivative gains (critical damping approx)
tau_max = np.array([80, 80, 60, 40, 20, 10])           # per-joint torque limits (Nm)
tau_rate_max = np.array([50, 50, 40, 30, 20, 10])      # Nm/s rate limit
dt = 0.01                                              # control loop period (s)

# --- setup robot model (URDF previously loaded) ---
model = pin.buildModelFromUrdf("robot.urdf")
data = model.createData()

pub = rospy.Publisher('/joint_effort_controller/command', Float64MultiArray, queue_size=1)

last_tau = np.zeros(model.nq)

def joint_state_cb(msg):
    q = np.array(msg.position)
    qdot = np.array(msg.velocity)
    # desired states (from planner) should be provided; here we use a simple hold
    q_des = q.copy()                 # example placeholder: hold current position
    qdot_des = np.zeros_like(q)
    qddot_des = np.zeros_like(q)

    # compute dynamics terms via Pinocchio
    pin.computeAllTerms(model, data, q, qdot)
    g = data.g  # gravity vector
    # simple PD + gravity feedforward
    e = q_des - q
    edot = qdot_des - qdot
    tau_ff = g                       # gravity compensation
    tau_cmd = tau_ff + Kp * e + Kd * edot

    # torque saturation and rate limiting
    tau_cmd = np.clip(tau_cmd, -tau_max, tau_max)
    tau_rate = (tau_cmd - last_tau) / dt
    tau_rate = np.clip(tau_rate, -tau_rate_max, tau_rate_max)
    tau_cmd_limited = last_tau + tau_rate * dt

    # publish
    msg_out = Float64MultiArray(data=tau_cmd_limited.tolist())
    pub.publish(msg_out)

    # update
    global last_tau
    last_tau = tau_cmd_limited

# ROS node spin
rospy.init_node('pd_gravity_controller')
rospy.Subscriber('/joint_states', JointState, joint_state_cb)
rospy.spin()
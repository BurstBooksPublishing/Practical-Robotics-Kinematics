import pybullet as p
import pybullet_data
import numpy as np
p.connect(p.DIRECT)                     # headless fast mode
p.setAdditionalSearchPath(pybullet_data.getDataPath())
robot = p.loadURDF("ur5/ur5.urdf", useFixedBase=True)  # realistic industrial arm

# Simulation parameters
dt = 1.0/1000.0                          # 1 kHz physics step
p.setTimeStep(dt)
p.setRealTimeSimulation(0)
p.setPhysicsEngineParameter(numSolverIterations=50)    # improve joint accuracy

# Cache joint indices and limits
joint_indices = [i for i in range(p.getNumJoints(robot)) if p.getJointInfo(robot, i)[2] != p.JOINT_FIXED]
n = len(joint_indices)

# PD gains (vectorized)
Kp = np.array([200.0]*n)
Kd = np.array([10.0]*n)

# Desired trajectory: hold a small offset on joint 2 while others constant
q_des = np.zeros(n)
q_des[1] = 0.6

for iter in range(10000):
    # Read joint states in batch
    states = p.getJointStates(robot, joint_indices)         # returns tuples
    q = np.array([s[0] for s in states])
    qd = np.array([s[1] for s in states])

    # Simple PD control law (eq. reference) -> torque vector
    tau = Kp * (q_des - q) + Kd * (0.0 - qd)

    # Clip torques to realistic actuator limits
    tau = np.clip(tau, -50.0, 50.0)

    # Apply torques using array API for fewer calls
    p.setJointMotorControlArray(robot, joint_indices,
                                controlMode=p.TORQUE_CONTROL,
                                forces=tau.tolist())    # list accepted by PyBullet

    p.stepSimulation()    # advance physics by dt
# end loop
p.disconnect()